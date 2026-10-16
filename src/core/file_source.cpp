#include "core/file_source.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <regex>
#include <vector>

namespace iridi {

const char *iq_format_name(IqFormat f) {
    switch (f) {
    case IqFormat::CI8:  return "ci8";
    case IqFormat::CI16: return "ci16";
    case IqFormat::CF32: return "cf32";
    default:             return "unknown";
    }
}

int iq_format_bytes_per_sample(IqFormat f) {
    switch (f) {
    case IqFormat::CI8:  return 2;
    case IqFormat::CI16: return 4;
    case IqFormat::CF32: return 8;
    default:             return 0;
    }
}

IqFileSource::~IqFileSource() { close(); }

void IqFileSource::close() {
    if (fp_) {
        fclose(fp_);
        fp_ = nullptr;
    }
    info_ = IqFileInfo{};
    pos_sample_ = 0;
}

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rd32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

double parse_center_freq_from_name(const std::string &path) {
    // Look for the last "<number>Hz" occurrence, e.g. baseband_1623326935Hz_...
    static const std::regex re(R"((\d{6,12}(?:\.\d+)?)\s*[Hh][Zz])");
    std::smatch m;
    double best = 0.0;
    std::string s = path;
    auto it = std::sregex_iterator(s.begin(), s.end(), re);
    for (; it != std::sregex_iterator(); ++it) {
        try { best = std::stod((*it)[1].str()); } catch (...) {}
    }
    return best;
}

// ---- WAV parsing -----------------------------------------------------------

static bool detect_by_extension(const std::string &path, IqFormat &fmt) {
    std::string ext;
    size_t dot = path.find_last_of('.');
    if (dot != std::string::npos) {
        ext = path.substr(dot + 1);
        for (auto &c : ext) c = (char)std::tolower((unsigned char)c);
    }
    if (ext == "cf32" || ext == "fc32" || ext == "cfile" || ext == "raw") {
        fmt = IqFormat::CF32;
        return true;
    }
    if (ext == "ci16" || ext == "cs16" || ext == "sc16") {
        fmt = IqFormat::CI16;
        return true;
    }
    if (ext == "ci8" || ext == "cs8" || ext == "sc8") {
        fmt = IqFormat::CI8;
        return true;
    }
    return false;
}

bool IqFileSource::open(const std::string &path, std::string &err) {
    close();
    err.clear();

    FILE *f = fopen(path.c_str(), "rb");
    if (!f) {
        err = "Cannot open file: " + path;
        return false;
    }
#ifdef _WIN32
    _fseeki64(f, 0, SEEK_END);
    uint64_t file_size = (uint64_t)_ftelli64(f);
#else
    fseeko(f, 0, SEEK_END);
    uint64_t file_size = (uint64_t)ftello(f);
#endif
    _fseeki64(f, 0, SEEK_SET);

    info_ = IqFileInfo{};
    info_.path = path;
    info_.file_size = file_size;
    info_.center_frequency = parse_center_freq_from_name(path);

    uint8_t magic[12] = {0};
    size_t got = fread(magic, 1, 12, f);
    bool is_riff = (got == 12 && memcmp(magic, "RIFF", 4) == 0 &&
                    memcmp(magic + 8, "WAVE", 4) == 0);

    int channels = 2, bytes_per_channel = 2;
    bool wav_unsigned8 = false, pcm32 = false;
    int sample_rate = 0;
    IqFormat fmt = IqFormat::UNKNOWN;
    uint64_t data_offset = 0, data_bytes = 0;

    if (is_riff) {
        info_.is_wav = true;
        // Walk chunks.
#ifdef _WIN32
        _fseeki64(f, 12, SEEK_SET);
#else
        fseeko(f, 12, SEEK_SET);
#endif
        uint8_t hdr[8];
        bool have_fmt = false;
        bool have_data = false;
        while (fread(hdr, 1, 8, f) == 8) {
            char id[5] = {(char)hdr[0], (char)hdr[1], (char)hdr[2], (char)hdr[3], 0};
            uint32_t sz = rd32(hdr + 4);
            if (memcmp(id, "fmt ", 4) == 0 && sz >= 16) {
                uint8_t fb[64] = {0};
                uint32_t want = std::min<uint32_t>(sz, sizeof(fb));
                if (fread(fb, 1, want, f) != want) break;
                uint16_t af = rd16(fb + 0);
                uint16_t ch = rd16(fb + 2);
                uint32_t sr = rd32(fb + 4);
                uint16_t bits = rd16(fb + 14);
                if (af == 0xFFFE && sz >= 40) af = rd16(fb + 24); // subformat GUID
                channels = ch;
                sample_rate = (int)sr;
                bytes_per_channel = bits / 8;
                if (af == 3 && bits == 32) {
                    fmt = IqFormat::CF32;
                } else if (af == 1) {
                    if (bits == 8)       { fmt = IqFormat::CI8; wav_unsigned8 = true; }
                    else if (bits == 16) { fmt = IqFormat::CI16; }
                    else if (bits == 32) { fmt = IqFormat::CF32; pcm32 = true; }
                    else { fmt = IqFormat::UNKNOWN; }
                }
                // Skip any remaining fmt bytes (with pad to even).
                uint32_t consumed = want;
                if (consumed < sz) {
                    uint32_t skip = sz - consumed + (sz & 1);
#ifdef _WIN32
                    _fseeki64(f, skip, SEEK_CUR);
#else
                    fseeko(f, skip, SEEK_CUR);
#endif
                } else if (sz & 1) {
                    fseek(f, 1, SEEK_CUR);
                }
                have_fmt = true;
            } else if (memcmp(id, "data", 4) == 0) {
#ifdef _WIN32
                data_offset = (uint64_t)_ftelli64(f);
#else
                data_offset = (uint64_t)ftello(f);
#endif
                data_bytes = sz;
                have_data = true;
                break;
            } else {
                uint64_t skip = sz + (sz & 1);
#ifdef _WIN32
                _fseeki64(f, (int64_t)skip, SEEK_CUR);
#else
                fseeko(f, (int64_t)skip, SEEK_CUR);
#endif
            }
        }
        if (!have_fmt || !have_data || fmt == IqFormat::UNKNOWN) {
            fclose(f);
            err = "Unsupported or malformed WAV file";
            return false;
        }
        // Handle 32-bit overflowed chunk-size fields (files > 4 GB). The RIFF
        // "data" size is a 32-bit field, so on large recordings it wraps at
        // 2^32. In that case the true payload runs to EOF.
        uint64_t remaining = (file_size > data_offset) ? file_size - data_offset : 0;
        if (data_bytes == 0 || data_offset + data_bytes > file_size) {
            data_bytes = remaining;
        } else if (remaining > data_bytes) {
            uint64_t diff = remaining - data_bytes;
            if (diff != 0 && (diff % 0x100000000ULL) == 0)
                data_bytes = remaining; // wrapped size field
        }
    } else {
        // Headerless raw IQ.
        if (!detect_by_extension(path, fmt)) {
            // Default to ci8, matching common conventions.
            fmt = IqFormat::CI8;
        }
        channels = 2;
        bytes_per_channel = iq_format_bytes_per_sample(fmt) / 2;
        if (info_.center_frequency == 0.0) info_.center_frequency = 0.0;
        data_offset = 0;
        data_bytes = file_size;
    }

    if (channels < 1) channels = 1;
    if (bytes_per_channel < 1) {
        fclose(f);
        err = "Invalid sample size";
        return false;
    }

    uint64_t stride = (uint64_t)channels * bytes_per_channel;
    uint64_t frames = (stride > 0) ? data_bytes / stride : 0;

    info_.format = fmt;
    info_.sample_rate = sample_rate;
    info_.channels = channels;
    info_.bits_per_sample = bytes_per_channel * 8;
    info_.data_offset = data_offset;
    info_.data_bytes = data_bytes;
    info_.num_samples = frames;
    info_.duration_s = sample_rate > 0 ? (double)frames / sample_rate : 0.0;

    // Stash private conversion details in the info-independent members.
    wav_unsigned8_ = wav_unsigned8;
    pcm32_ = pcm32;
    channels_ = channels;
    bytes_per_channel_ = bytes_per_channel;
    format_ = fmt;

#ifdef _WIN32
    _fseeki64(f, (int64_t)data_offset, SEEK_SET);
#else
    fseeko(f, (int64_t)data_offset, SEEK_SET);
#endif
    pos_sample_ = 0;
    fp_ = f;
    return true;
}

static float read_channel_value(const uint8_t *p, IqFormat fmt, bool wav_unsigned8,
                                bool pcm32) {
    switch (fmt) {
    case IqFormat::CI8: {
        int v = wav_unsigned8 ? ((int)p[0] - 128) : (int)(int8_t)p[0];
        return (float)v / 128.0f;
    }
    case IqFormat::CI16: {
        int16_t v = (int16_t)(p[0] | (p[1] << 8));
        return (float)v / 32768.0f;
    }
    case IqFormat::CF32: {
        if (pcm32) {
            int32_t v = (int32_t)rd32(p);
            return (float)(v / 2147483648.0);
        }
        float v;
        memcpy(&v, p, 4);
        return v;
    }
    default:
        return 0.0f;
    }
}

size_t IqFileSource::read_cf32(float *out, size_t max_samples) {
    if (!fp_ || max_samples == 0) return 0;
    const size_t stride = (size_t)channels_ * (size_t)bytes_per_channel_;
    if (stride == 0) return 0;

    std::vector<uint8_t> buf(max_samples * stride);
    size_t got = fread(buf.data(), 1, buf.size(), fp_);
    size_t frames = got / stride;

    for (size_t k = 0; k < frames; ++k) {
        const uint8_t *p = buf.data() + k * stride;
        float i = read_channel_value(p, format_, wav_unsigned8_, pcm32_);
        float q = 0.0f;
        if (channels_ >= 2)
            q = read_channel_value(p + bytes_per_channel_, format_, wav_unsigned8_, pcm32_);
        out[2 * k] = i;
        out[2 * k + 1] = q;
    }
    pos_sample_ += frames;
    return frames;
}

bool IqFileSource::seek_sample(uint64_t sample_index) {
    if (!fp_) return false;
    const size_t stride = (size_t)channels_ * (size_t)bytes_per_channel_;
    uint64_t off = info_.data_offset + sample_index * stride;
    if (off > info_.data_offset + info_.data_bytes) return false;
#ifdef _WIN32
    if (_fseeki64(fp_, (int64_t)off, SEEK_SET) != 0) return false;
#else
    if (fseeko(fp_, (int64_t)off, SEEK_SET) != 0) return false;
#endif
    pos_sample_ = sample_index;
    return true;
}

} // namespace iridi
