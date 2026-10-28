#include "core/iq_recorder.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>

#if defined(_WIN32)
#include <direct.h>
#define IQR_MKDIR(p) _mkdir(p)
#define IQR_SEEK(f, o) _fseeki64(f, (long long)(o), SEEK_SET)
#else
#include <sys/stat.h>
#define IQR_MKDIR(p) mkdir(p, 0755)
#define IQR_SEEK(f, o) fseeko(f, (off_t)(o), SEEK_SET)
#endif

namespace iridi {

namespace {
// Keep each file under 2 GB of samples (WAV sizes are 32-bit; staying below
// 2^31 also keeps every tool's seeking happy).
constexpr uint64_t kMaxFileBytes = 2000000000ULL;
// Gaps up to this long are written as silence; longer ones start a new file.
constexpr double kMaxSilenceS = 2.0;
// Backlog allowed if the disk falls behind before blocks are dropped.
constexpr size_t kMaxQueueBytes = (size_t)1 << 30;

uint64_t now_ns() {
    return (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch()).count();
}

void put_header(FILE *f, int rate, uint64_t data_bytes) {
    const uint32_t data = (uint32_t)data_bytes, riff = 36 + data, fsz = 16;
    const uint32_t r = (uint32_t)rate, byte_rate = r * 4;
    const uint16_t fmt = 1, ch = 2, block_align = 4, bits = 16;
    std::fwrite("RIFF", 1, 4, f); std::fwrite(&riff, 4, 1, f); std::fwrite("WAVE", 1, 4, f);
    std::fwrite("fmt ", 1, 4, f); std::fwrite(&fsz, 4, 1, f); std::fwrite(&fmt, 2, 1, f);
    std::fwrite(&ch, 2, 1, f); std::fwrite(&r, 4, 1, f); std::fwrite(&byte_rate, 4, 1, f);
    std::fwrite(&block_align, 2, 1, f); std::fwrite(&bits, 2, 1, f);
    std::fwrite("data", 1, 4, f); std::fwrite(&data, 4, 1, f);
}
} // namespace

bool IqRecorder::start(const std::string &dir, int sample_rate, double center_hz, std::string &err) {
    stop();
    if (sample_rate <= 0) { err = "no sample rate"; return false; }
    dir_ = dir;
    rate_ = sample_rate;
    center_hz_ = center_hz;
    // Create each level of the folder (e.g. recordings/iq).
    for (size_t i = 0; i <= dir.size(); i++)
        if (i == dir.size() || dir[i] == '/' || dir[i] == '\\') IQR_MKDIR(dir.substr(0, i).c_str());
    stream_samples_ = 0;
    file_samples_ = 0;
    start_ns_ = now_ns();
    bytes_.store(0); samples_.store(0); dropped_blocks_.store(0); files_.store(0);
    queued_bytes_ = 0;
    stopping_ = false;
    if (!open_file(start_ns_)) { err = "can't create a file in " + dir; return false; }
    writer_ = std::thread(&IqRecorder::writer_main, this);
    active_.store(true);
    return true;
}

void IqRecorder::stop() {
    if (!writer_.joinable()) { active_.store(false); return; }
    active_.store(false);
    {
        std::lock_guard<std::mutex> lk(mtx_);
        stopping_ = true;
    }
    cv_.notify_all();
    writer_.join();
    close_file();
}

std::string IqRecorder::current_file() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return file_;
}

void IqRecorder::push(const float *iq, size_t n, uint64_t gap_before) {
    if (!active_.load() || n == 0) return;
    std::deque<Block> out;
    bool split = false;
    if (gap_before) {
        stream_samples_ += gap_before;
        if (gap_before <= (uint64_t)(kMaxSilenceS * rate_) &&
            (file_samples_ + gap_before) * 4 < kMaxFileBytes) {
            Block z;
            z.s.assign((size_t)gap_before * 2, 0);   // silence keeps time in the file right
            file_samples_ += gap_before;
            out.push_back(std::move(z));
        } else {
            split = true;
        }
    }
    Block b;
    b.s.resize(n * 2);
    for (size_t i = 0; i < n * 2; i++) {
        const float v = iq[i] * 32768.0f;
        b.s[i] = (int16_t)std::lrintf(std::max(-32768.0f, std::min(32767.0f, v)));
    }
    if (split || (file_samples_ + n) * 4 > kMaxFileBytes) {
        b.new_file = true;
        b.start_ns = start_ns_ + (uint64_t)((double)stream_samples_ * 1e9 / rate_);
        file_samples_ = 0;
    }
    file_samples_ += n;
    stream_samples_ += n;
    out.push_back(std::move(b));

    {
        std::lock_guard<std::mutex> lk(mtx_);
        size_t add = 0;
        for (auto &x : out) add += x.s.size() * 2;
        if (queued_bytes_ + add > kMaxQueueBytes) {
            // Disk can't keep up: drop this block; the next one starts a new
            // file at its true time so no file has a hidden jump.
            dropped_blocks_.fetch_add(1);
            file_samples_ = (uint64_t)-1 / 8; // forces a split on the next push
            return;
        }
        for (auto &x : out) { queued_bytes_ += x.s.size() * 2; queue_.push_back(std::move(x)); }
    }
    cv_.notify_one();
}

bool IqRecorder::open_file(uint64_t start_ns) {
    close_file();
    const std::time_t t = (std::time_t)(start_ns / 1000000000ULL);
    std::tm u{};
#if defined(_WIN32)
    gmtime_s(&u, &t);
#else
    gmtime_r(&t, &u);
#endif
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%SZ", &u);
    char name[512];
    std::snprintf(name, sizeof(name), "%s/IridiScope_%s_%.0fHz_IQ.wav", dir_.c_str(), stamp, center_hz_);
    FILE *f = std::fopen(name, "wb");
    if (!f) return false;
    put_header(f, rate_, 0);
    std::lock_guard<std::mutex> lk(mtx_);
    f_ = f;
    file_ = name;
    data_bytes_ = 0;
    since_header_ = 0;
    files_.fetch_add(1);
    return true;
}

void IqRecorder::close_file() {
    FILE *f;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        f = f_;
        f_ = nullptr;
    }
    if (!f) return;
    IQR_SEEK(f, 0);
    put_header(f, rate_, data_bytes_);
    std::fclose(f);
}

void IqRecorder::writer_main() {
    for (;;) {
        Block b;
        {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait(lk, [this] { return stopping_ || !queue_.empty(); });
            if (queue_.empty()) return;
            b = std::move(queue_.front());
            queue_.pop_front();
            queued_bytes_ -= b.s.size() * 2;
        }
        if (b.new_file && !open_file(b.start_ns)) continue;
        if (!f_) continue;
        const size_t bytes = b.s.size() * 2;
        if (std::fwrite(b.s.data(), 1, bytes, f_) != bytes) continue;
        data_bytes_ += bytes;
        bytes_.fetch_add(bytes);
        samples_.fetch_add(b.s.size() / 2);
        // Keep the header's sizes current so a crash still leaves a playable file.
        if (++since_header_ >= 64) {
            since_header_ = 0;
            IQR_SEEK(f_, 0);
            put_header(f_, rate_, data_bytes_);
            IQR_SEEK(f_, 44 + data_bytes_);
        }
    }
}

} // namespace iridi
