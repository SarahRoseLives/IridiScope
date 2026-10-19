// IridiScope - Iridium signal analysis workstation.
//
// Milestone 2: the vendored DSP pipeline runs over the IQ file and drives a
// live spectrum (ImPlot), scrolling waterfall, and burst statistics.
#include <windows.h>
#include <commdlg.h>
#include <shellapi.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include "imgui.h"
#include "imgui_internal.h"
#include "implot.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

#include "core/file_source.h"
#include "core/engine.h"
#include "acars/aircraft_lookup.h"
#include "sdr/live_iq_source.h"
#include "ui/waterfall.h"
#include "update/version_check.h"
#include "util/iritime.h"
#include "voice/audio.h"

#ifndef IRIDISCOPE_ASSETS
#define IRIDISCOPE_ASSETS "assets"
#endif

#ifndef IRIDISCOPE_VERSION_MAJOR
#define IRIDISCOPE_VERSION_MAJOR 0
#endif
#ifndef IRIDISCOPE_VERSION_MINOR
#define IRIDISCOPE_VERSION_MINOR 1
#endif
#define IRIDI_STR2(x) #x
#define IRIDI_STR(x) IRIDI_STR2(x)
#define IRIDISCOPE_VERSION_STRING \
    IRIDI_STR(IRIDISCOPE_VERSION_MAJOR) "." IRIDI_STR(IRIDISCOPE_VERSION_MINOR)

namespace {

GLFWwindow *g_window = nullptr;
GLuint g_map_tex = 0;
int g_map_w = 0, g_map_h = 0;

iridi::IqFileSource g_file;
iridi::LiveIqSource g_sdr(iridi::SdrKind::Sdrplay);
iridi::LiveIqSource g_hackrf(iridi::SdrKind::HackRF);
iridi::LiveIqSource g_airspy(iridi::SdrKind::Airspy);
iridi::LiveIqSource g_rtlsdr(iridi::SdrKind::RTL);
iridi::Engine g_engine;
Waterfall g_waterfall;
iridi::AudioPlayer g_audio;

// Update check.
iridi::VersionCheck g_version_check;
const char *kUpdateUrl = "https://sarahsforge.dev/api/version.php?slug=iridiscope";

// Aircraft info lookups (adsbdb).
iridi::AircraftLookup g_lookup;
bool g_lookup_enabled = true;
std::string g_lookup_reg;
std::string g_lookup_flight;
bool g_show_aircraft_info = false;

// Selected ACARS message detail (ARINC-622).
bool g_show_acars_detail = false;
iridi::AcarsMessage g_acars_detail;

// Voice auto-play: newly completed calls are queued and played in order.
bool g_autoplay = false;
int g_autoplay_last_id = -1;
std::deque<int> g_autoplay_queue;

// Active input: 0 = IQ file, 1 = SDRplay, 2 = HackRF, 3 = Airspy, 4 = RTL-SDR.
int g_source_kind = 0;
iridi::IqSource *active_source() {
    if (g_source_kind == 1) return (iridi::IqSource *)&g_sdr;
    if (g_source_kind == 2) return (iridi::IqSource *)&g_hackrf;
    if (g_source_kind == 3) return (iridi::IqSource *)&g_airspy;
    if (g_source_kind == 4) return (iridi::IqSource *)&g_rtlsdr;
    return (iridi::IqSource *)&g_file;
}

// Default VFO: center of the Iridium band. The VFO can be moved 1616-1626.5 MHz.
const double kDefaultVfoHz = 1621250000.0;
double g_vfo_hz = kDefaultVfoHz;

// SDRplay sample rates.
const double kSdrplayRates[] = {2.0e6, 3.0e6, 4.0e6, 5.0e6, 6.0e6, 7.0e6, 8.0e6, 9.0e6, 10.0e6};
const char *kSdrplayRateLabels[] = {"2.0", "3.0", "4.0", "5.0", "6.0", "7.0", "8.0", "9.0", "10.0"};
const int kNumSdrplayRates = (int)(sizeof(kSdrplayRates) / sizeof(kSdrplayRates[0]));
int g_sdr_rate_idx = kNumSdrplayRates - 1; // 10 MHz
int g_sdr_device_idx = 0;
bool g_sdr_agc = true;
float g_sdr_gr = 40.0f;
bool g_sdr_bias = false;
std::vector<iridi::SdrDeviceInfo> g_sdr_devices;
std::mutex g_sdr_dev_mtx;
std::atomic<bool> g_sdr_scanning{false};
bool g_sdr_scan_requested = false;
std::thread g_sdr_scan_thread;

// HackRF sample rates (2..20 MSPS).
const double kHackrfRates[] = {2.0e6, 4.0e6, 8.0e6, 10.0e6, 20.0e6};
const char *kHackrfRateLabels[] = {"2.0", "4.0", "8.0", "10.0", "20.0"};
const int kNumHackrfRates = (int)(sizeof(kHackrfRates) / sizeof(kHackrfRates[0]));
int g_hrf_rate_idx = 3; // 10 MHz
int g_hrf_device_idx = 0;
int g_hrf_lna = 16; // 0..40, 8 dB steps
int g_hrf_vga = 16; // 0..62, 2 dB steps
bool g_hrf_amp = false;
bool g_hrf_bias = false;
std::vector<iridi::SdrDeviceInfo> g_hrf_devices;
std::mutex g_hrf_dev_mtx;
std::atomic<bool> g_hrf_scanning{false};
bool g_hrf_scan_requested = false;
std::thread g_hrf_scan_thread;

// Airspy sample rates.
const double kAirspyRates[] = {2.5e6, 3.0e6, 6.0e6, 10.0e6};
const char *kAirspyRateLabels[] = {"2.5", "3.0", "6.0", "10.0"};
const int kNumAirspyRates = (int)(sizeof(kAirspyRates) / sizeof(kAirspyRates[0]));
int g_ap_rate_idx = 3; // 10 MHz
int g_ap_device_idx = 0;
int g_ap_gain_mode = 0; // 0=Sensitivity, 1=Linear, 2=Free
int g_ap_sense = 10;    // 0..21
int g_ap_linear = 10;   // 0..21
int g_ap_lna = 8;       // 0..15 (free)
int g_ap_mixer = 8;     // 0..15 (free)
int g_ap_vga = 4;       // 0..15 (free)
bool g_ap_lna_agc = false;
bool g_ap_mixer_agc = false;
bool g_ap_bias = false;
std::vector<iridi::SdrDeviceInfo> g_ap_devices;
std::mutex g_ap_dev_mtx;
std::atomic<bool> g_ap_scanning{false};
bool g_ap_scan_requested = false;
std::thread g_ap_scan_thread;

// RTL-SDR sample rates (the RTL2832 tops out at 3.2 MSPS).
const double kRtlRates[] = {2.4e6, 2.56e6, 3.2e6};
const char *kRtlRateLabels[] = {"2.4", "2.56", "3.2"};
const int kNumRtlRates = (int)(sizeof(kRtlRates) / sizeof(kRtlRates[0]));
int g_rtl_rate_idx = 0; // 2.4 MHz (most stable)
int g_rtl_device_idx = 0;
bool g_rtl_agc = false;
float g_rtl_gain = 28.0f;
int g_rtl_ppm = 0;
bool g_rtl_bias = false;
std::vector<iridi::SdrDeviceInfo> g_rtl_devices;
std::mutex g_rtl_dev_mtx;
std::atomic<bool> g_rtl_scanning{false};
bool g_rtl_scan_requested = false;
std::thread g_rtl_scan_thread;

std::vector<std::string> g_log;
bool g_show_map = true;
bool g_show_source = true;
bool g_show_spectrum = true;
bool g_show_waterfall = true;
bool g_show_stats = true;
bool g_show_bursts = false;
bool g_show_messages = true;
bool g_show_sats = true;
bool g_show_voice = true;
bool g_show_acars = true;
bool g_show_pager = true;
bool g_show_sms = true;
bool g_show_sbd = true;
bool g_show_signaling = true;
bool g_show_broadcast = true;
bool g_show_itl = true;
bool g_show_paging = true;
bool g_show_about = false;
int g_selected_voice = -1;
int g_voice_cache_id = -1;
std::vector<int16_t> g_voice_cache;
bool g_request_open = false;
bool g_auto_start = true;

// Spectrum configuration.
const int kFftSizes[] = {1024, 2048, 4096, 8192, 16384, 32768, 65536};
const char *kFftLabels[] = {"1024", "2048", "4096", "8192", "16384", "32768", "65536"};
const int kNumFftSizes = (int)(sizeof(kFftSizes) / sizeof(kFftSizes[0]));
int g_fft_idx = 2;              // default 4096
float g_avg_alpha = 0.6f;
float g_db_min = -80.0f;
float g_db_max = 0.0f;
bool g_autoscale = true;
bool g_dc_block = true;

uint64_t g_spec_seq = 0;
std::vector<float> g_freq_mhz;
double g_axis_center = 0;
int g_axis_rate = 0;

// Shared FFT/waterfall view in MHz. Both panes always show this exact range.
double g_view_lo_mhz = 0.0;
double g_view_hi_mhz = 0.0;
bool g_view_reset = true;
bool g_view_dirty = false;

void add_log(const char *fmt, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    g_log.emplace_back(buf);
    fprintf(stdout, "%s\n", buf);
    fflush(stdout);
    if (g_log.size() > 2000) g_log.erase(g_log.begin(), g_log.begin() + 500);
}

void open_url(const char *url) {
#ifdef _WIN32
    ShellExecuteA(nullptr, "open", url, nullptr, nullptr, SW_SHOWNORMAL);
#else
    (void)url;
#endif
}

// ---- map texture loading ---------------------------------------------------
static std::wstring exe_directory() {
    std::vector<wchar_t> buf(32768);
    DWORD n = GetModuleFileNameW(nullptr, buf.data(), (DWORD)buf.size());
    if (n == 0 || n >= buf.size()) return L".";
    std::wstring p(buf.data(), n);
    size_t pos = p.find_last_of(L"\\/");
    return pos == std::wstring::npos ? L"." : p.substr(0, pos);
}

static bool read_file_wide(const std::wstring &path, std::vector<unsigned char> &out) {
    HANDLE h = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr,
                           OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h == INVALID_HANDLE_VALUE) return false;
    LARGE_INTEGER size;
    bool ok = GetFileSizeEx(h, &size) != 0 && size.QuadPart > 0 &&
              size.QuadPart <= 256LL * 1024 * 1024;
    if (ok) {
        out.resize((size_t)size.QuadPart);
        DWORD read = 0;
        ok = ReadFile(h, out.data(), (DWORD)out.size(), &read, nullptr) != 0 &&
             read == out.size();
    }
    CloseHandle(h);
    if (!ok) out.clear();
    return ok;
}

static bool read_resource_bytes(int id, std::vector<unsigned char> &out) {
    HMODULE mod = GetModuleHandleW(nullptr);
    HRSRC res = FindResourceW(mod, MAKEINTRESOURCEW(id), MAKEINTRESOURCEW(10));
    if (!res) return false;
    DWORD size = SizeofResource(mod, res);
    if (size == 0) return false;
    HGLOBAL handle = LoadResource(mod, res);
    if (!handle) return false;
    void *data = LockResource(handle);
    if (!data) return false;
    out.assign(static_cast<unsigned char *>(data),
               static_cast<unsigned char *>(data) + size);
    return true;
}

static void resize_rgba_box(const unsigned char *src, int sw, int sh,
                            unsigned char *dst, int dw, int dh) {
    for (int y = 0; y < dh; y++) {
        int sy0 = (int)((int64_t)y * sh / dh);
        int sy1 = (int)((int64_t)(y + 1) * sh / dh);
        if (sy1 <= sy0) sy1 = sy0 + 1;
        if (sy1 > sh) sy1 = sh;
        for (int x = 0; x < dw; x++) {
            int sx0 = (int)((int64_t)x * sw / dw);
            int sx1 = (int)((int64_t)(x + 1) * sw / dw);
            if (sx1 <= sx0) sx1 = sx0 + 1;
            if (sx1 > sw) sx1 = sw;
            uint64_t r = 0, g = 0, b = 0, a = 0, n = 0;
            for (int sy = sy0; sy < sy1; sy++) {
                const unsigned char *row = src + ((size_t)sy * sw + sx0) * 4;
                for (int sx = sx0; sx < sx1; sx++, row += 4) {
                    r += row[0]; g += row[1]; b += row[2]; a += row[3]; n++;
                }
            }
            if (!n) n = 1;
            unsigned char *o = dst + ((size_t)y * dw + x) * 4;
            o[0] = (unsigned char)(r / n);
            o[1] = (unsigned char)(g / n);
            o[2] = (unsigned char)(b / n);
            o[3] = (unsigned char)(a / n);
        }
    }
}

static GLuint upload_map_texture(const unsigned char *pixels, int w, int h,
                                 const char *source, int &out_w, int &out_h) {
    int max_tex = 0;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max_tex);
    if (max_tex <= 0) max_tex = 2048;
    // Cap at 4096 even on very large GPUs: the extra detail is not visible in
    // the map window and this keeps the texture around 32 MB.
    int limit = std::min(max_tex, 4096);
    int uw = w, uh = h;
    std::vector<unsigned char> scaled;
    const unsigned char *upload = pixels;
    if (w > limit || h > limit) {
        double scale = std::min((double)limit / w, (double)limit / h);
        uw = std::max(1, (int)(w * scale));
        uh = std::max(1, (int)(h * scale));
        scaled.resize((size_t)uw * uh * 4);
        resize_rgba_box(pixels, w, h, scaled.data(), uw, uh);
        upload = scaled.data();
    }

    while (glGetError() != GL_NO_ERROR) {}
    GLuint tex = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, uw, uh, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, upload);
    GLenum err = glGetError();
    glBindTexture(GL_TEXTURE_2D, 0);
    if (err != GL_NO_ERROR) {
        if (tex) glDeleteTextures(1, &tex);
        add_log("Map texture upload failed: %s (%dx%d -> %dx%d, GL max %d, err 0x%x)",
                source, w, h, uw, uh, max_tex, err);
        return 0;
    }
    out_w = uw;
    out_h = uh;
    add_log("Loaded map texture %dx%d from %s (source %dx%d, GL max %d)",
            uw, uh, source, w, h, max_tex);
    return tex;
}

static GLuint load_texture_from_bytes(const unsigned char *data, int len,
                                      const char *source, int &out_w, int &out_h) {
    if (!data || len <= 0) return 0;
    int w = 0, h = 0, comp = 0;
    unsigned char *pixels = stbi_load_from_memory(data, len, &w, &h, &comp, 4);
    if (!pixels) {
        add_log("Failed to decode map image: %s", source);
        return 0;
    }
    GLuint tex = upload_map_texture(pixels, w, h, source, out_w, out_h);
    stbi_image_free(pixels);
    return tex;
}

static GLuint load_map_texture(int &out_w, int &out_h) {
    // Prefer a user-provided copy next to the executable, then use the copy
    // embedded in the executable.
    std::wstring dir = exe_directory();
    std::wstring candidates[] = {
        dir + L"\\assets\\worldmap.jpg",
        dir + L"\\worldmap.jpg",
    };
    for (const auto &p : candidates) {
        std::vector<unsigned char> bytes;
        if (!read_file_wide(p, bytes)) continue;
        GLuint tex = load_texture_from_bytes(bytes.data(), (int)bytes.size(),
                                             "external worldmap.jpg", out_w, out_h);
        if (tex) return tex;
    }
    std::vector<unsigned char> bytes;
    if (read_resource_bytes(100, bytes)) {
        GLuint tex = load_texture_from_bytes(bytes.data(), (int)bytes.size(),
                                             "embedded worldmap.jpg", out_w, out_h);
        if (tex) return tex;
    }
    add_log("Map image unavailable: no external assets/worldmap.jpg and no embedded map");
    return 0;
}

std::string open_file_dialog() {
    char filename[MAX_PATH] = {0};
    OPENFILENAMEA ofn;
    ZeroMemory(&ofn, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter =
        "IQ files (*.wav;*.cf32;*.ci16;*.ci8;*.cfile;*.raw)\0"
        "*.wav;*.cf32;*.ci16;*.ci8;*.cfile;*.raw\0"
        "All files (*.*)\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
    if (GetOpenFileNameA(&ofn)) return std::string(filename);
    return std::string();
}

void apply_spectrum_settings() {
    g_engine.set_fft_size(kFftSizes[g_fft_idx]);
    g_engine.set_avg_alpha(g_avg_alpha);
    g_engine.set_dc_block(g_dc_block);
}

void start_engine() {
    iridi::IqSource *src = active_source();
    if (!src->is_open()) { add_log("ERROR: no source open"); return; }
    g_autoplay_last_id = -1;
    g_autoplay_queue.clear();
    if (g_engine.start(src)) { apply_spectrum_settings(); add_log("Engine started"); }
    else add_log("ERROR: failed to start engine");
}

const double kBandLoMhz = 1616.0;
const double kBandHiMhz = 1626.5;

// Retune a live SDR without changing the current FFT/waterfall view.
void retune_vfo(double hz) {
    hz = std::clamp(hz, kBandLoMhz * 1e6, kBandHiMhz * 1e6);
    if (std::fabs(hz - g_vfo_hz) < 1000.0) return;
    g_vfo_hz = hz;
    if (g_source_kind == 1) { g_sdr.set_center_frequency(hz); g_sdr.clear_buffer(); }
    else if (g_source_kind == 2) { g_hackrf.set_center_frequency(hz); g_hackrf.clear_buffer(); }
    else if (g_source_kind == 3) { g_airspy.set_center_frequency(hz); g_airspy.clear_buffer(); }
    else if (g_source_kind == 4) { g_rtlsdr.set_center_frequency(hz); g_rtlsdr.clear_buffer(); }
    g_engine.set_center_frequency(hz);
    g_waterfall.clear();
    g_spec_seq = 0;
    g_freq_mhz.clear();
}

// Keep the shared FFT/waterfall view inside the Iridium band and inside the
// SDR's captured passband. If the view leaves the passband, retune the radio
// so both windows continue to show real data.
void sync_view(double fs_mhz, bool live) {
    double &lo = g_view_lo_mhz, &hi = g_view_hi_mhz;
    if (hi <= lo) {
        double fc = active_source()->center_frequency() / 1e6;
        lo = std::max(kBandLoMhz, fc - fs_mhz * 0.5);
        hi = std::min(kBandHiMhz, fc + fs_mhz * 0.5);
        if (hi <= lo) { lo = kBandLoMhz; hi = kBandHiMhz; }
    }
    double max_span = std::min(kBandHiMhz - kBandLoMhz,
                               fs_mhz > 0 ? fs_mhz : kBandHiMhz - kBandLoMhz);
    double span = hi - lo;
    if (span > max_span) {
        double c = std::clamp((lo + hi) * 0.5, kBandLoMhz + max_span * 0.5,
                              kBandHiMhz - max_span * 0.5);
        lo = c - max_span * 0.5;
        hi = c + max_span * 0.5;
        span = max_span;
    }
    if (span < 0.05) {
        double c = std::clamp((lo + hi) * 0.5, kBandLoMhz + 0.025,
                              kBandHiMhz - 0.025);
        lo = c - 0.025;
        hi = c + 0.025;
    }
    if (lo < kBandLoMhz) { hi += kBandLoMhz - lo; lo = kBandLoMhz; }
    if (hi > kBandHiMhz) { lo -= hi - kBandHiMhz; hi = kBandHiMhz; }
    lo = std::max(lo, kBandLoMhz);
    hi = std::min(hi, kBandHiMhz);

    if (fs_mhz <= 0) return;

    double fc = active_source()->center_frequency() / 1e6;
    double cap_lo = fc - fs_mhz * 0.5, cap_hi = fc + fs_mhz * 0.5;

    if (live && (lo < cap_lo || hi > cap_hi)) {
        double desired = (lo + hi) * 0.5;
        if (fs_mhz <= kBandHiMhz - kBandLoMhz) {
            desired = std::clamp(desired, kBandLoMhz + fs_mhz * 0.5,
                                 kBandHiMhz - fs_mhz * 0.5);
        } else {
            desired = (kBandLoMhz + kBandHiMhz) * 0.5;
        }
        retune_vfo(desired * 1e6);
        fc = active_source()->center_frequency() / 1e6;
        cap_lo = fc - fs_mhz * 0.5;
        cap_hi = fc + fs_mhz * 0.5;
    }

    // Keep the view inside the captured passband as well. Otherwise the FFT
    // and the waterfall would be drawn with different frequency ranges.
    double nlo = std::max(lo, cap_lo);
    double nhi = std::min(hi, cap_hi);
    if (nhi <= nlo) {
        nlo = std::max(cap_lo, kBandLoMhz);
        nhi = std::min(cap_hi, kBandHiMhz);
        if (nhi <= nlo) { nlo = cap_lo; nhi = cap_hi; }
    }
    lo = std::max(nlo, kBandLoMhz);
    hi = std::min(nhi, kBandHiMhz);
}

// Initialise the shared view after a source or sample-rate change.
double ensure_view() {
    double fs = (active_source()->sample_rate() ? active_source()->sample_rate() : 0) / 1e6;
    if (g_view_reset || g_view_hi_mhz <= g_view_lo_mhz) {
        double fc = active_source()->center_frequency() / 1e6;
        g_view_lo_mhz = std::max(kBandLoMhz, fc - fs * 0.5);
        g_view_hi_mhz = std::min(kBandHiMhz, fc + fs * 0.5);
        if (g_view_hi_mhz <= g_view_lo_mhz) {
            g_view_lo_mhz = kBandLoMhz;
            g_view_hi_mhz = kBandHiMhz;
        }
        g_view_reset = false;
        g_view_dirty = true;
        sync_view(fs, g_source_kind != 0);
    }
    return fs;
}

// Change a live SDR's sample rate. The DSP pipeline has to be rebuilt, so the
// engine is restarted around the change.
void apply_sample_rate(int kind, double hz) {
    bool was_running = g_engine.running();
    if (was_running) g_engine.stop();
    if (kind == 1) { g_sdr.set_sample_rate(hz); g_sdr.clear_buffer(); }
    else if (kind == 2) { g_hackrf.set_sample_rate(hz); g_hackrf.clear_buffer(); }
    else if (kind == 3) { g_airspy.set_sample_rate(hz); g_airspy.clear_buffer(); }
    else if (kind == 4) { g_rtlsdr.set_sample_rate(hz); g_rtlsdr.clear_buffer(); }
    g_waterfall.clear();
    g_spec_seq = 0;
    g_freq_mhz.clear();
    g_view_reset = true;
    add_log("Sample rate -> %.2f Msps", hz / 1e6);
    if (was_running) start_engine();
}

void open_iq_file(const std::string &path) {
    if (path.empty()) return;
    g_engine.stop();
    g_sdr.close();
    g_hackrf.close();
    g_airspy.close();
    g_rtlsdr.close();
    g_source_kind = 0;
    std::string err;
    if (!g_file.open(path, err)) { add_log("ERROR: %s", err.c_str()); return; }
    const auto &in = g_file.info();
    add_log("Opened %s", in.path.c_str());
    add_log("  format=%s rate=%d Hz samples=%llu duration=%.3f s center=%.0f Hz",
            iridi::iq_format_name(in.format), in.sample_rate,
            (unsigned long long)in.num_samples, in.duration_s, in.center_frequency);

    g_waterfall.clear();
    g_spec_seq = 0;
    g_freq_mhz.clear();
    g_view_reset = true;

    if (g_auto_start) start_engine();
}

void start_sdr() {
    g_engine.stop();
    g_file.close();
    g_hackrf.close();
    g_airspy.close();
    g_rtlsdr.close();
    g_source_kind = 1;
    g_sdr.set_gain(g_sdr_agc ? -1.0 : (double)g_sdr_gr);
    g_sdr.set_bias_tee(g_sdr_bias);
    std::string err;
    if (!g_sdr.open(g_sdr_device_idx, g_vfo_hz, kSdrplayRates[g_sdr_rate_idx], err)) {
        add_log("ERROR: %s", err.c_str());
        return;
    }
    add_log("SDRplay started: %s @ %.6f MHz, %.1f Msps", g_sdr.device_name().c_str(),
            g_vfo_hz / 1e6, kSdrplayRates[g_sdr_rate_idx] / 1e6);
    g_waterfall.clear();
    g_spec_seq = 0;
    g_freq_mhz.clear();
    g_view_reset = true;
    start_engine();
}

void start_hackrf() {
    g_engine.stop();
    g_file.close();
    g_sdr.close();
    g_airspy.close();
    g_rtlsdr.close();
    g_source_kind = 2;
    g_hackrf.set_lna_gain(g_hrf_lna);
    g_hackrf.set_vga_gain(g_hrf_vga);
    g_hackrf.set_amp(g_hrf_amp);
    g_hackrf.set_bias_tee(g_hrf_bias);
    std::string err;
    if (!g_hackrf.open(g_hrf_device_idx, g_vfo_hz, kHackrfRates[g_hrf_rate_idx], err)) {
        add_log("ERROR: %s", err.c_str());
        return;
    }
    add_log("HackRF started: %s @ %.6f MHz, %.1f Msps (LNA %d, VGA %d, amp %s)",
            g_hackrf.device_name().c_str(), g_vfo_hz / 1e6,
            kHackrfRates[g_hrf_rate_idx] / 1e6, g_hrf_lna, g_hrf_vga, g_hrf_amp ? "on" : "off");
    g_waterfall.clear();
    g_spec_seq = 0;
    g_freq_mhz.clear();
    g_view_reset = true;
    start_engine();
}

void start_airspy() {
    g_engine.stop();
    g_file.close();
    g_sdr.close();
    g_hackrf.close();
    g_rtlsdr.close();
    g_source_kind = 3;
    g_airspy.set_gain_mode(g_ap_gain_mode);
    g_airspy.set_sense_gain(g_ap_sense);
    g_airspy.set_linear_gain(g_ap_linear);
    g_airspy.set_lna_gain(g_ap_lna);
    g_airspy.set_mixer_gain(g_ap_mixer);
    g_airspy.set_vga_gain(g_ap_vga);
    g_airspy.set_lna_agc(g_ap_lna_agc);
    g_airspy.set_mixer_agc(g_ap_mixer_agc);
    g_airspy.set_bias_tee(g_ap_bias);
    std::string err;
    if (!g_airspy.open(g_ap_device_idx, g_vfo_hz, kAirspyRates[g_ap_rate_idx], err)) {
        add_log("ERROR: %s", err.c_str());
        return;
    }
    add_log("Airspy started: %s @ %.6f MHz, %.1f Msps", g_airspy.device_name().c_str(),
            g_vfo_hz / 1e6, kAirspyRates[g_ap_rate_idx] / 1e6);
    g_waterfall.clear();
    g_spec_seq = 0;
    g_freq_mhz.clear();
    g_view_reset = true;
    start_engine();
}

void start_rtl() {
    g_engine.stop();
    g_file.close();
    g_sdr.close();
    g_hackrf.close();
    g_airspy.close();
    g_source_kind = 4;
    g_rtlsdr.set_gain(g_rtl_agc ? -1.0 : (double)g_rtl_gain);
    g_rtlsdr.set_ppm(g_rtl_ppm);
    g_rtlsdr.set_bias_tee(g_rtl_bias);
    std::string err;
    if (!g_rtlsdr.open(g_rtl_device_idx, g_vfo_hz, kRtlRates[g_rtl_rate_idx], err)) {
        add_log("ERROR: %s", err.c_str());
        return;
    }
    add_log("RTL-SDR started: %s @ %.6f MHz, %.2f Msps", g_rtlsdr.device_name().c_str(),
            g_vfo_hz / 1e6, kRtlRates[g_rtl_rate_idx] / 1e6);
    g_waterfall.clear();
    g_spec_seq = 0;
    g_freq_mhz.clear();
    g_view_reset = true;
    start_engine();
}

// ---- panels ---------------------------------------------------------------

void draw_menu_bar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Open IQ file...", "Ctrl+O")) g_request_open = true;
            if (ImGui::MenuItem("Close")) { g_engine.stop(); g_file.close(); g_sdr.close(); g_hackrf.close(); g_airspy.close(); g_rtlsdr.close(); g_waterfall.clear(); add_log("Closed source"); }
            ImGui::Separator();
            if (ImGui::MenuItem("Quit", "Alt+F4")) glfwSetWindowShouldClose(g_window, 1);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Playback")) {
            bool running = g_engine.running();
            if (ImGui::MenuItem("Start", nullptr, false, active_source()->is_open() && !running)) {
                start_engine();
            }
            if (ImGui::MenuItem(g_engine.paused() ? "Resume" : "Pause", nullptr, false, running))
                g_engine.set_paused(!g_engine.paused());
            if (ImGui::MenuItem("Stop", nullptr, false, running)) g_engine.stop();
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Map", nullptr, &g_show_map);
            ImGui::MenuItem("Spectrum", nullptr, &g_show_spectrum);
            ImGui::MenuItem("Waterfall", nullptr, &g_show_waterfall);
            ImGui::MenuItem("Stats", nullptr, &g_show_stats);
            ImGui::MenuItem("Bursts", nullptr, &g_show_bursts);
            ImGui::MenuItem("Messages", nullptr, &g_show_messages);
            ImGui::MenuItem("Satellites", nullptr, &g_show_sats);
            ImGui::MenuItem("Paging", nullptr, &g_show_paging);
            ImGui::MenuItem("Voice", nullptr, &g_show_voice);
            ImGui::MenuItem("ACARS", nullptr, &g_show_acars);
            ImGui::MenuItem("Pager", nullptr, &g_show_pager);
            ImGui::MenuItem("SMS", nullptr, &g_show_sms);
            ImGui::MenuItem("SBD", nullptr, &g_show_sbd);
            ImGui::MenuItem("Signaling", nullptr, &g_show_signaling);
            ImGui::MenuItem("Broadcast", nullptr, &g_show_broadcast);
            ImGui::MenuItem("ITL", nullptr, &g_show_itl);
            ImGui::MenuItem("Source", nullptr, &g_show_source);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Help")) {
            switch (g_version_check.state()) {
            case iridi::VersionCheck::UpdateAvailable: {
                std::string v = g_version_check.latest_version();
                std::string label = "Update available: v" + v + "  (click to download)";
                if (ImGui::MenuItem(label.c_str()))
                    open_url(g_version_check.product_url().c_str());
                break;
            }
            case iridi::VersionCheck::Checking:
                ImGui::MenuItem("Checking for updates...", nullptr, false, false);
                break;
            case iridi::VersionCheck::Error:
                ImGui::MenuItem("Update check failed", nullptr, false, false);
                break;
            default:
                ImGui::MenuItem("Up to date", nullptr, false, false);
                break;
            }
            ImGui::Separator();
            if (ImGui::MenuItem("About IridiScope")) g_show_about = true;
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }
}

void project_to_map(double lat, double lon, const ImVec2 &pos, const ImVec2 &size, ImVec2 &out) {
    out.x = pos.x + (float)((lon + 180.0) / 360.0 * size.x);
    out.y = pos.y + (float)((90.0 - lat) / 180.0 * size.y);
}

void draw_map_window() {
    if (!g_show_map) return;
    ImGui::SetNextWindowSize(ImVec2(900, 500), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Map", &g_show_map)) {
        ImVec2 avail = ImGui::GetContentRegionAvail();
        if (avail.x < 16 || avail.y < 16) { ImGui::End(); return; }
        float aspect = (g_map_h > 0) ? (float)g_map_w / (float)g_map_h : 2.0f;
        ImVec2 size = avail;
        if (size.x / size.y > aspect) size.x = size.y * aspect; else size.y = size.x / aspect;
        ImVec2 pos = ImGui::GetCursorScreenPos();
        if (g_map_tex) ImGui::Image((ImTextureID)(uintptr_t)g_map_tex, size);
        else { ImGui::Dummy(size); ImGui::GetWindowDrawList()->AddRectFilled(pos,
                 ImVec2(pos.x + size.x, pos.y + size.y), IM_COL32(20, 30, 40, 255)); }
        ImDrawList *dl = ImGui::GetWindowDrawList();
        ImU32 grid = IM_COL32(255, 255, 255, 40);
        ImVec2 a;
        for (int lon = -180; lon <= 180; lon += 30) { project_to_map(0, lon, pos, size, a);
            dl->AddLine(ImVec2(a.x, pos.y), ImVec2(a.x, pos.y + size.y), grid); }
        for (int lat = -90; lat <= 90; lat += 30) { project_to_map(lat, 0, pos, size, a);
            dl->AddLine(ImVec2(pos.x, a.y), ImVec2(pos.x + size.x, a.y), grid); }
        dl->AddRect(pos, ImVec2(pos.x + size.x, pos.y + size.y), IM_COL32(255, 255, 255, 90));

        // Satellites (from decoded ring alerts / broadcasts).
        for (const auto &s : g_engine.satellites()) {
            if (!s.has_pos || s.lat < -90.0 || s.lat > 90.0) continue;
            ImVec2 pt;
            project_to_map(s.lat, s.lon, pos, size, pt);
            dl->AddCircleFilled(pt, 4.0f, IM_COL32(255, 170, 0, 230));
            dl->AddCircle(pt, 7.0f, IM_COL32(255, 255, 0, 140), 0, 1.5f);
            char lbl[16];
            snprintf(lbl, sizeof(lbl), "S%03d", s.id);
            dl->AddText(ImVec2(pt.x + 7, pt.y - 7), IM_COL32(255, 255, 140, 255), lbl);
        }
        // Ground spot-beam centers (ring alerts with alt < 100 km).
        for (const auto &g : g_engine.ground_beams()) {
            if (g.lat < -90.0 || g.lat > 90.0) continue;
            ImVec2 pt;
            project_to_map(g.lat, g.lon, pos, size, pt);
            dl->AddCircleFilled(pt, 2.5f, IM_COL32(0, 210, 255, 190));
            if (g.pages > 0)
                dl->AddCircle(pt, 6.5f, IM_COL32(255, 60, 220, 230), 0, 1.5f);
        }
        if (!g_map_tex) {
            const char *msg = "Map image unavailable";
            ImVec2 ts = ImGui::CalcTextSize(msg);
            dl->AddText(ImVec2(pos.x + (size.x - ts.x) * 0.5f,
                               pos.y + (size.y - ts.y) * 0.5f),
                        IM_COL32(235, 235, 235, 240), msg);
            const char *hint =
                "Place worldmap.jpg in an assets folder next to IridiScope.exe";
            ImVec2 hs = ImGui::CalcTextSize(hint);
            dl->AddText(ImVec2(pos.x + (size.x - hs.x) * 0.5f,
                               pos.y + (size.y - hs.y) * 0.5f + 22),
                        IM_COL32(160, 185, 205, 220), hint);
        }
        dl->AddText(ImVec2(pos.x + 8, pos.y + size.y - 20),
                    IM_COL32(255, 255, 255, 190),
                    "orange = satellite   cyan = spot beam   magenta = paging");
    }
    ImGui::End();
}

void draw_satellites_window() {
    if (!g_show_sats) return;
    ImGui::SetNextWindowSize(ImVec2(420, 240), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Satellites", &g_show_sats)) {
        auto sats = g_engine.satellites();
        ImGui::TextDisabled("%zu satellites", sats.size());
        if (ImGui::BeginTable("sats", 6,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                ImVec2(0, 0))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Sat");
            ImGui::TableSetupColumn("Lat");
            ImGui::TableSetupColumn("Lon");
            ImGui::TableSetupColumn("Alt km");
            ImGui::TableSetupColumn("Beam");
            ImGui::TableSetupColumn("Hits");
            ImGui::TableHeadersRow();
            for (const auto &s : sats) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::Text("%03d", s.id);
                ImGui::TableSetColumnIndex(1);
                if (s.has_pos) ImGui::Text("%+.2f", s.lat); else ImGui::TextUnformatted("-");
                ImGui::TableSetColumnIndex(2);
                if (s.has_pos) ImGui::Text("%+.2f", s.lon); else ImGui::TextUnformatted("-");
                ImGui::TableSetColumnIndex(3);
                if (s.has_pos) ImGui::Text("%d", s.alt_km); else ImGui::TextUnformatted("-");
                ImGui::TableSetColumnIndex(4); ImGui::Text("%d", s.beam);
                ImGui::TableSetColumnIndex(5); ImGui::Text("%d", s.count);
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void draw_messages_window() {
    if (!g_show_messages) return;
    ImGui::SetNextWindowSize(ImVec2(820, 280), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Messages", &g_show_messages)) {
        auto msgs = g_engine.recent_messages(1000);
        ImGui::TextDisabled("%zu recent parsed messages", msgs.size());
        if (ImGui::BeginTable("msgs", 4,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                ImVec2(0, 0))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 52);
            ImGui::TableSetupColumn("Freq (Hz)", ImGuiTableColumnFlags_WidthFixed, 96);
            ImGui::TableSetupColumn("Sat", ImGuiTableColumnFlags_WidthFixed, 40);
            ImGui::TableSetupColumn("Detail", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();
            for (auto it = msgs.rbegin(); it != msgs.rend(); ++it) {
                const auto &m = *it;
                ImGui::TableNextRow();
                ImVec4 col = m.error ? ImVec4(0.6f, 0.6f, 0.6f, 1.0f)
                    : (m.type == "IRA" ? ImVec4(1.0f, 0.8f, 0.3f, 1.0f)
                    : (m.has_ida ? ImVec4(0.5f, 0.9f, 0.6f, 1.0f)
                    : ImVec4(0.85f, 0.85f, 0.95f, 1.0f)));
                ImGui::TableSetColumnIndex(0); ImGui::TextColored(col, "%s", m.type.c_str());
                ImGui::TableSetColumnIndex(1); ImGui::Text("%.0f", m.frequency);
                ImGui::TableSetColumnIndex(2);
                if (m.has_ira) ImGui::Text("%03d", m.ira_sat);
                else if (m.has_ibc) ImGui::Text("%03d", m.ibc_sat);
                else ImGui::TextUnformatted("-");
                ImGui::TableSetColumnIndex(3); ImGui::TextUnformatted(m.pretty.c_str());
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void draw_playback_window() {
    ImGui::SetNextWindowSize(ImVec2(430, 0), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Playback")) {
        bool running = g_engine.running();
        if (!running) {
            ImGui::BeginDisabled(!active_source()->is_open());
            if (ImGui::Button("Start")) start_engine();
            ImGui::EndDisabled();
        } else {
            if (ImGui::Button(g_engine.paused() ? "Resume" : "Pause")) g_engine.set_paused(!g_engine.paused());
            ImGui::SameLine();
            if (ImGui::Button("Stop")) g_engine.stop();
        }
        ImGui::SameLine();
        bool rt = g_engine.realtime();
        if (ImGui::Checkbox("1x realtime", &rt)) g_engine.set_realtime(rt);

        iridi::EngineStats s = g_engine.stats();
        char overlay[64];
        if (s.total_samples) {
            float frac = (float)((double)s.samples / (double)s.total_samples);
            snprintf(overlay, sizeof(overlay), "%.1f%% (%.1f s)", frac * 100.0f, s.elapsed_s);
            ImGui::ProgressBar(frac, ImVec2(-1, 0), overlay);
        } else {
            snprintf(overlay, sizeof(overlay), "live (%.1f s)", s.elapsed_s);
            ImGui::ProgressBar(0.0f, ImVec2(-1, 0), overlay);
        }
        if (s.done) ImGui::TextColored(ImVec4(0.4f, 1, 0.4f, 1), "Done");
    }
    ImGui::End();
}

void draw_stats_window() {
    if (!g_show_stats) return;
    ImGui::SetNextWindowSize(ImVec2(360, 220), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Stats", &g_show_stats)) {
        iridi::EngineStats s = g_engine.stats();
        if (ImGui::BeginTable("stats", 2, ImGuiTableFlags_SizingStretchProp)) {
            auto row = [](const char *k, const char *v) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(k);
                ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(v);
            };
            char b[64];
            snprintf(b, sizeof(b), "%llu", (unsigned long long)s.bursts);      row("Bursts tagged", b);
            snprintf(b, sizeof(b), "%llu", (unsigned long long)s.frames_ok);   row("Frames demod", b);
            snprintf(b, sizeof(b), "%llu", (unsigned long long)s.frames_fail); row("Frames failed", b);
            snprintf(b, sizeof(b), "%llu", (unsigned long long)g_engine.parsed_count()); row("Parsed frames", b);
            snprintf(b, sizeof(b), "%llu", (unsigned long long)g_engine.parse_error_count()); row("Parse errors", b);
            snprintf(b, sizeof(b), "%zu", g_engine.satellites().size());       row("Satellites", b);
            snprintf(b, sizeof(b), "%.1f s", s.elapsed_s);                     row("Elapsed", b);
            snprintf(b, sizeof(b), "%s", s.running ? (s.paused ? "paused" : "running") : "stopped");
            row("State", b);
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void draw_spectrum_controls() {
    ImGui::SetNextItemWidth(110);
    if (ImGui::Combo("FFT", &g_fft_idx, kFftLabels, kNumFftSizes)) {
        g_engine.set_fft_size(kFftSizes[g_fft_idx]);
        g_freq_mhz.clear(); // axis depends on N
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(110);
    if (ImGui::SliderFloat("Avg", &g_avg_alpha, 0.0f, 0.98f, "%.2f"))
        g_engine.set_avg_alpha(g_avg_alpha);
    ImGui::SameLine();
    ImGui::Checkbox("Auto", &g_autoscale);
    ImGui::SameLine();
    if (ImGui::Checkbox("DC block", &g_dc_block)) g_engine.set_dc_block(g_dc_block);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90);
    ImGui::InputFloat("min", &g_db_min, 1.0f, 5.0f, "%.0f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90);
    ImGui::InputFloat("max", &g_db_max, 1.0f, 5.0f, "%.0f");
    if (g_db_max < g_db_min + 5.0f) g_db_max = g_db_min + 5.0f;
}

// ---- Iridium band plan -----------------------------------------------------
static const double kDuplexLoMhz = kBandLoMhz;
static const double kDuplexHiMhz = 1626.0;
static const double kSimplexHiMhz = kBandHiMhz;

struct SimplexMark { double mhz; const char *label; };
static const SimplexMark kSimplexMarks[] = {
    {1626.000, "Guard"},
    {1626.104, "Quaternary"},
    {1626.146, "Tertiary"},
    {1626.271, "RA"},
    {1626.396, "Secondary"},
    {1626.438, "Primary"},
    {1626.500, nullptr},
};

// Draw the band plan as a single strip across the top of the FFT.
static void draw_band_plan_pixels(ImDrawList *dl, ImVec2 tl, ImVec2 br,
                                  double center_mhz, double span_mhz, bool labels) {
    if (span_mhz <= 1e-9 || br.x <= tl.x || br.y <= tl.y) return;
    double lo = center_mhz - span_mhz * 0.5;
    double hi = center_mhz + span_mhz * 0.5;
    auto X = [&](double f) {
        return tl.x + (float)((f - lo) / span_mhz) * (br.x - tl.x);
    };
    const float h = 30.0f;
    ImVec2 s0(tl.x, tl.y), s1(br.x, tl.y + h);
    dl->PushClipRect(tl, br, true);
    dl->AddRectFilled(s0, s1, IM_COL32(12, 16, 24, 235));
    dl->AddLine(ImVec2(s0.x, s1.y), ImVec2(s1.x, s1.y), IM_COL32(255, 255, 255, 60));

    auto seg = [&](double a, double b, ImU32 col) {
        a = std::max(a, lo); b = std::min(b, hi);
        if (b <= a) return;
        dl->AddRectFilled(ImVec2(X(a), s0.y + 1), ImVec2(X(b), s1.y - 1), col);
    };
    seg(kDuplexLoMhz, kDuplexHiMhz, IM_COL32(55, 90, 190, 190));
    seg(kDuplexHiMhz, kSimplexHiMhz, IM_COL32(40, 165, 95, 200));

    for (double f : {kDuplexLoMhz, kDuplexHiMhz, kSimplexHiMhz}) {
        if (f < lo || f > hi) continue;
        float x = X(f);
        dl->AddLine(ImVec2(x, s0.y), ImVec2(x, s1.y), IM_COL32(255, 255, 255, 140));
    }

    if (labels) {
        auto text_seg = [&](double a, double b, const char *full, const char *shrt,
                            ImU32 col) {
            a = std::max(a, lo); b = std::min(b, hi);
            if (b <= a) return;
            float xa = X(a), xb = X(b);
            float wf = ImGui::CalcTextSize(full).x;
            float ws = ImGui::CalcTextSize(shrt).x;
            if (xb - xa >= wf + 8)
                dl->AddText(ImVec2(xa + 4, s0.y + 3), col, full);
            else if (xb - xa >= ws + 8)
                dl->AddText(ImVec2(xa + 4, s0.y + 3), col, shrt);
            else if (xb - xa >= ws * 0.5f)
                dl->AddText(ImVec2(std::max(tl.x + 2, xb - ws - 2), s0.y + 3),
                            col, shrt);
        };
        text_seg(kDuplexLoMhz, kDuplexHiMhz,
                 "DUPLEX  calls / traffic / signaling / broadcast / sync",
                 "DUPLEX", IM_COL32(215, 228, 255, 255));
        text_seg(kDuplexHiMhz, kSimplexHiMhz,
                 "SIMPLEX  SMS / messaging / Ring Alert", "SIMPLEX",
                 IM_COL32(215, 255, 225, 255));

        const int n = (int)(sizeof(kSimplexMarks) / sizeof(kSimplexMarks[0]));
        for (int i = 0; i < n; i++) {
            double f = kSimplexMarks[i].mhz;
            if (f < lo || f > hi || !kSimplexMarks[i].label) continue;
            double next = (i + 1 < n) ? kSimplexMarks[i + 1].mhz : kSimplexHiMhz;
            float gap = X(next) - X(f);
            float w = ImGui::CalcTextSize(kSimplexMarks[i].label).x;
            if (gap > w + 4)
                dl->AddText(ImVec2(X(f) + 2, s0.y + 16),
                            IM_COL32(210, 255, 220, 240), kSimplexMarks[i].label);
        }
    }
    dl->PopClipRect();
}

void draw_spectrum_window() {
    if (!g_show_spectrum) return;
    ImGui::SetNextWindowSize(ImVec2(760, 300), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Spectrum", &g_show_spectrum)) {
        draw_spectrum_controls();
        std::vector<float> avg = g_engine.spectrum_avg();
        int N = (int)avg.size();
        if (N < 2) {
            ImGui::TextDisabled("Waiting for data...");
            ImGui::End();
            return;
        }
        double fs_mhz = ensure_view();
        bool live = g_source_kind != 0;

        if ((int)g_freq_mhz.size() != N ||
            active_source()->center_frequency() != g_axis_center ||
            active_source()->sample_rate() != g_axis_rate) {
            g_axis_center = active_source()->center_frequency();
            g_axis_rate = active_source()->sample_rate();
            g_freq_mhz.resize(N);
            double fc = g_axis_center;
            double fs = g_axis_rate ? g_axis_rate : 1.0;
            for (int i = 0; i < N; i++)
                g_freq_mhz[i] = (float)((fc + ((double)i - N / 2) * fs / N) / 1e6);
        }
        if (ImPlot::BeginPlot("##spectrum", ImVec2(-1, -1),
                              ImPlotFlags_NoLegend | ImPlotFlags_NoBoxSelect)) {
            ImPlot::SetupAxes("MHz", "dBm", ImPlotAxisFlags_None, ImPlotAxisFlags_None);
            ImPlot::SetupAxisLimits(ImAxis_Y1, g_db_min, g_db_max, ImGuiCond_Always);
            if (g_view_dirty) {
                ImPlot::SetupAxisLimits(ImAxis_X1, g_view_lo_mhz, g_view_hi_mhz,
                                        ImGuiCond_Always);
                g_view_dirty = false;
            }
            ImPlot::PlotLine("PSD", g_freq_mhz.data(), avg.data(), N);

            if (ImPlot::IsPlotHovered() &&
                ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                g_view_reset = true;

            ImPlotRect lim = ImPlot::GetPlotLimits();
            double vlo = lim.X.Min, vhi = lim.X.Max;
            g_view_lo_mhz = vlo;
            g_view_hi_mhz = vhi;
            sync_view(fs_mhz, live);
            // If the view had to be clamped (band edge or max span), force the
            // plot axes to the clamped range next frame.
            if (std::fabs(g_view_lo_mhz - vlo) > 1e-7 ||
                std::fabs(g_view_hi_mhz - vhi) > 1e-7)
                g_view_dirty = true;

            ImVec2 tl = ImPlot::PlotToPixels(lim.X.Min, lim.Y.Max);
            ImVec2 br = ImPlot::PlotToPixels(lim.X.Max, lim.Y.Min);
            draw_band_plan_pixels(ImPlot::GetPlotDrawList(), tl, br,
                                  (lim.X.Min + lim.X.Max) * 0.5,
                                  lim.X.Max - lim.X.Min, true);
            ImPlot::EndPlot();
        }
    }
    ImGui::End();
}

void draw_waterfall_window() {
    if (!g_show_waterfall) return;
    ImGui::SetNextWindowSize(ImVec2(760, 320), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Waterfall", &g_show_waterfall)) {
        draw_spectrum_controls();
        double fs_mhz = ensure_view();
        bool live = g_source_kind != 0;
        ImVec2 avail = ImGui::GetContentRegionAvail();

        // Both panes always show the same frequency slice.
        double fc = active_source()->center_frequency() / 1e6;
        double cap_lo = fc - fs_mhz * 0.5, cap_hi = fc + fs_mhz * 0.5;
        float uMin = 0.0f, uMax = 1.0f;
        if (fs_mhz > 0 && cap_hi > cap_lo) {
            uMin = (float)std::clamp((g_view_lo_mhz - cap_lo) / fs_mhz, 0.0, 1.0);
            uMax = (float)std::clamp((g_view_hi_mhz - cap_lo) / fs_mhz, 0.0, 1.0);
        }
        g_waterfall.draw(avail, uMin, uMax);

        bool wf_hovered = ImGui::IsItemHovered();
        bool wf_active = ImGui::IsItemActive();
        if ((wf_hovered || wf_active) && fs_mhz > 0) {
            ImVec2 rmin = ImGui::GetItemRectMin();
            ImVec2 rmax = ImGui::GetItemRectMax();
            float width = rmax.x - rmin.x;
            double span = g_view_hi_mhz - g_view_lo_mhz;
            if (width > 1.0f && span > 0.0) {
                if (wf_active && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                    double shift = -(double)ImGui::GetIO().MouseDelta.x * span / width;
                    g_view_lo_mhz += shift;
                    g_view_hi_mhz += shift;
                    sync_view(fs_mhz, live);
                    g_view_dirty = true;
                }
                float wheel = ImGui::GetIO().MouseWheel;
                if (wf_hovered && wheel != 0.0f) {
                    double anchor = g_view_lo_mhz +
                                    (double)(ImGui::GetIO().MousePos.x - rmin.x) /
                                        width * span;
                    double max_span = std::min(kBandHiMhz - kBandLoMhz, fs_mhz);
                    double factor = std::pow(0.85, (double)wheel);
                    double new_span = std::clamp(span * factor, 0.05, max_span);
                    double frac = (anchor - g_view_lo_mhz) / span;
                    g_view_lo_mhz = anchor - frac * new_span;
                    g_view_hi_mhz = g_view_lo_mhz + new_span;
                    sync_view(fs_mhz, live);
                    g_view_dirty = true;
                }
                if (wf_hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                    g_view_reset = true;
            }
        }
    }
    ImGui::End();
}

void draw_bursts_window() {
    if (!g_show_bursts) return;
    ImGui::SetNextWindowSize(ImVec2(620, 300), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Bursts", &g_show_bursts)) {
        auto frames = g_engine.recent_frames(500);
        ImGui::TextDisabled("%zu recent demodulated frames", frames.size());
        if (ImGui::BeginTable("bursts", 5,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                ImVec2(0, 0))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Freq (Hz)");
            ImGui::TableSetupColumn("Dir");
            ImGui::TableSetupColumn("Conf");
            ImGui::TableSetupColumn("Bits");
            ImGui::TableSetupColumn("Timestamp (s)");
            ImGui::TableHeadersRow();
            for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::Text("%.0f", it->frequency);
                ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(it->direction == 1 ? "DL" : "UL");
                ImGui::TableSetColumnIndex(2); ImGui::Text("%d%%", it->confidence);
                ImGui::TableSetColumnIndex(3); ImGui::Text("%d", it->n_bits);
                ImGui::TableSetColumnIndex(4); ImGui::Text("%.6f", it->timestamp_ns / 1e9);
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void draw_aircraft_info_window() {
    if (!g_show_aircraft_info) return;
    ImGui::SetNextWindowSize(ImVec2(460, 0), ImGuiCond_Appearing);
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (!ImGui::Begin("Aircraft Info", &g_show_aircraft_info, ImGuiWindowFlags_NoCollapse)) {
        ImGui::End();
        return;
    }

    iridi::AircraftInfo inf = g_lookup.get_aircraft(g_lookup_reg);
    ImGui::Text("Registration: %s", g_lookup_reg.c_str());
    ImGui::Separator();

    if (!g_lookup_enabled) {
        ImGui::TextDisabled("Aircraft lookups are disabled.");
    } else {
        switch (inf.status) {
        case iridi::AircraftInfo::Pending:
        case iridi::AircraftInfo::None:
            ImGui::TextDisabled("Looking up %s ...", g_lookup_reg.c_str());
            break;
        case iridi::AircraftInfo::Ok: {
            auto row = [](const char *k, const std::string &v) {
                if (!v.empty()) ImGui::Text("%s: %s", k, v.c_str());
            };
            row("Operator", inf.owner);
            row("Aircraft", inf.type);
            row("ICAO type", inf.icao_type);
            row("Manufacturer", inf.manufacturer);
            row("Mode-S", inf.mode_s);
            row("Country", inf.country);
            row("Operator code", inf.operator_code);
            if (!inf.photo_url.empty()) {
                if (ImGui::Button("View photo")) open_url(inf.photo_url.c_str());
            }
            break;
        }
        case iridi::AircraftInfo::NotFound:
            ImGui::TextDisabled("No information found for this registration.");
            break;
        default:
            ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f), "Lookup failed: %s",
                               inf.error.c_str());
            if (ImGui::Button("Retry")) g_lookup.request_aircraft(g_lookup_reg, true);
            break;
        }
    }

    if (!g_lookup_flight.empty()) {
        iridi::AircraftInfo rt = g_lookup.get_route(g_lookup_flight);
        if (rt.status == iridi::AircraftInfo::Ok) {
            ImGui::Separator();
            ImGui::Text("Flight: %s", g_lookup_flight.c_str());
            if (!rt.airline.empty()) ImGui::Text("Airline: %s", rt.airline.c_str());
            if (!rt.origin.empty() || !rt.destination.empty())
                ImGui::Text("Route: %s -> %s", rt.origin.c_str(), rt.destination.c_str());
        }
    }

    ImGui::Separator();
    if (ImGui::Button("Close")) g_show_aircraft_info = false;
    ImGui::End();
}

void draw_acars_window() {
    if (!g_show_acars) return;
    ImGui::SetNextWindowSize(ImVec2(980, 300), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("ACARS", &g_show_acars)) {
        auto msgs = g_engine.acars().messages();
        const auto &as = g_engine.acars();
        ImGui::TextDisabled("%zu ACARS messages   [IDA asm %llu, SBD pkts %llu]",
                            msgs.size(), (unsigned long long)as.ida_assembled(),
                            (unsigned long long)as.sbd_packets());
        ImGui::SameLine();
        if (ImGui::Checkbox("Look up aircraft info", &g_lookup_enabled))
            g_lookup.set_enabled(g_lookup_enabled);

        if (ImGui::BeginTable("acars", 7,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                ImVec2(0, 0))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableSetupColumn("Dir", ImGuiTableColumnFlags_WidthFixed, 30);
            ImGui::TableSetupColumn("Reg", ImGuiTableColumnFlags_WidthFixed, 84);
            ImGui::TableSetupColumn("Label", ImGuiTableColumnFlags_WidthFixed, 96);
            ImGui::TableSetupColumn("bID", ImGuiTableColumnFlags_WidthFixed, 28);
            ImGui::TableSetupColumn("622", ImGuiTableColumnFlags_WidthFixed, 72);
            ImGui::TableSetupColumn("Text", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();
            for (auto it = msgs.rbegin(); it != msgs.rend(); ++it) {
                const auto &m = *it;
                ImGui::TableNextRow();
                char rowid[96];
                snprintf(rowid, sizeof(rowid), "%llu|%s|%s|%s",
                         (unsigned long long)m.timestamp_ns, m.reg.c_str(), m.bid.c_str(),
                         m.label.c_str());
                ImGui::PushID(rowid);
                time_t t = (time_t)(m.timestamp_ns / 1000000000ULL);
                struct tm tmv; localtime_s(&tmv, &t);
                char tbuf[32];
                snprintf(tbuf, sizeof(tbuf), "%02d:%02d:%02d", tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
                ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(tbuf);
                ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(m.ul ? "UL" : "DL");
                ImGui::TableSetColumnIndex(2);
                if (m.reg.empty()) {
                    ImGui::TextUnformatted("-");
                } else {
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.55f, 0.8f, 1.0f, 1.0f));
                    bool clicked = ImGui::Selectable(m.reg.c_str());
                    ImGui::PopStyleColor();
                    if (clicked) {
                        g_lookup_reg = m.reg;
                        g_lookup_flight = m.flight;
                        if (g_lookup_enabled) {
                            g_lookup.request_aircraft(m.reg);
                            if (!m.flight.empty()) g_lookup.request_route(m.flight);
                        }
                        g_show_aircraft_info = true;
                    }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Click for aircraft info");
                }
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%s (%s)", m.label.c_str(), m.label_desc.c_str());
                ImGui::TableSetColumnIndex(4); ImGui::TextUnformatted(m.bid.c_str());
                ImGui::TableSetColumnIndex(5);
                if (!m.arinc622_kind.empty()) {
                    if (ImGui::Selectable(m.arinc622_kind.c_str())) {
                        g_acars_detail = m;
                        g_show_acars_detail = true;
                    }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Click for ARINC-622 detail");
                } else if (!m.sublabel.empty() || !m.mfi.empty()) {
                    ImGui::Text("%s%s", m.sublabel.c_str(), m.mfi.c_str());
                } else {
                    ImGui::TextUnformatted("-");
                }
                ImGui::TableSetColumnIndex(6);
                ImVec4 col = m.crc_ok ? ImVec4(0.6f, 1.0f, 0.6f, 1.0f)
                                      : ImVec4(0.9f, 0.8f, 0.5f, 1.0f);
                float wrap_x = ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x;
                ImGui::PushTextWrapPos(wrap_x);
                ImGui::PushStyleColor(ImGuiCol_Text, col);
                ImGui::TextWrapped("%s", m.text.c_str());
                ImGui::PopStyleColor();
                ImGui::PopTextWrapPos();
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void draw_acars_detail_window() {
    if (!g_show_acars_detail) return;
    ImGui::SetNextWindowSize(ImVec2(720, 520), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("ACARS Detail", &g_show_acars_detail)) {
        const auto &m = g_acars_detail;
        ImGui::Text("Reg: %s   Flight: %s   Label: %s %s   bID: %s",
                    m.reg.c_str(), m.flight.c_str(), m.label.c_str(),
                    m.label_desc.c_str(), m.bid.c_str());
        ImGui::Text("Sublabel: %s   MFI: %s   Msg: %s   Errors: %s",
                    m.sublabel.c_str(), m.mfi.c_str(), m.msg_num.c_str(),
                    m.errors.c_str());
        ImGui::Separator();
        if (!m.arinc622_text.empty()) {
            ImGui::TextWrapped("%s", m.arinc622_text.c_str());
        } else {
            ImGui::TextDisabled("No ARINC-622 payload");
        }
        if (!m.arinc622_json.empty() && ImGui::CollapsingHeader("JSON")) {
            ImGui::TextWrapped("%s", m.arinc622_json.c_str());
        }
    }
    ImGui::End();
}

static void pager_time(uint64_t ts_ns) {
    time_t t = (time_t)(ts_ns / 1000000000ULL);
    struct tm tmv; localtime_s(&tmv, &t);
    ImGui::Text("%02d:%02d:%02d", tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
}

static void pager_text_cell(const std::string &text, ImVec4 col) {
    float wrap_x = ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x;
    ImGui::PushTextWrapPos(wrap_x);
    ImGui::PushStyleColor(ImGuiCol_Text, col);
    ImGui::TextWrapped("%s", text.c_str());
    ImGui::PopStyleColor();
    ImGui::PopTextWrapPos();
}

void draw_pager_window() {
    if (!g_show_pager) return;
    ImGui::SetNextWindowSize(ImVec2(760, 300), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Pager", &g_show_pager)) {
        auto pager = g_engine.pager().messages();
        ImGui::TextDisabled("%zu pager messages", pager.size());
        if (ImGui::BeginTable("pager", 5,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                ImVec2(0, 0))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableSetupColumn("RIC", ImGuiTableColumnFlags_WidthFixed, 80);
            ImGui::TableSetupColumn("Seq", ImGuiTableColumnFlags_WidthFixed, 40);
            ImGui::TableSetupColumn("Parts", ImGuiTableColumnFlags_WidthFixed, 44);
            ImGui::TableSetupColumn("Text", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();
            for (auto it = pager.rbegin(); it != pager.rend(); ++it) {
                const auto &m = *it;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); pager_time(m.timestamp_ns);
                ImGui::TableSetColumnIndex(1); ImGui::Text("%07d", m.ric);
                ImGui::TableSetColumnIndex(2); ImGui::Text("%02d", m.seq);
                ImGui::TableSetColumnIndex(3); ImGui::Text("%d", m.parts > 0 ? m.parts : 1);
                ImGui::TableSetColumnIndex(4);
                pager_text_cell(m.text, m.ok ? ImVec4(0.6f, 1, 0.6f, 1)
                                             : ImVec4(0.9f, 0.8f, 0.5f, 1));
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void draw_sms_window() {
    if (!g_show_sms) return;
    ImGui::SetNextWindowSize(ImVec2(760, 300), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("SMS", &g_show_sms)) {
        auto sms = g_engine.acars().sms_messages();
        ImGui::TextDisabled("%zu SMS messages", sms.size());
        if (ImGui::BeginTable("sms", 4,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                ImVec2(0, 0))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableSetupColumn("Dir", ImGuiTableColumnFlags_WidthFixed, 30);
            ImGui::TableSetupColumn("From / To", ImGuiTableColumnFlags_WidthFixed, 120);
            ImGui::TableSetupColumn("Text", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();
            for (auto it = sms.rbegin(); it != sms.rend(); ++it) {
                const auto &m = *it;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); pager_time(m.timestamp_ns);
                ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(m.ul ? "UL" : "DL");
                ImGui::TableSetColumnIndex(2);
                ImGui::TextUnformatted((m.ul ? m.to : m.from).c_str());
                ImGui::TableSetColumnIndex(3);
                pager_text_cell(m.text, m.ok ? ImVec4(0.6f, 1, 0.6f, 1)
                                             : ImVec4(0.9f, 0.8f, 0.5f, 1));
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void draw_sbd_window() {
    if (!g_show_sbd) return;
    ImGui::SetNextWindowSize(ImVec2(900, 320), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("SBD", &g_show_sbd)) {
        auto msgs = g_engine.acars().sbd_messages();
        static bool hide_acars = true;
        ImGui::TextDisabled("%zu SBD messages", msgs.size());
        ImGui::SameLine();
        ImGui::Checkbox("Hide ACARS", &hide_acars);
        if (ImGui::BeginTable("sbd", 6,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                ImVec2(0, 0))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableSetupColumn("Dir", ImGuiTableColumnFlags_WidthFixed, 30);
            ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 64);
            ImGui::TableSetupColumn("Len", ImGuiTableColumnFlags_WidthFixed, 40);
            ImGui::TableSetupColumn("Info", ImGuiTableColumnFlags_WidthFixed, 300);
            ImGui::TableSetupColumn("Payload / Fix", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();
            for (auto it = msgs.rbegin(); it != msgs.rend(); ++it) {
                const auto &m = *it;
                if (hide_acars && m.acars) continue;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); pager_time(m.timestamp_ns);
                ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(m.ul ? "UL" : "DL");
                ImGui::TableSetColumnIndex(2); ImGui::TextUnformatted(m.type.c_str());
                ImGui::TableSetColumnIndex(3); ImGui::Text("%d", m.length);
                ImGui::TableSetColumnIndex(4);
                ImGui::TextUnformatted(m.info.empty() ? m.preheader_hex.c_str() : m.info.c_str());
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("preheader: %s", m.preheader_hex.c_str());
                ImGui::TableSetColumnIndex(5);
                if (m.has_nal)
                    pager_text_cell(m.nal_info, ImVec4(0.6f, 1.0f, 0.6f, 1.0f));
                else
                    pager_text_cell(m.text, ImVec4(0.7f, 0.85f, 1.0f, 1.0f));
                if (ImGui::IsItemHovered()) {
                    std::string hx = m.payload_hex;
                    if (hx.size() > 900) hx = hx.substr(0, 900) + "...";
                    ImGui::SetTooltip("%s", hx.c_str());
                }
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void draw_paging_window() {
    if (!g_show_paging) return;
    ImGui::SetNextWindowSize(ImVec2(420, 300), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Paging", &g_show_paging)) {
        auto pages = g_engine.paging_events(2000);
        ImGui::TextDisabled("%zu paging events", pages.size());
        if (ImGui::BeginTable("paging", 5,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                ImVec2(0, 0))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableSetupColumn("Sat", ImGuiTableColumnFlags_WidthFixed, 40);
            ImGui::TableSetupColumn("Beam", ImGuiTableColumnFlags_WidthFixed, 45);
            ImGui::TableSetupColumn("TMSI", ImGuiTableColumnFlags_WidthFixed, 80);
            ImGui::TableSetupColumn("MSC", ImGuiTableColumnFlags_WidthFixed, 45);
            ImGui::TableHeadersRow();
            for (auto it = pages.rbegin(); it != pages.rend(); ++it) {
                const auto &p = *it;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); pager_time(p.timestamp_ns);
                ImGui::TableSetColumnIndex(1); ImGui::Text("%03d", p.sat);
                ImGui::TableSetColumnIndex(2); ImGui::Text("%02d", p.beam);
                ImGui::TableSetColumnIndex(3); ImGui::Text("%08x", (unsigned)p.tmsi);
                ImGui::TableSetColumnIndex(4); ImGui::Text("%02d", p.msc_id);
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void draw_signaling_window() {
    if (!g_show_signaling) return;
    ImGui::SetNextWindowSize(ImVec2(980, 300), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Signaling", &g_show_signaling)) {
        auto msgs = g_engine.acars().l3_messages();
        ImGui::TextDisabled("%zu L3 messages", msgs.size());
        if (ImGui::BeginTable("l3", 5,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                ImVec2(0, 0))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableSetupColumn("Dir", ImGuiTableColumnFlags_WidthFixed, 30);
            ImGui::TableSetupColumn("Proto", ImGuiTableColumnFlags_WidthFixed, 64);
            ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 150);
            ImGui::TableSetupColumn("Detail", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();
            for (auto it = msgs.rbegin(); it != msgs.rend(); ++it) {
                const auto &m = *it;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); pager_time(m.timestamp_ns);
                ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(m.ul ? "UL" : "DL");
                ImGui::TableSetColumnIndex(2); ImGui::TextUnformatted(m.proto.c_str());
                ImGui::TableSetColumnIndex(3); ImGui::TextUnformatted(m.type.c_str());
                ImGui::TableSetColumnIndex(4);
                float wrap_x = ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x;
                ImGui::PushTextWrapPos(wrap_x);
                ImGui::TextWrapped("%s", m.summary.c_str());
                ImGui::PopTextWrapPos();
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", m.raw_hex.c_str());
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void draw_broadcast_window() {
    if (!g_show_broadcast) return;
    ImGui::SetNextWindowSize(ImVec2(1100, 300), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Broadcast", &g_show_broadcast)) {
        auto msgs = g_engine.recent_messages(6000);
        size_t count = 0;
        for (const auto &m : msgs) if (m.has_ibc) count++;
        ImGui::TextDisabled("%zu broadcast frames", count);
        if (ImGui::BeginTable("ibc", 7,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                ImVec2(0, 0))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableSetupColumn("Sat", ImGuiTableColumnFlags_WidthFixed, 40);
            ImGui::TableSetupColumn("Cell", ImGuiTableColumnFlags_WidthFixed, 40);
            ImGui::TableSetupColumn("BC/ST", ImGuiTableColumnFlags_WidthFixed, 60);
            ImGui::TableSetupColumn("Time/Expiry", ImGuiTableColumnFlags_WidthFixed, 150);
            ImGui::TableSetupColumn("Pwr", ImGuiTableColumnFlags_WidthFixed, 40);
            ImGui::TableSetupColumn("Assignments", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();
            for (auto it = msgs.rbegin(); it != msgs.rend(); ++it) {
                const auto &m = *it;
                if (!m.has_ibc) continue;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); pager_time(m.timestamp_ns);
                ImGui::TableSetColumnIndex(1); ImGui::Text("%03d", m.ibc_sat);
                ImGui::TableSetColumnIndex(2); ImGui::Text("%02d", m.ibc_beam);
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%d/%d", m.ibc_type, m.ibc_subtype);
                ImGui::TableSetColumnIndex(4);
                if (m.ibc_has_time)
                    ImGui::TextUnformatted(iridi::fmt_iritime(m.ibc_iri_time).c_str());
                else if (m.ibc_has_tmsi_expiry)
                    ImGui::Text("%s", iridi::fmt_iritime(m.ibc_tmsi_expiry).c_str());
                else
                    ImGui::TextUnformatted("-");
                ImGui::TableSetColumnIndex(5);
                if (m.ibc_subtype == 0) ImGui::Text("%d", m.ibc_max_uplink_pwr);
                else ImGui::TextUnformatted("-");
                ImGui::TableSetColumnIndex(6);
                if (m.ibc_assignments.empty()) {
                    ImGui::TextUnformatted("-");
                } else {
                    std::string txt;
                    for (const auto &a : m.ibc_assignments) {
                        if (!txt.empty()) txt += " ";
                        if (a.empty) { txt += "[]"; continue; }
                        char b[96];
                        if (a.type == 0)
                            snprintf(b, sizeof(b), "[Rid:%03d ts:%d %02d/%02d acc:%d %+d/%d]",
                                     a.random_id, a.timeslot, a.ul_sb, a.dl_sb,
                                     a.access, a.dtoa, a.dfoa);
                        else
                            snprintf(b, sizeof(b), "[t%d]", a.type);
                        txt += b;
                    }
                    ImGui::TextUnformatted(txt.c_str());
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", m.pretty.c_str());
                }
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void draw_itl_window() {
    if (!g_show_itl) return;
    ImGui::SetNextWindowSize(ImVec2(760, 300), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("ITL", &g_show_itl)) {
        auto msgs = g_engine.recent_messages(6000);
        size_t count = 0, ok = 0;
        for (const auto &m : msgs) if (m.has_itl) { count++; if (m.itl_ok) ok++; }
        ImGui::TextDisabled("%zu ITL frames (%zu decoded)", count, ok);
        if (ImGui::BeginTable("itl", 6,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                ImVec2(0, 0))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableSetupColumn("Ver", ImGuiTableColumnFlags_WidthFixed, 40);
            ImGui::TableSetupColumn("Plane", ImGuiTableColumnFlags_WidthFixed, 50);
            ImGui::TableSetupColumn("Sat", ImGuiTableColumnFlags_WidthFixed, 50);
            ImGui::TableSetupColumn("Msg", ImGuiTableColumnFlags_WidthFixed, 50);
            ImGui::TableSetupColumn("Words / Detail", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();
            for (auto it = msgs.rbegin(); it != msgs.rend(); ++it) {
                const auto &m = *it;
                if (!m.has_itl) continue;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); pager_time(m.timestamp_ns);
                ImGui::TableSetColumnIndex(1); ImGui::Text("V%d", m.itl_version);
                ImGui::TableSetColumnIndex(2);
                if (m.itl_plane) ImGui::Text("P%d", m.itl_plane); else ImGui::TextUnformatted("-");
                ImGui::TableSetColumnIndex(3); ImGui::TextUnformatted(m.itl_sat.c_str());
                ImGui::TableSetColumnIndex(4); ImGui::TextUnformatted(m.itl_msg.c_str());
                ImGui::TableSetColumnIndex(5);
                if (m.itl_ok && !m.itl_words.empty()) {
                    char b[96];
                    snprintf(b, sizeof(b), "%03d %03d %03d",
                             m.itl_words[0], m.itl_words[1], m.itl_words[2]);
                    ImGui::TextUnformatted(b);
                } else {
                    ImGui::TextUnformatted(m.pretty.c_str());
                }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", m.pretty.c_str());
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void update_autoplay() {
    auto calls = g_engine.voices().calls();
    for (const auto &c : calls) {
        if (c.active || c.id <= g_autoplay_last_id) continue;
        g_autoplay_last_id = c.id;
        if (g_autoplay) g_autoplay_queue.push_back(c.id);
    }
    if (!g_autoplay) {
        g_autoplay_queue.clear();
        return;
    }
    if (!g_audio.playing() && !g_autoplay_queue.empty()) {
        int id = g_autoplay_queue.front();
        g_autoplay_queue.pop_front();
        auto pcm = g_engine.voices().pcm(id);
        if (!pcm.empty()) {
            g_audio.play(pcm, 8000);
            add_log("Auto play: voice call #%d (%.1f s)", id, pcm.size() / 8000.0);
        }
    }
}

void draw_voice_window() {
    if (!g_show_voice) return;
    ImGui::SetNextWindowSize(ImVec2(680, 320), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Voice", &g_show_voice)) {
        auto calls = g_engine.voices().calls();
        int active = 0; for (auto &c : calls) if (c.active) active++;
        ImGui::TextDisabled("%zu voice calls (%d active)  [%s]", calls.size(), active,
                            g_audio.playing() ? "playing" : "idle");
        ImGui::SameLine();
        if (ImGui::Button("Stop audio")) g_audio.stop();
        ImGui::SameLine();
        ImGui::Checkbox("Auto play", &g_autoplay);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Automatically play new voice calls as they complete");
        ImGui::SameLine();
        float vol = g_audio.volume();
        ImGui::SetNextItemWidth(120);
        if (ImGui::SliderFloat("Volume", &vol, 0.0f, 4.0f, "%.1fx"))
            g_audio.set_volume(vol);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Calls are peak-normalised, then amplified by this amount");
        if (ImGui::BeginTable("calls", 7,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                ImVec2(0, 140))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, 46);
            ImGui::TableSetupColumn("MHz", ImGuiTableColumnFlags_WidthFixed, 88);
            ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 52);
            ImGui::TableSetupColumn("Dur s", ImGuiTableColumnFlags_WidthFixed, 56);
            ImGui::TableSetupColumn("Frames", ImGuiTableColumnFlags_WidthFixed, 56);
            ImGui::TableSetupColumn("Err", ImGuiTableColumnFlags_WidthFixed, 50);
            ImGui::TableSetupColumn("State");
            ImGui::TableHeadersRow();
            for (auto it = calls.rbegin(); it != calls.rend(); ++it) {
                const auto &cv = *it;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                char lbl[32]; snprintf(lbl, sizeof(lbl), "#%d", cv.id);
                if (ImGui::Selectable(lbl, g_selected_voice == cv.id, ImGuiSelectableFlags_SpanAllColumns)) {
                    g_selected_voice = cv.id; g_voice_cache_id = -1;
                }
                ImGui::TableSetColumnIndex(1); ImGui::Text("%.4f", cv.channel_hz / 1e6);
                ImGui::TableSetColumnIndex(2); ImGui::TextUnformatted(cv.subtype.c_str());
                ImGui::TableSetColumnIndex(3); ImGui::Text("%.1f", cv.duration_s);
                ImGui::TableSetColumnIndex(4); ImGui::Text("%d", cv.frames);
                ImGui::TableSetColumnIndex(5); ImGui::Text("%.2f", cv.error_ratio);
                ImGui::TableSetColumnIndex(6);
                ImGui::TextColored(cv.error_ratio < 0.5f ? ImVec4(0.5f, 1, 0.5f, 1)
                                                         : ImVec4(0.9f, 0.7f, 0.4f, 1),
                                   "%s", cv.active ? "active" : "done");
            }
            ImGui::EndTable();
        }
        if (g_selected_voice >= 0) {
            if (g_voice_cache_id != g_selected_voice) {
                g_voice_cache = g_engine.voices().pcm(g_selected_voice);
                g_voice_cache_id = g_selected_voice;
            }
            if (!g_voice_cache.empty()) {
                ImGui::Text("Call #%d  (%zu samples, %.1f s)", g_selected_voice,
                            g_voice_cache.size(), g_voice_cache.size() / 8000.0);
                if (ImGui::Button("Play")) g_audio.play(g_voice_cache, 8000);
                ImGui::SameLine();
                if (ImGui::Button("Stop")) g_audio.stop();
                if (ImPlot::BeginPlot("##wave", ImVec2(-1, -1), ImPlotFlags_NoLegend)) {
                    const auto &pcm = g_voice_cache;
                    int n = (int)pcm.size();
                    int step = std::max(1, n / 4000);
                    std::vector<float> xs, ys;
                    xs.reserve(n / step + 1); ys.reserve(n / step + 1);
                    for (int i = 0; i < n; i += step) {
                        xs.push_back((float)i / 8000.0f);
                        ys.push_back(pcm[i] / 32768.0f);
                    }
                    ImPlot::SetupAxes("s", "", 0, ImPlotAxisFlags_AutoFit);
                    ImPlot::PlotLine("wave", xs.data(), ys.data(), (int)xs.size());
                    ImPlot::EndPlot();
                }
            } else {
                ImGui::TextDisabled("(no audio captured for this call yet)");
            }
        }
    }
    ImGui::End();
}

void request_scan(iridi::LiveIqSource &sdr, std::vector<iridi::SdrDeviceInfo> &devs,
                  std::mutex &mtx, std::atomic<bool> &scanning, std::thread &th) {
    if (scanning.load()) return;
    if (th.joinable()) th.join();
    scanning.store(true);
    th = std::thread([&sdr, &devs, &mtx, &scanning] {
        std::vector<iridi::SdrDeviceInfo> d = sdr.list_devices();
        {
            std::lock_guard<std::mutex> lk(mtx);
            devs = std::move(d);
        }
        scanning.store(false);
    });
}

void request_sdr_device_scan() {
    g_sdr_scan_requested = true;
    request_scan(g_sdr, g_sdr_devices, g_sdr_dev_mtx, g_sdr_scanning, g_sdr_scan_thread);
}

void request_hrf_device_scan() {
    g_hrf_scan_requested = true;
    request_scan(g_hackrf, g_hrf_devices, g_hrf_dev_mtx, g_hrf_scanning, g_hrf_scan_thread);
}

void request_ap_device_scan() {
    g_ap_scan_requested = true;
    request_scan(g_airspy, g_ap_devices, g_ap_dev_mtx, g_ap_scanning, g_ap_scan_thread);
}

void request_rtl_device_scan() {
    g_rtl_scan_requested = true;
    request_scan(g_rtlsdr, g_rtl_devices, g_rtl_dev_mtx, g_rtl_scanning, g_rtl_scan_thread);
}

void draw_sdr_source_controls() {
    // Kick a device scan the first time the SDR source is shown.
    if (g_sdr_devices.empty() && !g_sdr_scanning.load() && !g_sdr_scan_requested)
        request_sdr_device_scan();

    std::vector<iridi::SdrDeviceInfo> devs;
    {
        std::lock_guard<std::mutex> lk(g_sdr_dev_mtx);
        devs = g_sdr_devices;
    }

    bool running = g_engine.running();
    const bool scanning = g_sdr_scanning.load();

    if (scanning) {
        ImGui::TextDisabled("Scanning for SDRplay devices...");
    } else if (devs.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f), "No SDRplay device found.");
    } else {
        std::vector<const char *> names;
        names.reserve(devs.size());
        for (auto &d : devs) names.push_back(d.name.c_str());
        ImGui::SetNextItemWidth(200);
        ImGui::Combo("Device", &g_sdr_device_idx, names.data(), (int)names.size());
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(scanning);
    if (ImGui::Button("Refresh")) request_sdr_device_scan();
    ImGui::EndDisabled();

    ImGui::SetNextItemWidth(200);
    if (ImGui::Combo("Sample rate (Msps)", &g_sdr_rate_idx, kSdrplayRateLabels, kNumSdrplayRates)) {
        if (g_sdr.is_open()) apply_sample_rate(1, kSdrplayRates[g_sdr_rate_idx]);
    }

    if (ImGui::Checkbox("AGC", &g_sdr_agc)) {
        if (running) g_sdr.set_gain(g_sdr_agc ? -1.0 : (double)g_sdr_gr);
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(g_sdr_agc);
    ImGui::SetNextItemWidth(180);
    if (ImGui::SliderFloat("Gain reduction (dB)", &g_sdr_gr, 20.0f, 59.0f, "%.0f")) {
        if (running) g_sdr.set_gain((double)g_sdr_gr);
    }
    ImGui::EndDisabled();

    if (ImGui::Checkbox("Bias tee", &g_sdr_bias)) g_sdr.set_bias_tee(g_sdr_bias);
    ImGui::SameLine();
    ImGui::TextDisabled("(powers an inline LNA/antenna)");

#ifndef HAS_SDRPLAY
    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f),
                       "SDRplay support not compiled in (API not found at build time).");
    ImGui::BeginDisabled();
#endif
    if (!g_sdr.is_open()) {
        ImGui::BeginDisabled(devs.empty() || scanning);
        if (ImGui::Button("Start SDRplay")) start_sdr();
        ImGui::EndDisabled();
    } else {
        if (ImGui::Button("Stop SDRplay")) {
    g_engine.stop();
    g_lookup.stop();
    g_sdr.close();
            add_log("SDRplay stopped");
        }
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.4f, 1, 0.4f, 1), "streaming: %s", g_sdr.device_name().c_str());
    }
#ifndef HAS_SDRPLAY
    ImGui::EndDisabled();
#endif
}

void draw_hackrf_source_controls() {
    if (g_hrf_devices.empty() && !g_hrf_scanning.load() && !g_hrf_scan_requested)
        request_hrf_device_scan();

    std::vector<iridi::SdrDeviceInfo> devs;
    {
        std::lock_guard<std::mutex> lk(g_hrf_dev_mtx);
        devs = g_hrf_devices;
    }
    const bool scanning = g_hrf_scanning.load();
    if (scanning) {
        ImGui::TextDisabled("Scanning for HackRF devices...");
    } else if (devs.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f), "No HackRF device found.");
    } else {
        std::vector<const char *> names;
        names.reserve(devs.size());
        for (auto &d : devs) names.push_back(d.name.c_str());
        ImGui::SetNextItemWidth(200);
        ImGui::Combo("Device", &g_hrf_device_idx, names.data(), (int)names.size());
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(scanning);
    if (ImGui::Button("Refresh")) request_hrf_device_scan();
    ImGui::EndDisabled();

    ImGui::SetNextItemWidth(200);
    if (ImGui::Combo("Sample rate (Msps)", &g_hrf_rate_idx, kHackrfRateLabels, kNumHackrfRates)) {
        if (g_hackrf.is_open()) apply_sample_rate(2, kHackrfRates[g_hrf_rate_idx]);
    }

    ImGui::SetNextItemWidth(200);
    if (ImGui::SliderInt("IF/LNA gain (dB)", &g_hrf_lna, 0, 40, "%d"))
        g_hackrf.set_lna_gain(g_hrf_lna);
    ImGui::SetNextItemWidth(200);
    if (ImGui::SliderInt("Baseband/VGA gain (dB)", &g_hrf_vga, 0, 62, "%d"))
        g_hackrf.set_vga_gain(g_hrf_vga);
    if (ImGui::Checkbox("RF amp (+11 dB)", &g_hrf_amp)) g_hackrf.set_amp(g_hrf_amp);
    ImGui::SameLine();
    if (ImGui::Checkbox("Bias tee", &g_hrf_bias)) g_hackrf.set_bias_tee(g_hrf_bias);

#ifndef HAS_HACKRF
    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f),
                       "HackRF support not compiled in (libhackrf not found at build time).");
    ImGui::BeginDisabled();
#endif
    if (!g_hackrf.is_open()) {
        ImGui::BeginDisabled(devs.empty() || scanning);
        if (ImGui::Button("Start HackRF")) start_hackrf();
        ImGui::EndDisabled();
    } else {
        if (ImGui::Button("Stop HackRF")) {
            g_engine.stop();
            g_hackrf.close();
            add_log("HackRF stopped");
        }
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.4f, 1, 0.4f, 1), "streaming: %s",
                           g_hackrf.device_name().c_str());
    }
#ifndef HAS_HACKRF
    ImGui::EndDisabled();
#endif
}

void draw_airspy_source_controls() {
    if (g_ap_devices.empty() && !g_ap_scanning.load() && !g_ap_scan_requested)
        request_ap_device_scan();

    std::vector<iridi::SdrDeviceInfo> devs;
    {
        std::lock_guard<std::mutex> lk(g_ap_dev_mtx);
        devs = g_ap_devices;
    }
    const bool scanning = g_ap_scanning.load();

    if (scanning) {
        ImGui::TextDisabled("Scanning for Airspy devices...");
    } else if (devs.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f), "No Airspy device found.");
    } else {
        std::vector<const char *> names;
        names.reserve(devs.size());
        for (auto &d : devs) names.push_back(d.name.c_str());
        ImGui::SetNextItemWidth(200);
        ImGui::Combo("Device", &g_ap_device_idx, names.data(), (int)names.size());
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(scanning);
    if (ImGui::Button("Refresh")) request_ap_device_scan();
    ImGui::EndDisabled();

    ImGui::SetNextItemWidth(200);
    if (ImGui::Combo("Sample rate (Msps)", &g_ap_rate_idx, kAirspyRateLabels, kNumAirspyRates)) {
        if (g_airspy.is_open()) apply_sample_rate(3, kAirspyRates[g_ap_rate_idx]);
    }

    const char *modes[] = {"Sensitivity", "Linear", "Free"};
    ImGui::SetNextItemWidth(200);
    if (ImGui::Combo("Gain mode", &g_ap_gain_mode, modes, 3))
        g_airspy.set_gain_mode(g_ap_gain_mode);

    if (g_ap_gain_mode == 0) {
        ImGui::SetNextItemWidth(200);
        if (ImGui::SliderInt("Sensitivity gain", &g_ap_sense, 0, 21, "%d"))
            g_airspy.set_sense_gain(g_ap_sense);
    } else if (g_ap_gain_mode == 1) {
        ImGui::SetNextItemWidth(200);
        if (ImGui::SliderInt("Linearity gain", &g_ap_linear, 0, 21, "%d"))
            g_airspy.set_linear_gain(g_ap_linear);
    } else {
        if (ImGui::Checkbox("LNA AGC", &g_ap_lna_agc)) g_airspy.set_lna_agc(g_ap_lna_agc);
        ImGui::BeginDisabled(g_ap_lna_agc);
        ImGui::SetNextItemWidth(200);
        if (ImGui::SliderInt("LNA gain", &g_ap_lna, 0, 15, "%d")) g_airspy.set_lna_gain(g_ap_lna);
        ImGui::EndDisabled();
        if (ImGui::Checkbox("Mixer AGC", &g_ap_mixer_agc)) g_airspy.set_mixer_agc(g_ap_mixer_agc);
        ImGui::BeginDisabled(g_ap_mixer_agc);
        ImGui::SetNextItemWidth(200);
        if (ImGui::SliderInt("Mixer gain", &g_ap_mixer, 0, 15, "%d"))
            g_airspy.set_mixer_gain(g_ap_mixer);
        ImGui::EndDisabled();
        ImGui::SetNextItemWidth(200);
        if (ImGui::SliderInt("VGA gain", &g_ap_vga, 0, 15, "%d")) g_airspy.set_vga_gain(g_ap_vga);
    }
    if (ImGui::Checkbox("Bias tee", &g_ap_bias)) g_airspy.set_bias_tee(g_ap_bias);

#ifndef HAS_AIRSPY
    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f),
                       "Airspy support not compiled in (libairspy not found at build time).");
    ImGui::BeginDisabled();
#endif
    if (!g_airspy.is_open()) {
        ImGui::BeginDisabled(devs.empty() || scanning);
        if (ImGui::Button("Start Airspy")) start_airspy();
        ImGui::EndDisabled();
    } else {
        if (ImGui::Button("Stop Airspy")) {
            g_engine.stop();
            g_airspy.close();
            add_log("Airspy stopped");
        }
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.4f, 1, 0.4f, 1), "streaming: %s",
                           g_airspy.device_name().c_str());
    }
#ifndef HAS_AIRSPY
    ImGui::EndDisabled();
#endif
}

void draw_rtlsdr_source_controls() {
    if (g_rtl_devices.empty() && !g_rtl_scanning.load() && !g_rtl_scan_requested)
        request_rtl_device_scan();

    std::vector<iridi::SdrDeviceInfo> devs;
    {
        std::lock_guard<std::mutex> lk(g_rtl_dev_mtx);
        devs = g_rtl_devices;
    }
    const bool scanning = g_rtl_scanning.load();

    if (scanning) {
        ImGui::TextDisabled("Scanning for RTL-SDR devices...");
    } else if (devs.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f), "No RTL-SDR device found.");
    } else {
        std::vector<const char *> names;
        names.reserve(devs.size());
        for (auto &d : devs) names.push_back(d.name.c_str());
        ImGui::SetNextItemWidth(220);
        ImGui::Combo("Device", &g_rtl_device_idx, names.data(), (int)names.size());
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(scanning);
    if (ImGui::Button("Refresh")) request_rtl_device_scan();
    ImGui::EndDisabled();

    ImGui::SetNextItemWidth(200);
    if (ImGui::Combo("Sample rate (Msps)", &g_rtl_rate_idx, kRtlRateLabels, kNumRtlRates)) {
        if (g_rtlsdr.is_open()) apply_sample_rate(4, kRtlRates[g_rtl_rate_idx]);
    }

    if (ImGui::Checkbox("AGC", &g_rtl_agc)) {
        if (g_rtlsdr.is_open()) g_rtlsdr.set_gain(g_rtl_agc ? -1.0 : (double)g_rtl_gain);
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(g_rtl_agc);
    ImGui::SetNextItemWidth(180);
    if (ImGui::SliderFloat("Tuner gain (dB)", &g_rtl_gain, 0.0f, 50.0f, "%.1f")) {
        if (g_rtlsdr.is_open()) g_rtlsdr.set_gain((double)g_rtl_gain);
    }
    ImGui::EndDisabled();

    ImGui::SetNextItemWidth(100);
    if (ImGui::InputInt("PPM", &g_rtl_ppm)) {
        g_rtl_ppm = std::clamp(g_rtl_ppm, -100, 100);
        if (g_rtlsdr.is_open()) g_rtlsdr.set_ppm(g_rtl_ppm);
    }
    if (ImGui::Checkbox("Bias tee", &g_rtl_bias)) {
        if (g_rtlsdr.is_open()) g_rtlsdr.set_bias_tee(g_rtl_bias);
    }

#ifndef HAS_RTLSDR
    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f),
                       "RTL-SDR support not compiled in (librtlsdr not found at build time).");
    ImGui::BeginDisabled();
#endif
    if (!g_rtlsdr.is_open()) {
        ImGui::BeginDisabled(devs.empty() || scanning);
        if (ImGui::Button("Start RTL-SDR")) start_rtl();
        ImGui::EndDisabled();
    } else {
        if (ImGui::Button("Stop RTL-SDR")) {
            g_engine.stop();
            g_rtlsdr.close();
            add_log("RTL-SDR stopped");
        }
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.4f, 1, 0.4f, 1), "streaming: %s",
                           g_rtlsdr.device_name().c_str());
    }
#ifndef HAS_RTLSDR
    ImGui::EndDisabled();
#endif
}

void draw_source_window() {
    if (!g_show_source) return;
    ImGui::SetNextWindowSize(ImVec2(520, 300), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Source", &g_show_source)) {
        const char *kinds[] = {"IQ file", "SDRplay", "HackRF", "Airspy", "RTL-SDR"};
        ImGui::SetNextItemWidth(160);
        if (ImGui::Combo("Source", &g_source_kind, kinds, 5)) {
            g_engine.stop();
            g_file.close();
            g_sdr.close();
            g_hackrf.close();
            g_airspy.close();
            g_rtlsdr.close();
            g_freq_mhz.clear();
            g_view_reset = true;
            if (g_source_kind == 1) g_sdr_scan_requested = false; // allow a fresh scan
            if (g_source_kind == 2) g_hrf_scan_requested = false;
            if (g_source_kind == 3) g_ap_scan_requested = false;
            if (g_source_kind == 4) g_rtl_scan_requested = false;
        }
        ImGui::Separator();

        if (g_source_kind == 0) {
            if (!g_file.is_open()) {
                ImGui::TextDisabled("No IQ file loaded.");
                if (ImGui::Button("Open IQ file...")) g_request_open = true;
            } else {
                const auto &in = g_file.info();
                ImGui::TextWrapped("%s", in.path.c_str());
                ImGui::Separator();
                if (ImGui::BeginTable("info", 2, ImGuiTableFlags_SizingStretchProp)) {
                    auto row = [](const char *k, const char *fmt, ...) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(k);
                        ImGui::TableSetColumnIndex(1);
                        char b[256]; va_list ap; va_start(ap, fmt);
                        vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
                        ImGui::TextUnformatted(b);
                    };
                    row("Format", "%s", iridi::iq_format_name(in.format));
                    row("Container", "%s", in.is_wav ? "WAV (RIFF)" : "raw");
                    row("Sample rate", "%d Hz", in.sample_rate);
                    row("Complex samples", "%llu", (unsigned long long)in.num_samples);
                    row("Duration", "%.3f s", in.duration_s);
                    if (in.center_frequency > 0) row("Center freq", "%.0f Hz", in.center_frequency);
                    ImGui::EndTable();
                }
            }
        } else if (g_source_kind == 1) {
            draw_sdr_source_controls();
        } else if (g_source_kind == 2) {
            draw_hackrf_source_controls();
        } else if (g_source_kind == 3) {
            draw_airspy_source_controls();
        } else {
            draw_rtlsdr_source_controls();
        }
        if (g_source_kind != 0) {
            ImGui::Separator();
            ImGui::Text("VFO: %.4f MHz", active_source()->center_frequency() / 1e6);
        }
    }
    ImGui::End();
}

void draw_about_window() {
    if (!g_show_about) return;
    ImGui::SetNextWindowSize(ImVec2(480, 400), ImGuiCond_Appearing);
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::Begin("About IridiScope###AboutIridiScope", &g_show_about,
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoDocking |
                     ImGuiWindowFlags_NoCollapse)) {
        ImGui::TextWrapped("IridiScope v" IRIDISCOPE_VERSION_STRING);
        ImGui::Separator();
        switch (g_version_check.state()) {
        case iridi::VersionCheck::Checking:
            ImGui::TextDisabled("Checking for updates...");
            break;
        case iridi::VersionCheck::UpdateAvailable: {
            std::string v = g_version_check.latest_version();
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "Update available: v%s", v.c_str());
            ImGui::SameLine();
            if (ImGui::SmallButton("Open download page"))
                open_url(g_version_check.product_url().c_str());
            break;
        }
        case iridi::VersionCheck::UpToDate:
            ImGui::TextDisabled("Up to date.");
            break;
        case iridi::VersionCheck::Unreleased:
            ImGui::TextDisabled("Development build (newer than the published release).");
            break;
        case iridi::VersionCheck::Error:
            ImGui::TextDisabled("Update check failed.");
            break;
        default:
            break;
        }
        ImGui::Separator();
        ImGui::TextWrapped("An Iridium L-band signal analysis workstation by Sarah Rose.");
        ImGui::Spacing();
        ImGui::TextWrapped("Built with components from:");
        ImGui::BulletText("iridium-sniffer (muccc)");
        ImGui::BulletText("iridium-toolkit (Sec, schneider)");
        ImGui::BulletText("osmo-ir77 AMBE vocoder (Sylvain Munaut / Osmocom)");
        ImGui::BulletText("Dear ImGui (Omar Cornut) and ImPlot (Evan Pezent)");
        ImGui::BulletText("GLFW (Camilla Lowy, Marcus Geelnard)");
        ImGui::BulletText("FFTW (Matteo Frigo, Steven G. Johnson)");
        ImGui::BulletText("miniaudio (David Reid) and stb (Sean Barrett)");
        ImGui::Spacing();
        ImGui::TextWrapped("SDR backends:");
        ImGui::BulletText("SDRplay API (SDRplay Ltd)");
        ImGui::BulletText("libhackrf (Great Scott Gadgets)");
        ImGui::BulletText("libairspy (Airspy)");
        ImGui::Spacing();
        ImGui::TextWrapped("SDR integration was modelled on AirScope by Sarah Rose.");
        ImGui::TextWrapped("Equirectangular map imagery courtesy of NASA.");
        ImGui::Spacing();
        ImGui::TextWrapped("Thanks to Jomjom for providing the IQ samples used to develop IridiScope.");
        ImGui::Spacing();
        ImGui::TextDisabled("Licensed under the GNU General Public License v3.");
    }
    ImGui::End();
}

void build_default_dock_layout(ImGuiID dockspace_id) {
    ImGui::DockBuilderRemoveNode(dockspace_id);
    ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace_id, ImGui::GetMainViewport()->Size);

    ImGuiID center = dockspace_id;
    ImGuiID bottom = ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.34f, nullptr, &center);
    ImGuiID right  = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.34f, nullptr, &center);
    ImGuiID left   = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.22f, nullptr, &center);

    ImGui::DockBuilderDockWindow("Map", center);
    ImGui::DockBuilderDockWindow("Spectrum", right);
    ImGui::DockBuilderDockWindow("Bursts", right);
    ImGui::DockBuilderDockWindow("Waterfall", bottom);
    ImGui::DockBuilderDockWindow("Messages", bottom);
    ImGui::DockBuilderDockWindow("Voice", bottom);
    ImGui::DockBuilderDockWindow("ACARS", bottom);
    ImGui::DockBuilderDockWindow("Pager", bottom);
    ImGui::DockBuilderDockWindow("SMS", bottom);
    ImGui::DockBuilderDockWindow("SBD", bottom);
    ImGui::DockBuilderDockWindow("Signaling", bottom);
    ImGui::DockBuilderDockWindow("Broadcast", bottom);
    ImGui::DockBuilderDockWindow("ITL", bottom);
    ImGui::DockBuilderDockWindow("Playback", left);
    ImGui::DockBuilderDockWindow("Stats", left);
    ImGui::DockBuilderDockWindow("Satellites", left);
    ImGui::DockBuilderDockWindow("Paging", left);
    ImGui::DockBuilderDockWindow("Source", left);
    ImGui::DockBuilderFinish(dockspace_id);
}

void apply_style() {
    ImGui::StyleColorsDark();
    ImGuiStyle &s = ImGui::GetStyle();
    s.WindowRounding = 4.0f;
    s.FrameRounding = 3.0f;
    s.GrabRounding = 3.0f;
    s.WindowBorderSize = 1.0f;
    ImVec4 accent = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    s.Colors[ImGuiCol_Header] = ImVec4(accent.x, accent.y, accent.z, 0.35f);
    s.Colors[ImGuiCol_Button] = ImVec4(accent.x, accent.y, accent.z, 0.45f);
}

} // namespace

static int print_file_info(const char *path) {
    iridi::IqFileSource src;
    std::string err;
    if (!src.open(path, err)) { fprintf(stderr, "%s\n", err.c_str()); return 2; }
    const auto &in = src.info();
    printf("path=%s\nformat=%s\ncontainer=%s\nsample_rate=%d\nchannels=%d\n",
           in.path.c_str(), iridi::iq_format_name(in.format), in.is_wav ? "wav" : "raw",
           in.sample_rate, in.channels);
    printf("num_samples=%llu\nduration_s=%.6f\ncenter_frequency=%.0f\n",
           (unsigned long long)in.num_samples, in.duration_s, in.center_frequency);
    return 0;
}

static int scan_raw(const char *path, double seconds, double start_s) {
    iridi::IqFileSource src;
    std::string err;
    if (!src.open(path, err)) { fprintf(stderr, "%s\n", err.c_str()); return 2; }
    if (start_s > 0.0 && src.info().sample_rate > 0) {
        uint64_t idx = (uint64_t)(start_s * (double)src.info().sample_rate);
        src.seek_sample(idx);
    }
    iridi::Engine eng;
    eng.set_raw_dump(true);
    if (!eng.start(&src)) { fprintf(stderr, "engine start failed\n"); return 3; }
    auto t0 = std::chrono::steady_clock::now();
    while (true) {
        auto s = eng.stats();
        if (s.done) break;
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > seconds)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    eng.stop();
    return 0;
}

static void print_type_tally(iridi::Engine &eng) {
    auto msgs = eng.recent_messages(4000);
    std::map<std::string, int> tally;
    size_t ims = 0, mframes = 0;
    for (const auto &m : msgs) {
        tally[m.type]++;
        if (m.has_ims) ims++;
        if (m.has_msg) mframes++;
    }
    printf("recent_types:");
    for (const auto &kv : tally) printf(" %s=%d", kv.first.c_str(), kv.second);
    printf("\nims=%zu msg_frames=%zu\n", ims, mframes);
}

static int scan_file(const char *path, double seconds) {
    iridi::IqFileSource src;
    std::string err;
    if (!src.open(path, err)) { fprintf(stderr, "%s\n", err.c_str()); return 2; }
    iridi::Engine eng;
    if (!eng.start(&src)) { fprintf(stderr, "engine start failed\n"); return 3; }
    auto t0 = std::chrono::steady_clock::now();
    while (true) {
        auto s = eng.stats();
        if (s.done) break;
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > seconds) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    auto s = eng.stats();
    printf("samples=%llu\nbursts=%llu\nframes_ok=%llu\nframes_fail=%llu\nelapsed_s=%.3f\ndone=%d\n",
           (unsigned long long)s.samples, (unsigned long long)s.bursts,
           (unsigned long long)s.frames_ok, (unsigned long long)s.frames_fail,
           s.elapsed_s, s.done ? 1 : 0);
    printf("parsed=%llu\nparse_errors=%llu\nsatellites=%zu\nmessages=%zu\n",
           (unsigned long long)eng.parsed_count(),
           (unsigned long long)eng.parse_error_count(),
           eng.satellites().size(), eng.recent_messages(100000).size());
    auto vc = eng.voices().calls();
    int good = 0; size_t withpcm = 0;
    for (auto &c : vc) { if (c.error_ratio < 0.5) good++; if (!eng.voices().pcm(c.id).empty()) withpcm++; }
    printf("voice_calls=%zu voice_good=%d voice_with_audio=%zu\n", vc.size(), good, withpcm);
    printf("acars_messages=%zu sbd_packets=%llu ida_assembled=%llu\n",
           eng.acars().messages().size(),
           (unsigned long long)eng.acars().sbd_packets(),
           (unsigned long long)eng.acars().ida_assembled());
    printf("pager_messages=%zu sms_messages=%zu\n",
           eng.pager().messages().size(), eng.acars().sms_messages().size());
    printf("sbd_messages=%llu\n", (unsigned long long)eng.acars().sbd_message_count());
    {
        size_t nal = 0;
        for (const auto &m : eng.acars().sbd_messages()) if (m.has_nal) nal++;
        printf("sbd_nal=%zu paging_events=%zu\n", nal, eng.paging_events(100000).size());
    }
    {
        size_t ibc = 0, itl = 0, itl_ok = 0, assigns = 0;
        for (const auto &m : eng.recent_messages(100000)) {
            if (m.has_ibc) { ibc++; assigns += m.ibc_assignments.size(); }
            if (m.has_itl) { itl++; if (m.itl_ok) itl_ok++; }
        }
        printf("l3_messages=%llu ibc_frames=%zu ibc_assignments=%zu itl_frames=%zu itl_ok=%zu\n",
               (unsigned long long)eng.acars().l3_count(), ibc, assigns, itl, itl_ok);
    }
    {
        size_t voc = 0, vod = 0, voz = 0, vo6 = 0, vda = 0;
        for (const auto &m : eng.recent_messages(100000)) {
            if (m.voice_subtype == "VOC") voc++;
            else if (m.voice_subtype == "VOD") vod++;
            else if (m.voice_subtype == "VOZ") voz++;
            else if (m.voice_subtype == "VO6") vo6++;
            else if (m.voice_subtype == "VDA") vda++;
        }
        printf("voice_types=VOC:%zu VOD:%zu VOZ:%zu VO6:%zu VDA:%zu\n",
               voc, vod, voz, vo6, vda);
    }
    {
        size_t arinc = 0;
        for (const auto &m : eng.acars().messages()) if (m.has_arinc622) arinc++;
        printf("arinc622_messages=%zu\n", arinc);
    }
    print_type_tally(eng);
    eng.stop();
    return 0;
}

static int parse_raw_file(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); return 2; }
    char line[16384];
    std::map<std::string, long> counts;
    long total = 0, err = 0, malformed = 0;
    while (fgets(line, sizeof(line), f)) {
        std::string s(line);
        if (s.rfind("RAW:", 0) != 0) continue;
        std::istringstream iss(s);
        std::vector<std::string> t;
        std::string tok;
        while (iss >> tok) t.push_back(tok);
        if (t.size() < 6) { malformed++; continue; }
        double freq = atof(t[3].c_str());
        int conf = 0;
        for (auto &x : t) if (x.size() && x.back() == '%') conf = atoi(x.c_str());
        std::string bits = t.back();
        if (bits.empty() || bits.find_first_not_of("01") != std::string::npos) { malformed++; continue; }
        irid::RawFrame rf;
        rf.bitstream = bits;
        rf.frequency = freq;
        rf.confidence = conf;
        rf.n_symbols = (int)bits.size() / 2;
        irid::ParsedMessage pm;
        irid::parse_frame(rf, pm);
        counts[pm.type]++;
        total++;
        if (pm.error) err++;
    }
    fclose(f);
    std::vector<std::pair<long, std::string>> v;
    for (auto &kv : counts) v.push_back({kv.second, kv.first});
    std::sort(v.rbegin(), v.rend());
    printf("total=%ld errors=%ld malformed=%ld\n", total, err, malformed);
    for (auto &kv : v) printf("  %-6s %ld\n", kv.second.c_str(), kv.first);
    return 0;
}

namespace irid { extern int irid_debug; }

static int sdr_list_devices() {
    iridi::LiveIqSource sdr(iridi::SdrKind::Sdrplay);
    if (!sdr.available()) { printf("SDRplay support not compiled in\n"); return 1; }
    auto devs = sdr.list_devices();
    printf("sdrplay_devices=%zu\n", devs.size());
    for (auto &d : devs) printf("  [%d] %s  sn=%s\n", d.index, d.name.c_str(), d.serial.c_str());
    return 0;
}

static int hrf_list_devices() {
    iridi::LiveIqSource hrf(iridi::SdrKind::HackRF);
    if (!hrf.available()) { printf("HackRF support not compiled in\n"); return 1; }
    auto devs = hrf.list_devices();
    printf("hackrf_devices=%zu\n", devs.size());
    for (auto &d : devs) printf("  [%d] %s  sn=%s\n", d.index, d.name.c_str(), d.serial.c_str());
    return 0;
}

static int ap_list_devices() {
    iridi::LiveIqSource ap(iridi::SdrKind::Airspy);
    if (!ap.available()) { printf("Airspy support not compiled in\n"); return 1; }
    auto devs = ap.list_devices();
    printf("airspy_devices=%zu\n", devs.size());
    for (auto &d : devs) printf("  [%d] %s  sn=%s\n", d.index, d.name.c_str(), d.serial.c_str());
    return 0;
}

static int rtl_list_devices() {
    iridi::LiveIqSource rtl(iridi::SdrKind::RTL);
    if (!rtl.available()) { printf("RTL-SDR support not compiled in\n"); return 1; }
    auto devs = rtl.list_devices();
    printf("rtlsdr_devices=%zu\n", devs.size());
    for (auto &d : devs) printf("  [%d] %s  sn=%s\n", d.index, d.name.c_str(), d.serial.c_str());
    return 0;
}

static int check_update() {
    iridi::VersionCheck vc;
    vc.start(kUpdateUrl, IRIDISCOPE_VERSION_STRING);
    for (int i = 0; i < 150; ++i) { // up to 15 s
        auto st = vc.state();
        if (st != iridi::VersionCheck::Checking && st != iridi::VersionCheck::Idle) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    static const char *names[] = {"Idle", "Checking",     "UpToDate",
                                  "UpdateAvailable", "Unreleased", "Error"};
    auto st = vc.state();
    printf("state=%s\n", names[(int)st]);
    printf("local=%s\n", IRIDISCOPE_VERSION_STRING);
    printf("latest=%s\n", vc.latest_version().c_str());
    printf("product_url=%s\n", vc.product_url().c_str());
    if (st == iridi::VersionCheck::Error) printf("error=%s\n", vc.error().c_str());
    return 0;
}

static int lookup_registration(const char *reg) {
    iridi::AircraftLookup lk;
    lk.start();
    lk.request_aircraft(reg);
    iridi::AircraftInfo inf;
    for (int i = 0; i < 150; ++i) {
        inf = lk.get_aircraft(reg);
        if (inf.status != iridi::AircraftInfo::None &&
            inf.status != iridi::AircraftInfo::Pending)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    switch (inf.status) {
    case iridi::AircraftInfo::Ok:
        printf("registration=%s\noperator=%s\ntype=%s\nicao_type=%s\nmanufacturer=%s\n"
               "mode_s=%s\ncountry=%s\nphoto=%s\n",
               inf.registration.c_str(), inf.owner.c_str(), inf.type.c_str(),
               inf.icao_type.c_str(), inf.manufacturer.c_str(), inf.mode_s.c_str(),
               inf.country.c_str(), inf.photo_url.c_str());
        break;
    case iridi::AircraftInfo::NotFound:
        printf("not_found=1\n");
        break;
    case iridi::AircraftInfo::Error:
        printf("error=%s\n", inf.error.c_str());
        break;
    default:
        printf("timeout=1\n");
        break;
    }
    lk.stop();
    return 0;
}

static int sms_decode_hex(const char *hex) {
    std::vector<uint8_t> b;
    auto hv = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };
    for (size_t i = 0; hex[i] && hex[i + 1]; i += 2) {
        int hi = hv(hex[i]), lo = hv(hex[i + 1]);
        if (hi < 0 || lo < 0) break;
        b.push_back((uint8_t)((hi << 4) | lo));
    }
    iridi::SmsMessage m;
    bool ok = iridi::decode_sms_tpdu(b, m);
    printf("ok=%d\nfrom=%s\nto=%s\ntext=%s\ndcs=%s\n", ok ? 1 : 0, m.from.c_str(),
           m.to.c_str(), m.text.c_str(), m.dcs.c_str());
    return 0;
}

static int scan_live(iridi::SdrKind kind, double seconds, double rate_hz) {
    iridi::LiveIqSource sdr(kind);
    const char *label = (kind == iridi::SdrKind::HackRF) ? "HackRF"
                      : (kind == iridi::SdrKind::Airspy) ? "Airspy"
                      : (kind == iridi::SdrKind::RTL) ? "RTL-SDR" : "SDRplay";
    if (!sdr.available()) { fprintf(stderr, "%s support not compiled in\n", label); return 2; }
    std::string err;
    if (!sdr.open(0, kDefaultVfoHz, rate_hz, err)) {
        fprintf(stderr, "%s open failed: %s\n", label, err.c_str());
        return 3;
    }
    iridi::Engine eng;
    if (!eng.start(&sdr)) { fprintf(stderr, "engine start failed\n"); sdr.close(); return 4; }
    auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < seconds) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    auto s = eng.stats();
    printf("samples=%llu\nbursts=%llu\nframes_ok=%llu\nframes_fail=%llu\n",
           (unsigned long long)s.samples, (unsigned long long)s.bursts,
           (unsigned long long)s.frames_ok, (unsigned long long)s.frames_fail);
    printf("parsed=%llu\nparse_errors=%llu\nsatellites=%zu\nmessages=%zu\n",
           (unsigned long long)eng.parsed_count(), (unsigned long long)eng.parse_error_count(),
           eng.satellites().size(), eng.recent_messages(100000).size());
    auto vc = eng.voices().calls();
    int good = 0; size_t withpcm = 0;
    for (auto &c : vc) { if (c.error_ratio < 0.5) good++; if (!eng.voices().pcm(c.id).empty()) withpcm++; }
    printf("voice_calls=%zu voice_good=%d voice_with_audio=%zu\n", vc.size(), good, withpcm);
    printf("acars_messages=%zu sbd_packets=%llu ida_assembled=%llu\n",
           eng.acars().messages().size(),
           (unsigned long long)eng.acars().sbd_packets(),
           (unsigned long long)eng.acars().ida_assembled());
    printf("pager_messages=%zu sms_messages=%zu\n",
           eng.pager().messages().size(), eng.acars().sms_messages().size());
    printf("sbd_messages=%llu\n", (unsigned long long)eng.acars().sbd_message_count());
    {
        size_t nal = 0;
        for (const auto &m : eng.acars().sbd_messages()) if (m.has_nal) nal++;
        printf("sbd_nal=%zu paging_events=%zu\n", nal, eng.paging_events(100000).size());
    }
    {
        size_t ibc = 0, itl = 0, itl_ok = 0, assigns = 0;
        for (const auto &m : eng.recent_messages(100000)) {
            if (m.has_ibc) { ibc++; assigns += m.ibc_assignments.size(); }
            if (m.has_itl) { itl++; if (m.itl_ok) itl_ok++; }
        }
        printf("l3_messages=%llu ibc_frames=%zu ibc_assignments=%zu itl_frames=%zu itl_ok=%zu\n",
               (unsigned long long)eng.acars().l3_count(), ibc, assigns, itl, itl_ok);
    }
    {
        size_t voc = 0, vod = 0, voz = 0, vo6 = 0, vda = 0;
        for (const auto &m : eng.recent_messages(100000)) {
            if (m.voice_subtype == "VOC") voc++;
            else if (m.voice_subtype == "VOD") vod++;
            else if (m.voice_subtype == "VOZ") voz++;
            else if (m.voice_subtype == "VO6") vo6++;
            else if (m.voice_subtype == "VDA") vda++;
        }
        printf("voice_types=VOC:%zu VOD:%zu VOZ:%zu VO6:%zu VDA:%zu\n",
               voc, vod, voz, vo6, vda);
    }
    {
        size_t arinc = 0;
        for (const auto &m : eng.acars().messages()) if (m.has_arinc622) arinc++;
        printf("arinc622_messages=%zu\n", arinc);
    }
    eng.stop();
    sdr.close();
    return 0;
}

static int parse_frames_debug(const char *path) {
    irid::irid_debug = 1;
    FILE *f = fopen(path, "r");
    if (!f) return 2;
    char line[16384];
    while (fgets(line, sizeof(line), f)) {
        std::string s(line);
        if (s.rfind("RAW:", 0) != 0) continue;
        std::istringstream iss(s);
        std::vector<std::string> t; std::string tok;
        while (iss >> tok) t.push_back(tok);
        if (t.size() < 6) continue;
        irid::RawFrame rf;
        rf.frequency = atof(t[3].c_str());
        rf.bitstream = t.back();
        rf.n_symbols = (int)rf.bitstream.size() / 2;
        irid::ParsedMessage pm;
        bool ok = irid::parse_frame(rf, pm);
        fprintf(stderr, "=> type=%s ok=%d err=%d\n", pm.type.c_str(), ok, pm.error);
    }
    fclose(f);
    return 0;
}

// When built as a GUI subsystem app there is no console. For the headless
// modes, attach to the invoking terminal so output is visible.
static void attach_console() {
#ifdef _WIN32
    // If stdout is already connected (e.g. redirected to a pipe), keep it.
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    if (h != NULL && h != INVALID_HANDLE_VALUE) return;
    if (AttachConsole(ATTACH_PARENT_PROCESS)) {
        FILE *f;
        freopen_s(&f, "CONOUT$", "w", stdout);
        freopen_s(&f, "CONOUT$", "w", stderr);
        freopen_s(&f, "CONIN$", "r", stdin);
    }
#endif
}

int main(int argc, char **argv) {
    if (getenv("IRIDI_DEBUG")) irid::irid_debug = 1;
    if (argc >= 2) {
        std::string a1 = argv[1];
        if (a1 == "--info" || a1 == "--parse" || a1 == "--frame" || a1 == "--scan" ||
            a1 == "--sdr-list" || a1 == "--sdr-scan" || a1 == "--hrf-list" || a1 == "--hrf-scan" ||
            a1 == "--ap-list" || a1 == "--ap-scan" || a1 == "--rtl-list" || a1 == "--rtl-scan" ||
            a1 == "--check-update" || a1 == "--lookup" || a1 == "--sms" || a1 == "--raw")
            attach_console();
    }
    if (argc >= 2 && std::string(argv[1]) == "--check-update") return check_update();
    if (argc >= 3 && std::string(argv[1]) == "--lookup") return lookup_registration(argv[2]);
    if (argc >= 3 && std::string(argv[1]) == "--sms") return sms_decode_hex(argv[2]);
    if (argc >= 3 && std::string(argv[1]) == "--raw")
        return scan_raw(argv[2], argc >= 4 ? atof(argv[3]) : 60.0,
                        argc >= 5 ? atof(argv[4]) : 0.0);
    if (argc >= 2 && std::string(argv[1]) == "--sdr-list") return sdr_list_devices();
    if (argc >= 2 && std::string(argv[1]) == "--hrf-list") return hrf_list_devices();
    if (argc >= 2 && std::string(argv[1]) == "--ap-list") return ap_list_devices();
    if (argc >= 2 && std::string(argv[1]) == "--rtl-list") return rtl_list_devices();
    if (argc >= 2 && std::string(argv[1]) == "--sdr-scan")
        return scan_live(iridi::SdrKind::Sdrplay, argc >= 3 ? atof(argv[2]) : 10.0,
                         argc >= 4 ? atof(argv[3]) : 2000000.0);
    if (argc >= 2 && std::string(argv[1]) == "--hrf-scan")
        return scan_live(iridi::SdrKind::HackRF, argc >= 3 ? atof(argv[2]) : 10.0,
                         argc >= 4 ? atof(argv[3]) : 10000000.0);
    if (argc >= 2 && std::string(argv[1]) == "--ap-scan")
        return scan_live(iridi::SdrKind::Airspy, argc >= 3 ? atof(argv[2]) : 10.0,
                         argc >= 4 ? atof(argv[3]) : 10000000.0);
    if (argc >= 2 && std::string(argv[1]) == "--rtl-scan")
        return scan_live(iridi::SdrKind::RTL, argc >= 3 ? atof(argv[2]) : 10.0,
                         argc >= 4 ? atof(argv[3]) : 2400000.0);
    if (argc >= 3 && std::string(argv[1]) == "--info") return print_file_info(argv[2]);
    if (argc >= 3 && std::string(argv[1]) == "--parse") return parse_raw_file(argv[2]);
    if (argc >= 3 && std::string(argv[1]) == "--frame") return parse_frames_debug(argv[2]);
    if (argc >= 3 && std::string(argv[1]) == "--scan") return scan_file(argv[2], argc >= 4 ? atof(argv[3]) : 12.0);

    if (!glfwInit()) { fprintf(stderr, "Failed to initialize GLFW\n"); return 1; }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    g_window = glfwCreateWindow(1600, 900, "IridiScope", nullptr, nullptr);
    if (!g_window) { fprintf(stderr, "Failed to create GLFW window\n"); glfwTerminate(); return 1; }
    glfwMakeContextCurrent(g_window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    // Seed the window layout from the bundled default on first run, so the
    // shipped docking arrangement is the default.
    const char *ini_path = "iridiscope.ini";
    bool force_default = false;
    if (GetFileAttributesA(ini_path) == INVALID_FILE_ATTRIBUTES) {
        std::string def = std::string(IRIDISCOPE_ASSETS) + "/default_layout.ini";
        if (!CopyFileA(def.c_str(), ini_path, FALSE)) force_default = true;
    }
    io.IniFilename = ini_path;
    bool dock_built = false;
    apply_style();

    ImGui_ImplGlfw_InitForOpenGL(g_window, true);
    ImGui_ImplOpenGL3_Init("#version 150");

    add_log("IridiScope - Iridium signal analysis workstation");
    add_log("OpenGL: %s", glGetString(GL_VERSION) ? (const char *)glGetString(GL_VERSION) : "?");

    g_map_tex = load_map_texture(g_map_w, g_map_h);

    g_version_check.start(kUpdateUrl, IRIDISCOPE_VERSION_STRING);
    g_lookup.start();

    int file_idx = 1;
    if (argc > 1 && std::string(argv[1]) == "--autoplay") { g_autoplay = true; file_idx = 2; }
    if (argc > file_idx) open_iq_file(argv[file_idx]);

    while (!glfwWindowShouldClose(g_window)) {
        glfwPollEvents();
        if (g_request_open) { g_request_open = false; open_iq_file(open_file_dialog()); }
        if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_O)) g_request_open = true;

        // Notify of an available update by appending to the window title once.
        if (g_version_check.state() == iridi::VersionCheck::UpdateAvailable) {
            static bool titled = false;
            if (!titled) {
                std::string t = "IridiScope - Update available: v" +
                                g_version_check.latest_version();
                glfwSetWindowTitle(g_window, t.c_str());
                titled = true;
            }
        }

        // Pull new spectrum into the waterfall and auto-range.
        uint64_t seq = g_engine.spectrum_seq();
        if (seq != g_spec_seq) {
            g_spec_seq = seq;
            std::vector<float> inst = g_engine.spectrum_inst();
            int N = (int)inst.size();
            if (N > 0) {
                if (g_autoscale) {
                    float floorDb = g_engine.spectrum_floor();
                    float peakDb = g_engine.spectrum_peak();
                    float tgtMin = floorDb - 6.0f;
                    float tgtMax = std::max(peakDb + 12.0f, floorDb + 20.0f);
                    g_db_min = 0.85f * g_db_min + 0.15f * tgtMin;
                    g_db_max = 0.85f * g_db_max + 0.15f * tgtMax;
                    if (g_db_max < g_db_min + 5.0f) g_db_max = g_db_min + 5.0f;
                }
                g_waterfall.addRow(inst.data(), N, g_db_min, g_db_max);
            }
        }

        update_autoplay();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiID dock_id = ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());
        if (force_default && !dock_built) { build_default_dock_layout(dock_id); dock_built = true; }

        // Place the Pager and SMS tabs in the same dock group as Messages.
        static bool grouped_msgs = false;
        if (!grouped_msgs) {
            ImGuiWindow *msgw = ImGui::FindWindowByName("Messages");
            if (msgw && msgw->DockId) {
                if (ImGuiWindow *w = ImGui::FindWindowByName("Pager"))
                    ImGui::SetWindowDock(w, msgw->DockId, ImGuiCond_Always);
                if (ImGuiWindow *w = ImGui::FindWindowByName("SMS"))
                    ImGui::SetWindowDock(w, msgw->DockId, ImGuiCond_Always);
                if (ImGuiWindow *w = ImGui::FindWindowByName("SBD"))
                    ImGui::SetWindowDock(w, msgw->DockId, ImGuiCond_Always);
                if (ImGuiWindow *w = ImGui::FindWindowByName("Signaling"))
                    ImGui::SetWindowDock(w, msgw->DockId, ImGuiCond_Always);
                if (ImGuiWindow *w = ImGui::FindWindowByName("Broadcast"))
                    ImGui::SetWindowDock(w, msgw->DockId, ImGuiCond_Always);
                if (ImGuiWindow *w = ImGui::FindWindowByName("ITL"))
                    ImGui::SetWindowDock(w, msgw->DockId, ImGuiCond_Always);
                grouped_msgs = true;
            }
        }

        // Place Paging in the same dock group as Satellites.
        static bool grouped_sats = false;
        if (!grouped_sats) {
            ImGuiWindow *sw = ImGui::FindWindowByName("Satellites");
            if (sw && sw->DockId) {
                if (ImGuiWindow *w = ImGui::FindWindowByName("Paging"))
                    ImGui::SetWindowDock(w, sw->DockId, ImGuiCond_Always);
                grouped_sats = true;
            }
        }

        // Make Messages the active tab in its dock group when the app opens.
        static bool messages_focused = false;
        if (!messages_focused) {
            if (ImGui::FindWindowByName("Messages") != nullptr) {
                ImGui::SetWindowFocus("Messages");
                messages_focused = true;
            }
        }

        draw_menu_bar();
        draw_map_window();
        draw_playback_window();
        draw_stats_window();
        draw_spectrum_window();
        draw_waterfall_window();
        draw_bursts_window();
        draw_messages_window();
        draw_satellites_window();
        draw_paging_window();
        draw_voice_window();
        draw_acars_window();
        draw_acars_detail_window();
        draw_aircraft_info_window();
        draw_pager_window();
        draw_sms_window();
        draw_sbd_window();
        draw_signaling_window();
        draw_broadcast_window();
        draw_itl_window();
        draw_source_window();
        draw_about_window();

        ImGui::Render();
        int fbw, fbh;
        glfwGetFramebufferSize(g_window, &fbw, &fbh);
        glViewport(0, 0, fbw, fbh);
        glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(g_window);
    }

    g_engine.stop();
    g_sdr.close();
    g_hackrf.close();
    g_airspy.close();
    if (g_sdr_scan_thread.joinable()) g_sdr_scan_thread.join();
    if (g_hrf_scan_thread.joinable()) g_hrf_scan_thread.join();
    if (g_ap_scan_thread.joinable()) g_ap_scan_thread.join();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    if (g_map_tex) glDeleteTextures(1, &g_map_tex);
    glfwDestroyWindow(g_window);
    glfwTerminate();
    return 0;
}
