// IridiScope - Iridium signal analysis workstation.
//
// Milestone 2: the vendored DSP pipeline runs over the IQ file and drives a
// live spectrum (ImPlot), scrolling waterfall, and burst statistics.
#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#include <commdlg.h>
#include <shellapi.h>
#else
// macOS / Linux replacements for the Win32 APIs used below.
#include <climits>
#include <spawn.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
extern char **environ;
// MSVC/MinGW localtime_s(tm*, time_t*) -> POSIX localtime_r(time_t*, tm*)
#define localtime_s(tmp, tp) localtime_r((tp), (tmp))
#endif

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <cctype>
#include <cstring>
#include <random>
#include <deque>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <GLFW/glfw3.h>
#if defined(__APPLE__)
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include "core/priority.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "implot.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

#include "core/file_source.h"
#include "core/engine.h"
#include "iridium/pipeline_c.h"
#include "store/msg_db.h"
#include "voice/announce.h"
#include <filesystem>
#include "parser/iridium_parser.h"
#include "acars/aircraft_lookup.h"
#include "sdr/live_iq_source.h"
#include "output/gsmtap.h"
#include "ui/waterfall.h"
#include "ui/map_tiles.h"
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
int g_audio_call_id = -1;   // voice call loaded in g_audio

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
bool g_voice_hide_failed = true;  // hide finished calls above the error threshold
bool g_voice_hide_ann = true;     // hide calls that matched a network announcement
bool g_voice_sort_ann = true;     // file announcement calls under recordings/announcements/
float g_voice_max_err = 0.50f;    // Err threshold (fraction of failed frames)
bool g_voice_hide_short = true;   // hide finished calls shorter than g_voice_min_s
float g_voice_min_s = 1.0f;       // ... seconds
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

// Iridium band limits, used for the band plan and as the default tuning range.
const double kBandLoMhz = 1616.0;
const double kBandHiMhz = 1626.5;

// Default VFO: center of the Iridium band. The VFO itself is not band-limited;
// live SDRs can be tuned anywhere their hardware supports.
const double kDefaultVfoHz = (kBandLoMhz + kBandHiMhz) * 0.5 * 1e6;
double g_vfo_hz = kDefaultVfoHz;

// SDRplay sample rates.
const double kSdrplayRates[] = {2.0e6, 3.0e6, 4.0e6, 5.0e6, 6.0e6, 7.0e6, 8.0e6, 9.0e6, 10.0e6};
const char *kSdrplayRateLabels[] = {"2.0", "3.0", "4.0", "5.0", "6.0", "7.0", "8.0", "9.0", "10.0"};
const int kNumSdrplayRates = (int)(sizeof(kSdrplayRates) / sizeof(kSdrplayRates[0]));
int g_sdr_rate_idx = kNumSdrplayRates - 1; // 10 MHz
int g_sdr_device_idx = 0;
int g_sdr_antenna = 0;
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
int g_ap_rate_idx = 3; // 10 MHz (index into the generic list)
// Selected rate in Hz. When the chosen device reports its own rates (Airspy
// Mini: 6 / 3 Msps, R2: 10 / 2.5 Msps) only those are offered.
double g_ap_rate_hz = 0.0; // 0 = not chosen yet -> device's highest rate
bool g_ap_unofficial10 = false; // Airspy Mini: offer the unadvertised 10 Msps mode
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
uint64_t g_log_version = 0;      // bumped on every add_log (lets the Log pane skip copies)
bool g_reset_layout = false;     // View > Reset layout
int g_queue_budget_idx = 2;      // burst queue memory: 1/2/4/8 GB (default 4) - playback
bool g_protect_radio = true;     // ABOVE_NORMAL process class, see core/priority.h
bool g_reject_wide = true;       // drop wideband (>=150 kHz) carriers before decoding
int g_backlog_idx = 2;           // live decoder backlog: 1/2/4/8 s (default 4)
static const double kBacklogS[] = {1.0, 2.0, 4.0, 8.0};
int g_layout_focus_frames = 0;   // frames until default tab selection is applied
bool g_show_map = true;
bool g_show_source = true;

// GSMTAP output and decoded-data export.
char g_gsmtap_host[64] = "127.0.0.1";
int g_gsmtap_port = 4729;
bool g_gsmtap_on = false;
std::string g_gsmtap_status;
std::string g_save_status;
bool g_show_spectrum = true;
bool g_show_waterfall = true;
bool g_show_bursts = true;
bool g_show_messages = true;
bool g_show_log = true;
// Log categories ("[TAG] " after the time): Status, Alerts, Radio, Voice,
// Handoffs, Other (untagged). Shown/hidden in the Log window; saved.
bool g_log_cat[6] = {true, true, true, true, false, true};
int g_status_interval_idx = 1;   // status summary every 30 s / 1 min / 5 min
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

std::chrono::steady_clock::time_point g_gain_changed_at;   // last gain change (noise floor alerts)

// Current gain setting of the running SDR, for the log.
std::string gain_desc() {
    char b[160];
    switch (g_source_kind) {
    case 1: snprintf(b, sizeof(b), "SDRplay %s, gain reduction %.1f dB", g_sdr_agc ? "AGC" : "manual", (double)g_sdr_gr); break;
    case 2: snprintf(b, sizeof(b), "HackRF LNA %d, VGA %d, amp %s", g_hrf_lna, g_hrf_vga, g_hrf_amp ? "on" : "off"); break;
    case 3:
        if (g_ap_gain_mode == 0) snprintf(b, sizeof(b), "Airspy sensitivity %d", g_ap_sense);
        else if (g_ap_gain_mode == 1) snprintf(b, sizeof(b), "Airspy linearity %d", g_ap_linear);
        else snprintf(b, sizeof(b), "Airspy LNA %d%s, mixer %d%s, VGA %d", g_ap_lna, g_ap_lna_agc ? " (AGC)" : "",
                      g_ap_mixer, g_ap_mixer_agc ? " (AGC)" : "", g_ap_vga);
        break;
    case 4: snprintf(b, sizeof(b), "RTL-SDR %s", g_rtl_agc ? "AGC" : (std::to_string(g_rtl_gain) + " dB").c_str()); break;
    default: snprintf(b, sizeof(b), "file playback"); break;
    }
    return b;
}

// Category of a log line from its "[TAG] " (after the "HH:MM:SS  " time):
// 0 Status, 1 Alerts, 2 Radio, 3 Voice, 4 Handoffs, 5 Other.
int log_category(const std::string &l) {
    if (l.size() < 12 || l[10] != '[') return 5;
    const char *t = l.c_str() + 11;
    if (!strncmp(t, "STATUS]", 7) || !strncmp(t, "TRAFFIC]", 8)) return 0;
    if (!strncmp(t, "ALERT]", 6)) return 1;
    if (!strncmp(t, "RADIO]", 6)) return 2;
    if (!strncmp(t, "VOICE]", 6)) return 3;
    if (!strncmp(t, "HANDOFF]", 8)) return 4;
    return 5;
}

std::mutex g_log_mtx;
void add_log(const char *fmt, ...) {
    char msg[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    // Local wall-clock time prefix.
    char buf[1100];
    {
        const auto now = std::chrono::system_clock::now();
        const std::time_t tt = std::chrono::system_clock::to_time_t(now);
        std::tm lt{};
#ifdef _WIN32
        localtime_s(&lt, &tt);
#else
        localtime_r(&tt, &lt);
#endif
        snprintf(buf, sizeof(buf), "%02d:%02d:%02d  %s", lt.tm_hour, lt.tm_min, lt.tm_sec, msg);
    }
    std::lock_guard<std::mutex> lk(g_log_mtx);
    g_log.emplace_back(buf);
    if (g_log.size() > 5000) g_log.erase(g_log.begin(), g_log.begin() + 1000);
    g_log_version++;
    // The GUI build has no console, so keep a file copy next to the exe
    // (the working directory, like iridiscope.ini and recordings/).
    static FILE *lf = nullptr;
    static bool tried = false;
    if (!tried) {
        tried = true;
        lf = fopen("iridiscope.log", "a");
        if (lf) fprintf(lf, "---- IridiScope session start ----\n");
    }
    if (lf) { fprintf(lf, "%s\n", buf); fflush(lf); }
    fprintf(stdout, "%s\n", buf);
}

void open_url(const char *url) {
#ifdef _WIN32
    ShellExecuteA(nullptr, "open", url, nullptr, nullptr, SW_SHOWNORMAL);
#else
    if (!url || strncmp(url, "http", 4) != 0) return; // only web links
#if defined(__APPLE__)
    const char *opener = "open";
#else
    const char *opener = "xdg-open";
#endif
    // posix_spawnp: no shell involved, so the URL can't inject commands.
    char *args[] = {(char *)opener, (char *)url, nullptr};
    pid_t pid;
    if (posix_spawnp(&pid, opener, nullptr, nullptr, args, environ) == 0) {
        std::thread([pid] { int st; waitpid(pid, &st, 0); }).detach();
    }
#endif
}

// ---- map texture loading ---------------------------------------------------
#ifdef _WIN32
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
#else // !_WIN32
// Directory containing the executable (resolving symlinks).
static std::string exe_directory() {
    char buf[PATH_MAX] = {0};
#if defined(__APPLE__)
    uint32_t sz = sizeof(buf);
    if (_NSGetExecutablePath(buf, &sz) != 0) return ".";
#else
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return ".";
    buf[n] = 0;
#endif
    char real[PATH_MAX] = {0};
    std::string p = realpath(buf, real) ? std::string(real) : std::string(buf);
    size_t pos = p.find_last_of('/');
    return pos == std::string::npos ? "." : p.substr(0, pos);
}

static bool file_exists(const std::string &path) {
    struct stat st;
    return stat(path.c_str(), &st) == 0;
}

static bool read_file_bytes(const std::string &path, std::vector<unsigned char> &out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    f.seekg(0, std::ios::end);
    std::streamoff size = f.tellg();
    if (size <= 0 || size > 256LL * 1024 * 1024) return false;
    f.seekg(0, std::ios::beg);
    out.resize((size_t)size);
    if (!f.read(reinterpret_cast<char *>(out.data()), size)) { out.clear(); return false; }
    return true;
}

// Where the assets folder may live: next to the executable, inside a macOS
// .app bundle (Contents/Resources/assets), or the source tree at build time.
static std::vector<std::string> asset_dirs() {
    std::string dir = exe_directory();
    return {dir + "/assets", dir + "/../Resources/assets", IRIDISCOPE_ASSETS};
}

// Outside Windows there are no embedded resources; assets are files.
static bool read_resource_bytes(int, std::vector<unsigned char> &) { return false; }

// A Finder/Dock launch starts with cwd "/" (read-only). IridiScope writes its
// layout (iridiscope.ini) and voice recordings to the working directory, so
// move to a per-user data folder in that case.
static void ensure_writable_workdir() {
    char cwd[PATH_MAX] = {0};
    bool bad = !getcwd(cwd, sizeof(cwd)) || strcmp(cwd, "/") == 0 ||
               access(cwd, W_OK) != 0;
    if (!bad) return;
    const char *home = getenv("HOME");
    if (!home || !*home) return;
#if defined(__APPLE__)
    std::string base = std::string(home) + "/Library/Application Support";
#else
    const char *xdg = getenv("XDG_DATA_HOME");
    std::string base = (xdg && *xdg) ? std::string(xdg) : std::string(home) + "/.local/share";
    mkdir(base.c_str(), 0755);
#endif
    std::string dir = base + "/IridiScope";
    mkdir(dir.c_str(), 0755);
    if (chdir(dir.c_str()) == 0) fprintf(stderr, "Working directory: %s\n", dir.c_str());
}
#endif // _WIN32

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
    // The map can be zoomed, so keep the full image resolution (the bundled
    // map is 5400x2700, ~58 MB as RGBA) when the GPU allows it.
    int limit = std::min(max_tex, 8192);
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
#ifdef _WIN32
    std::wstring dir = exe_directory();
    std::wstring candidates[] = {
        dir + L"\\assets\\worldmap.jpg",
        dir + L"\\worldmap.jpg",
    };
    for (const auto &p : candidates) {
        std::vector<unsigned char> bytes;
        if (!read_file_wide(p, bytes)) continue;
#else
    std::vector<std::string> candidates;
    for (const auto &d : asset_dirs()) candidates.push_back(d + "/worldmap.jpg");
    candidates.push_back(exe_directory() + "/worldmap.jpg");
    for (const auto &p : candidates) {
        std::vector<unsigned char> bytes;
        if (!read_file_bytes(p, bytes)) continue;
#endif
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

#ifdef _WIN32
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

std::string save_file_dialog() {
    char filename[MAX_PATH];
    SYSTEMTIME st;
    GetLocalTime(&st);
    snprintf(filename, sizeof(filename), "iridiscope-%04d%02d%02d-%02d%02d%02d.csv",
             st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
    OPENFILENAMEA ofn;
    ZeroMemory(&ofn, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "CSV files (*.csv)\0*.csv\0All files (*.*)\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrDefExt = "csv";
    ofn.Flags = OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
    if (GetSaveFileNameA(&ofn)) return std::string(filename);
    return std::string();
}
#else // !_WIN32
// Run a fixed command line and return its first output line ("" on cancel).
// Only constant strings and digits are ever placed in the command.
static std::string run_dialog_command(const std::string &cmd) {
    FILE *p = popen(cmd.c_str(), "r");
    if (!p) return std::string();
    std::string out;
    char buf[4096];
    while (fgets(buf, sizeof(buf), p)) out += buf;
    int rc = pclose(p);
    if (rc != 0) return std::string();
    size_t nl = out.find('\n');
    if (nl != std::string::npos) out.resize(nl);
    return out;
}

static bool have_program(const char *name) {
    std::string cmd = std::string("command -v ") + name + " >/dev/null 2>&1";
    return system(cmd.c_str()) == 0;
}

std::string open_file_dialog() {
#if defined(__APPLE__)
    // Native macOS "choose file" panel via AppleScript.
    return run_dialog_command(
        "osascript -e 'activate' -e 'POSIX path of (choose file with prompt "
        "\"Open IQ recording (.wav .cf32 .ci16 .ci8 .cfile .raw)\")' 2>/dev/null");
#else
    if (have_program("zenity"))
        return run_dialog_command(
            "zenity --file-selection --title='Open IQ recording' "
            "--file-filter='IQ files | *.wav *.cf32 *.ci16 *.ci8 *.cfile *.raw' "
            "--file-filter='All files | *' 2>/dev/null");
    if (have_program("kdialog"))
        return run_dialog_command(
            "kdialog --getopenfilename . '*.wav *.cf32 *.ci16 *.ci8 *.cfile *.raw' 2>/dev/null");
    add_log("No file dialog available: install zenity or kdialog, "
            "or pass the IQ file on the command line");
    return std::string();
#endif
}

std::string save_file_dialog() {
    char name[64];
    time_t now = time(nullptr);
    struct tm tmv;
    localtime_r(&now, &tmv);
    snprintf(name, sizeof(name), "iridiscope-%04d%02d%02d-%02d%02d%02d.csv",
             tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
    std::string path;
#if defined(__APPLE__)
    path = run_dialog_command(
        std::string("osascript -e 'activate' -e 'POSIX path of (choose file name "
                    "with prompt \"Export CSV\" default name \"") + name +
        "\")' 2>/dev/null");
#else
    if (have_program("zenity"))
        path = run_dialog_command(std::string("zenity --file-selection --save "
                                              "--confirm-overwrite --filename=") +
                                  name + " 2>/dev/null");
    else if (have_program("kdialog"))
        path = run_dialog_command(std::string("kdialog --getsavefilename ") + name +
                                  " 2>/dev/null");
    else
        path = name; // no dialog: save in the working directory
#endif
    if (!path.empty() && path.size() >= 4 &&
        path.compare(path.size() - 4, 4, ".csv") != 0)
        path += ".csv";
    return path;
}
#endif // _WIN32

static std::string csv_escape(const std::string &s) {
    if (s.find_first_of(",\"\r\n") == std::string::npos) return s;
    std::string o = "\"";
    for (char c : s) {
        if (c == '"') o += "\"\"";
        else o += c;
    }
    o += "\"";
    return o;
}

static std::string utc_iso(uint64_t ns) {
    time_t t = (time_t)(ns / 1000000000ULL);
    struct tm *tmv = gmtime(&t);
    if (!tmv) return "";
    char b[32];
    strftime(b, sizeof(b), "%Y-%m-%dT%H:%M:%SZ", tmv);
    return b;
}

static bool save_decoded_csv(const std::string &path) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f << "type,time_utc,freq_hz,direction,summary,detail\n";
    size_t n = 0;
    auto row = [&](const char *type, uint64_t ts, double freq, int ul,
                   const std::string &summary, const std::string &detail) {
        f << type << ',' << utc_iso(ts) << ',' << (long long)freq << ','
          << (ul ? "UL" : "DL") << ',' << csv_escape(summary) << ','
          << csv_escape(detail) << '\n';
        n++;
    };

    for (const auto &m : g_engine.acars().messages()) {
        std::string detail = m.text;
        if (!m.arinc622_text.empty()) detail += "\n" + m.arinc622_text;
        row("ACARS", m.timestamp_ns, m.frequency, m.ul,
            m.reg + " " + m.flight + " " + m.label, detail);
    }
    for (const auto &m : g_engine.acars().sbd_messages())
        row("SBD", m.timestamp_ns, m.frequency, m.ul, m.info,
            m.text.empty() ? m.payload_hex : m.text);
    for (const auto &m : g_engine.acars().sms_messages())
        row("SMS", m.timestamp_ns, m.frequency, m.ul,
            m.from + " -> " + m.to, m.text);
    for (const auto &m : g_engine.pager().messages())
        row("PAGER", m.timestamp_ns, m.frequency, 0,
            "RIC " + std::to_string(m.ric) + " fmt " + std::to_string(m.fmt), m.text);
    for (const auto &m : g_engine.acars().l3_messages())
        row("L3", m.timestamp_ns, m.frequency, m.ul,
            m.proto + " " + m.type, m.summary);
    for (const auto &m : g_engine.recent_messages(100000))
        row(m.type.c_str(), m.timestamp_ns, m.frequency, m.uplink, m.pretty,
            m.has_ida ? m.ida_text : "");

    f.flush();
    g_save_status = "Saved " + std::to_string(n) + " records to " + path;
    return true;
}

void apply_spectrum_settings() {
    g_engine.set_fft_size(kFftSizes[g_fft_idx]);
    g_engine.set_avg_alpha(g_avg_alpha);
    g_engine.set_dc_block(g_dc_block);
}

// Recording start time from an IQ file name (Unix ns, 0 = unknown), so
// playback shows when things were actually received:
//   SDR# / Airspy / HDSDR / IridiScope: ..._YYYYMMDD_HHMMSSZ_...  (UTC)
//   SDR++: ..._HH-MM-SS_DD-MM-YYYY...                               (local)
static uint64_t recording_time_from_name(const std::string &path) {
    std::string name = path;
    const size_t slash = name.find_last_of("/\\");
    if (slash != std::string::npos) name = name.substr(slash + 1);
    auto digits = [&](size_t at, size_t n) {
        if (at + n > name.size()) return false;
        for (size_t i = 0; i < n; i++) if (!std::isdigit((unsigned char)name[at + i])) return false;
        return true;
    };
    auto num = [&](size_t at, size_t n) { return std::atoi(name.substr(at, n).c_str()); };
    for (size_t i = 0; i + 16 <= name.size(); i++) {
        // YYYYMMDD_HHMMSSZ
        if (digits(i, 8) && name[i + 8] == '_' && digits(i + 9, 6) && name[i + 15] == 'Z') {
            std::tm t{};
            t.tm_year = num(i, 4) - 1900; t.tm_mon = num(i + 4, 2) - 1; t.tm_mday = num(i + 6, 2);
            t.tm_hour = num(i + 9, 2); t.tm_min = num(i + 11, 2); t.tm_sec = num(i + 13, 2);
            if (t.tm_mon < 0 || t.tm_mon > 11 || t.tm_mday < 1 || t.tm_mday > 31 || t.tm_hour > 23) continue;
#ifdef _WIN32
            const std::time_t tt = _mkgmtime(&t);
#else
            const std::time_t tt = timegm(&t);
#endif
            if (tt > 0) return (uint64_t)tt * 1000000000ULL;
        }
        // HH-MM-SS_DD-MM-YYYY
        if (i + 19 <= name.size() && digits(i, 2) && name[i + 2] == '-' && digits(i + 3, 2) && name[i + 5] == '-' &&
            digits(i + 6, 2) && name[i + 8] == '_' && digits(i + 9, 2) && name[i + 11] == '-' &&
            digits(i + 12, 2) && name[i + 14] == '-' && digits(i + 15, 4)) {
            std::tm t{};
            t.tm_hour = num(i, 2); t.tm_min = num(i + 3, 2); t.tm_sec = num(i + 6, 2);
            t.tm_mday = num(i + 9, 2); t.tm_mon = num(i + 12, 2) - 1; t.tm_year = num(i + 15, 4) - 1900;
            t.tm_isdst = -1;
            if (t.tm_mon < 0 || t.tm_mon > 11 || t.tm_mday < 1 || t.tm_mday > 31 || t.tm_hour > 23) continue;
            const std::time_t tt = std::mktime(&t);
            if (tt > 0) return (uint64_t)tt * 1000000000ULL;
        }
    }
    return 0;
}

void start_engine() {
    iridi::IqSource *src = active_source();
    if (!src->is_open()) { add_log("ERROR: no source open"); return; }
    g_autoplay_last_id = -1;
    g_autoplay_queue.clear();
    {
        static const double gb[] = {1.0, 2.0, 4.0, 8.0};
        iridi::Engine::set_queue_budget(gb[std::clamp(g_queue_budget_idx, 0, 3)] * 1.0e9);
        iridi::Engine::set_live_backlog(g_source_kind != 0 ? kBacklogS[std::clamp(g_backlog_idx, 0, 3)] : 0.0);
    }
    if (g_source_kind == 0) {
        // Playback: stamp frames with the recording's own time when the file
        // name carries it (offset by where playback starts in the file).
        const uint64_t t0 = recording_time_from_name(g_file.info().path);
        if (t0 && g_file.sample_rate() > 0) {
            g_engine.set_start_time_ns(t0 + (uint64_t)((double)g_file.tell_sample() * 1e9 / g_file.sample_rate()));
            static std::string logged;
            if (logged != g_file.info().path) {
                logged = g_file.info().path;
                const std::time_t tt = (std::time_t)(t0 / 1000000000ULL);
                std::tm u{};
#ifdef _WIN32
                gmtime_s(&u, &tt);
#else
                gmtime_r(&tt, &u);
#endif
                char b[40]; std::strftime(b, sizeof(b), "%Y-%m-%d %H:%M:%S UTC", &u);
                add_log("Playback times taken from the file name: recording started %s", b);
            }
        }
    }
    if (g_engine.start(src)) {
        apply_spectrum_settings();
        if (g_source_kind != 0)
            add_log("[RADIO] Engine started: %d decoder worker threads (+1 detector thread), %.0f s decoder backlog",
                    g_engine.worker_threads(), kBacklogS[std::clamp(g_backlog_idx, 0, 3)]);
        else
            add_log("[RADIO] Engine started: %d decoder worker threads (+1 detector thread), burst queue %d (%s budget)",
                    g_engine.worker_threads(), g_engine.queue_capacity(),
                    g_queue_budget_idx == 0 ? "1 GB" : g_queue_budget_idx == 1 ? "2 GB" : g_queue_budget_idx == 2 ? "4 GB" : "8 GB");
    }
    else add_log("[ALERT] ERROR: failed to start engine");
}

// Retune a live SDR without changing the current FFT/waterfall view.
void retune_vfo(double hz) {
    if (hz <= 0.0) return;
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

// Keep the shared FFT/waterfall view inside the source's captured passband.
// For live SDRs, if the view leaves the passband, retune the radio to the
// centre of the view so both windows continue to show real data.
void sync_view(double fs_mhz, bool live) {
    double &lo = g_view_lo_mhz, &hi = g_view_hi_mhz;
    if (fs_mhz <= 0) return;

    if (hi <= lo) {
        double fc = active_source()->center_frequency() / 1e6;
        lo = fc - fs_mhz * 0.5;
        hi = fc + fs_mhz * 0.5;
    }

    double span = hi - lo;
    double max_span = fs_mhz;
    if (!live) max_span = std::min(kBandHiMhz - kBandLoMhz, fs_mhz);
    if (span > max_span) {
        double c = (lo + hi) * 0.5;
        lo = c - max_span * 0.5;
        hi = c + max_span * 0.5;
        span = max_span;
    }
    if (span < 0.05) {
        double c = (lo + hi) * 0.5;
        lo = c - 0.025;
        hi = c + 0.025;
    }

    // IQ files are Iridium captures; keep their view inside the band plan.
    if (!live) {
        if (lo < kBandLoMhz) { hi += kBandLoMhz - lo; lo = kBandLoMhz; }
        if (hi > kBandHiMhz) { lo -= hi - kBandHiMhz; hi = kBandHiMhz; }
        lo = std::max(lo, kBandLoMhz);
        hi = std::min(hi, kBandHiMhz);
    }

    double fc = active_source()->center_frequency() / 1e6;
    double cap_lo = fc - fs_mhz * 0.5, cap_hi = fc + fs_mhz * 0.5;

    if (live && (lo < cap_lo || hi > cap_hi)) {
        retune_vfo((lo + hi) * 0.5 * 1e6);
        fc = active_source()->center_frequency() / 1e6;
        cap_lo = fc - fs_mhz * 0.5;
        cap_hi = fc + fs_mhz * 0.5;
    }

    // Keep the view inside the captured passband. Live SDRs were retuned
    // above if they needed it; IQ files cannot be retuned.
    lo = std::max(lo, cap_lo);
    hi = std::min(hi, cap_hi);
    if (hi <= lo) { lo = cap_lo; hi = cap_hi; }
}

// Initialise the shared view after a source or sample-rate change.
double ensure_view() {
    double fs = (active_source()->sample_rate() ? active_source()->sample_rate() : 0) / 1e6;
    bool live = g_source_kind != 0;
    if (g_view_reset || g_view_hi_mhz <= g_view_lo_mhz) {
        double fc = active_source()->center_frequency() / 1e6;
        if (fs > 0) {
            g_view_lo_mhz = fc - fs * 0.5;
            g_view_hi_mhz = fc + fs * 0.5;
        } else {
            g_view_lo_mhz = fc - 5.0;
            g_view_hi_mhz = fc + 5.0;
        }
        if (!live) {
            g_view_lo_mhz = std::max(g_view_lo_mhz, kBandLoMhz);
            g_view_hi_mhz = std::min(g_view_hi_mhz, kBandHiMhz);
            if (g_view_hi_mhz <= g_view_lo_mhz) {
                g_view_lo_mhz = kBandLoMhz;
                g_view_hi_mhz = kBandHiMhz;
            }
        }
        g_view_reset = false;
        g_view_dirty = true;
        sync_view(fs, live);
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
    add_log("[RADIO] Sample rate -> %.2f Msps", hz / 1e6);
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
    g_sdr.set_antenna(g_sdr_antenna);
    std::string err;
    if (!g_sdr.open(g_sdr_device_idx, g_vfo_hz, kSdrplayRates[g_sdr_rate_idx], err)) {
        add_log("ERROR: %s", err.c_str());
        return;
    }
    std::string port_label;
    {
        std::lock_guard<std::mutex> lk(g_sdr_dev_mtx);
        if (g_sdr_device_idx >= 0 && g_sdr_device_idx < (int)g_sdr_devices.size()) {
            const auto &ports = g_sdr_devices[g_sdr_device_idx].ports;
            if (g_sdr_antenna >= 0 && g_sdr_antenna < (int)ports.size())
                port_label = ports[g_sdr_antenna];
        }
    }
    add_log("[RADIO] SDRplay started: %s @ %.6f MHz, %.1f Msps%s%s", g_sdr.device_name().c_str(),
            g_vfo_hz / 1e6, kSdrplayRates[g_sdr_rate_idx] / 1e6,
            port_label.empty() ? "" : ", port ", port_label.c_str());
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
    add_log("[RADIO] HackRF started: %s @ %.6f MHz, %.1f Msps (LNA %d, VGA %d, amp %s)",
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
    g_airspy.set_allow_unofficial_rates(g_ap_unofficial10);
    std::string err;
    const double want = g_ap_rate_hz > 0 ? g_ap_rate_hz : kAirspyRates[g_ap_rate_idx];
    if (!g_airspy.open(g_ap_device_idx, g_vfo_hz, want, err)) {
        add_log("ERROR: %s", err.c_str());
        return;
    }
    const double got = (double)g_airspy.sample_rate();
    if (std::fabs(got - want) > 1.0)
        add_log("[RADIO] Airspy: %.1f Msps not supported by this device, using %.1f Msps", want / 1e6, got / 1e6);
    g_ap_rate_hz = got;
    add_log("[RADIO] Airspy started: %s @ %.6f MHz, %.1f Msps", g_airspy.device_name().c_str(),
            g_vfo_hz / 1e6, got / 1e6);
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
    add_log("[RADIO] RTL-SDR started: %s @ %.6f MHz, %.2f Msps", g_rtlsdr.device_name().c_str(),
            g_vfo_hz / 1e6, kRtlRates[g_rtl_rate_idx] / 1e6);
    g_waterfall.clear();
    g_spec_seq = 0;
    g_freq_mhz.clear();
    g_view_reset = true;
    start_engine();
}

// ---- panels ---------------------------------------------------------------

bool g_msgdb_on = true;   // message database (File menu)

// "Sort existing recordings now": moves announcement WAVs out of recordings/
// on a background thread; the result is logged when it finishes.
std::atomic<bool> g_sort_running{false};
std::atomic<int> g_sort_done{-1}, g_sort_moved{0}, g_sort_checked{0};
void start_sort_existing() {
    if (g_sort_running.exchange(true)) return;
    g_sort_moved = 0; g_sort_checked = 0; g_sort_done = -1;
    add_log("Sorting existing recordings: checking WAVs in recordings/ for network announcements...");
    std::thread([] {
        iridi_apply_thread_priority(IRIDI_PRIO_BACKGROUND);   // never competes with decoding
        namespace fs = std::filesystem;
        std::error_code ec;
        std::vector<fs::path> files;
        for (auto &e : fs::directory_iterator("recordings", ec))
            if (e.is_regular_file() && e.path().extension() == ".wav") files.push_back(e.path());
        for (const auto &p : files) {
            std::vector<int16_t> pcm;
            if (FILE *f = std::fopen(p.string().c_str(), "rb")) {
                std::fseek(f, 0, SEEK_END);
                const long n = std::ftell(f);
                if (n > 44) {
                    std::fseek(f, 44, SEEK_SET);
                    pcm.resize((size_t)(n - 44) / 2);
                    pcm.resize(std::fread(pcm.data(), 2, pcm.size(), f));
                }
                std::fclose(f);
            }
            g_sort_checked++;
            const std::string ann = iridi::AnnouncementMatcher::instance().classify(pcm);
            if (ann.empty()) continue;
            const fs::path dir = fs::path("recordings") / "announcements" / ann;
            fs::create_directories(dir, ec);
            fs::rename(p, dir / p.filename(), ec);
            if (!ec) g_sort_moved++;
        }
        g_sort_done = (int)files.size();
        g_sort_running = false;
    }).detach();
}

void draw_menu_bar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Open IQ file...", "Ctrl+O")) g_request_open = true;
            if (ImGui::MenuItem("Close")) { g_engine.stop(); g_file.close(); g_sdr.close(); g_hackrf.close(); g_airspy.close(); g_rtlsdr.close(); g_waterfall.clear(); add_log("[RADIO] Closed source"); }
            ImGui::Separator();
            {
                bool on = g_msgdb_on;
                if (ImGui::MenuItem("Log messages to database", nullptr, &on)) {
                    g_msgdb_on = on;
                    iridi::MsgDb::instance().set_enabled(on);
                }
                if (ImGui::IsItemHovered()) {
                    const std::string pth = iridi::MsgDb::instance().path();
                    ImGui::SetTooltip("ACARS, SMS, SBD, pager, voice calls and ring alerts go into an SQLite\n"
                                      "file per session in logs\\ (open with DB Browser for SQLite etc.).\n%s%s",
                                      pth.empty() ? "No messages logged yet this session." : "This session: ",
                                      pth.c_str());
                }
                {
                    bool so = g_voice_sort_ann;
                    if (ImGui::MenuItem("File announcement calls separately", nullptr, &so)) {
                        g_voice_sort_ann = so;
                        g_engine.voices().set_sort_announcements(so);
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("New calls that match a network announcement are saved under\n"
                                          "recordings\\announcements\\<name>\\ instead of recordings\\.");
                    if (ImGui::MenuItem("Sort existing recordings now", nullptr, false, !g_sort_running.load()))
                        start_sort_existing();
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Checks every WAV directly in recordings\\ and moves the ones that are a\n"
                                          "network announcement into recordings\\announcements\\<name>\\.\n"
                                          "Everything else is left untouched.");
                }
                if (ImGui::MenuItem("Open logs folder")) {
#ifdef _WIN32
                    CreateDirectoryA("logs", nullptr);
                    ShellExecuteA(nullptr, "open", "logs", nullptr, nullptr, SW_SHOWNORMAL);
#endif
                }
            }
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
            ImGui::MenuItem("Bursts", nullptr, &g_show_bursts);
            ImGui::MenuItem("Messages", nullptr, &g_show_messages);
            ImGui::MenuItem("Log", nullptr, &g_show_log);
            ImGui::Separator();
            if (ImGui::MenuItem("Reset layout")) {
                // Re-open every docked pane, then rebuild the default arrangement.
                g_show_map = g_show_messages = g_show_log = g_show_sats = g_show_voice = true;
                g_reset_layout = true;
            }
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
            ImGui::MenuItem("Control", nullptr, &g_show_source);
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

// Ageing for satellites and spot beams. Positions are only as fresh as the
// last ring alert: a satellite moves ~7 km/s, so stale markers are faded and
// then hidden. "Now" is the later of the newest frame time and the wall clock
// (frame times are wall-clock based; file playback can run ahead of it).
float g_sat_hide_after_s = 120.0f;
bool g_sat_show_stale = false;
static uint64_t ring_now_ns() {
    const uint64_t wall = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    return std::max(wall, g_engine.last_timestamp());
}
static double age_s(uint64_t last_seen_ns, uint64_t now_ns) {
    if (last_seen_ns == 0 || last_seen_ns >= now_ns) return 0.0;
    return (double)(now_ns - last_seen_ns) / 1e9;
}
static bool is_stale(double age) { return !g_sat_show_stale && age > g_sat_hide_after_s; }
// 1.0 while fresh, fading to 0.25 as the entry approaches the hide age.
static float fresh_alpha(double age) {
    const double fade_start = std::min(30.0, g_sat_hide_after_s * 0.25);
    if (age <= fade_start) return 1.0f;
    const double t = std::min(1.0, (age - fade_start) / std::max(1.0, g_sat_hide_after_s - fade_start));
    return (float)(1.0 - 0.75 * t);
}
static ImU32 with_alpha(ImU32 c, float a) {
    const ImU32 al = (ImU32)std::clamp((int)(((c >> IM_COL32_A_SHIFT) & 0xFF) * a), 0, 255);
    return (c & ~IM_COL32_A_MASK) | (al << IM_COL32_A_SHIFT);
}

// Map view: centre (deg) and scale (degrees per screen pixel, same on both
// axes for the equirectangular image). g_map_dpp <= 0 means "fit the world".
double g_map_clon = -80.0, g_map_clat = 45.0;
double g_map_dpp = 0.0;
bool g_map_autofit_done = false; // first received positions zoom the view once

static double map_fit_world_dpp(const ImVec2 &size) {
    return std::max(360.0 / std::max(1.0f, size.x), 180.0 / std::max(1.0f, size.y));
}

static void map_clamp_view(const ImVec2 &size) {
    const double world = map_fit_world_dpp(size);
    const double min_dpp = 0.004; // ~ 250 px per degree: well past image detail
    if (g_map_dpp <= 0.0 || g_map_dpp > world) g_map_dpp = world;
    if (g_map_dpp < min_dpp) g_map_dpp = min_dpp;
    // Keep the centre on the map; when zoomed out further than the world,
    // centre it.
    const double half_w = size.x * 0.5 * g_map_dpp, half_h = size.y * 0.5 * g_map_dpp;
    if (half_w >= 180.0) g_map_clon = 0.0;
    else g_map_clon = std::clamp(g_map_clon, -180.0 + half_w, 180.0 - half_w);
    if (half_h >= 90.0) g_map_clat = 0.0;
    else g_map_clat = std::clamp(g_map_clat, -90.0 + half_h, 90.0 - half_h);
}

// Zoom to the received satellites and beams with some margin.
static bool map_fit_to_data(const ImVec2 &size) {
    double lo_lat = 90, hi_lat = -90, lo_lon = 180, hi_lon = -180;
    int n = 0;
    auto add = [&](double lat, double lon) {
        if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) return;
        lo_lat = std::min(lo_lat, lat); hi_lat = std::max(hi_lat, lat);
        lo_lon = std::min(lo_lon, lon); hi_lon = std::max(hi_lon, lon);
        n++;
    };
    const uint64_t now = ring_now_ns();
    for (const auto &s : g_engine.satellites())
        if (s.has_pos && !is_stale(age_s(s.last_seen_ns, now))) add(s.lat, s.lon);
    for (const auto &g : g_engine.ground_beams())
        if (!is_stale(age_s(g.last_seen_ns, now))) add(g.lat, g.lon);
    if (n == 0) return false;
    // At least ~20 x 12 degrees so a single beam isn't absurdly zoomed.
    double w = std::max(20.0, (hi_lon - lo_lon) * 1.3);
    double h = std::max(12.0, (hi_lat - lo_lat) * 1.3);
    g_map_clon = (lo_lon + hi_lon) * 0.5;
    g_map_clat = (lo_lat + hi_lat) * 0.5;
    g_map_dpp = std::max(w / std::max(1.0f, size.x), h / std::max(1.0f, size.y));
    map_clamp_view(size);
    return true;
}

// Online satellite imagery over the built-in world image (see ui/map_tiles.h).
iridi::MapTiles g_tiles;
bool g_map_online = true;
// Night mode: NASA Black Marble (Earth at night, city lights) from NASA GIBS,
// over a darkened day map (which is what shows if those tiles can't load).
iridi::MapTiles g_tiles_night;
bool g_map_night = false;
const ImU32 kNightTint = IM_COL32(70, 82, 120, 255);

// Draw the Esri tiles covering the view, in the map's lat/lon (equirectangular)
// projection: each Web Mercator tile is drawn as horizontal strips so its
// latitude scale is right. Tiles still loading show a coarser cached one.
template <typename LL2S>
static bool draw_map_tiles(iridi::MapTiles &tiles, ImDrawList *dl, LL2S ll_to_screen, double dpp,
                           double base_px_per_deg, double vlat_top, double vlon_left, double vlat_bot,
                           double vlon_right, int zmax = 18, ImU32 tint = IM_COL32_WHITE) {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kMaxLat = 85.0511287798;
    // Only once zoomed in past the built-in image's own resolution (a
    // base_px_per_deg of 0 means always: the night layer has no base image).
    if (dpp <= 0.0 || (base_px_per_deg > 0 && 1.0 / dpp <= base_px_per_deg)) return false;
    int z = (int)std::ceil(std::log2(360.0 / (256.0 * dpp)));
    z = std::clamp(z, std::min(3, zmax), zmax);
    const double top = std::min(vlat_top, kMaxLat), bot = std::max(vlat_bot, -kMaxLat);
    if (top <= bot) return false;
    auto merc_y = [&](double lat, int zz) {
        const double r = lat * kPi / 180.0;
        return (1.0 - std::asinh(std::tan(r)) / kPi) / 2.0 * (double)(1 << zz);
    };
    auto tile_lat = [&](double ty, int zz) {
        return std::atan(std::sinh(kPi * (1.0 - 2.0 * ty / (double)(1 << zz)))) * 180.0 / kPi;
    };
    int x0, x1, y0, y1;
    for (;;) {
        const int n = 1 << z;
        x0 = std::clamp((int)std::floor((std::max(vlon_left, -180.0) + 180.0) / 360.0 * n), 0, n - 1);
        x1 = std::clamp((int)std::floor((std::min(vlon_right, 180.0) + 180.0) / 360.0 * n), 0, n - 1);
        y0 = std::clamp((int)std::floor(merc_y(top, z)), 0, n - 1);
        y1 = std::clamp((int)std::floor(merc_y(bot, z)), 0, n - 1);
        if ((x1 - x0 + 1) * (y1 - y0 + 1) <= 96 || z <= 3) break;
        z--;
    }
    tiles.begin_frame();
    const int n = 1 << z;
    bool any = false;
    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            unsigned int tex = tiles.get(z, x, y);
            int k = 0;
            // While loading, use the nearest coarser tile already in memory.
            while (!tex && k < 4 && z - k - 1 >= 0) { k++; tex = tiles.peek(z - k, x >> k, y >> k); }
            if (!tex) continue;
            const double f = (double)(1 << k);           // target tiles per texture tile
            const double u0 = (x - (double)((x >> k) << k)) / f, u1 = u0 + 1.0 / f;
            const double lon_l = (double)x / n * 360.0 - 180.0, lon_r = (double)(x + 1) / n * 360.0 - 180.0;
            const int strips = z < 6 ? 16 : 6;
            for (int i = 0; i < strips; i++) {
                const double ty0 = y + (double)i / strips, ty1 = y + (double)(i + 1) / strips;
                const double v0 = (ty0 - (double)((y >> k) << k)) / f, v1 = (ty1 - (double)((y >> k) << k)) / f;
                const ImVec2 p0 = ll_to_screen(tile_lat(ty0, z), lon_l);
                const ImVec2 p1 = ll_to_screen(tile_lat(ty1, z), lon_r);
                dl->AddImage((ImTextureID)(uintptr_t)tex, p0, p1, ImVec2((float)u0, (float)v0),
                             ImVec2((float)u1, (float)v1), tint);
            }
            any = true;
        }
    }
    return any || base_px_per_deg > 0;
}

void draw_map_window() {
    if (!g_show_map) return;
    ImGui::SetNextWindowSize(ImVec2(900, 500), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Map", &g_show_map)) {
        // Toolbar
        bool fit_now = false;
        if (ImGui::SmallButton("Fit to data")) fit_now = true;
        ImGui::SameLine();
        if (ImGui::SmallButton("World")) g_map_dpp = 0.0;
        ImGui::SameLine();
        if (ImGui::SmallButton("North America")) {
            g_map_clon = -95.0; g_map_clat = 45.0; g_map_dpp = -1.0; // sized below
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110);
        ImGui::SliderFloat("Hide after", &g_sat_hide_after_s, 30.0f, 900.0f, "%.0f s");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Satellites and spot beams not heard for this long are hidden\n"
                              "(map and Satellites list). Markers fade as they age.");
        ImGui::SameLine();
        ImGui::Checkbox("Show stale", &g_sat_show_stale);
        ImGui::SameLine();
        ImGui::Checkbox("Satellite imagery", &g_map_online);
        ImGui::SameLine();
        ImGui::Checkbox("Night", &g_map_night);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Night map: NASA Black Marble (the Earth at night, city lights) when\n"
                              "online (saved in mapcache/ like the day tiles), over a darkened\n"
                              "day map.");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Sharper imagery as you zoom in (Esri World Imagery, needs internet).\n"
                              "Tiles you've viewed are kept in mapcache/ and work offline later;\n"
                              "elsewhere the built-in world image is shown.");
        ImGui::SameLine();
        ImGui::TextDisabled("Wheel: zoom   Drag: pan   Double-click: world");

        ImVec2 avail = ImGui::GetContentRegionAvail();
        if (avail.x < 16 || avail.y < 16) { ImGui::End(); return; }
        const ImVec2 pos = ImGui::GetCursorScreenPos();
        const ImVec2 size = avail;
        if (g_map_dpp < 0.0) g_map_dpp = std::max(70.0 / size.x, 40.0 / size.y); // NA preset

        // Interaction surface
        ImGui::InvisibleButton("##mapview", size,
                               ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
        const bool hovered = ImGui::IsItemHovered();
        ImGuiIO &io = ImGui::GetIO();
        auto screen_to_ll = [&](const ImVec2 &p, double &lat, double &lon) {
            lon = g_map_clon + (p.x - (pos.x + size.x * 0.5)) * g_map_dpp;
            lat = g_map_clat - (p.y - (pos.y + size.y * 0.5)) * g_map_dpp;
        };
        map_clamp_view(size);
        if (hovered && io.MouseWheel != 0.0f) {
            // Zoom about the cursor so the point under it stays put.
            double mlat, mlon;
            screen_to_ll(io.MousePos, mlat, mlon);
            g_map_dpp *= std::pow(1.25, -io.MouseWheel);
            map_clamp_view(size);
            double nlat, nlon;
            screen_to_ll(io.MousePos, nlat, nlon);
            g_map_clon += mlon - nlon;
            g_map_clat += mlat - nlat;
            map_clamp_view(size);
        }
        if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left, 0.0f)) {
            g_map_clon -= io.MouseDelta.x * g_map_dpp;
            g_map_clat += io.MouseDelta.y * g_map_dpp;
            map_clamp_view(size);
        }
        if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) g_map_dpp = 0.0;
        if (fit_now) map_fit_to_data(size);
        if (!g_map_autofit_done && map_fit_to_data(size)) g_map_autofit_done = true;
        map_clamp_view(size);

        auto ll_to_screen = [&](double lat, double lon) {
            return ImVec2((float)(pos.x + size.x * 0.5 + (lon - g_map_clon) / g_map_dpp),
                          (float)(pos.y + size.y * 0.5 - (lat - g_map_clat) / g_map_dpp));
        };

        ImDrawList *dl = ImGui::GetWindowDrawList();
        const ImVec2 pmax(pos.x + size.x, pos.y + size.y);
        dl->PushClipRect(pos, pmax, true);
        dl->AddRectFilled(pos, pmax, IM_COL32(12, 18, 26, 255));
        const ImVec2 w0 = ll_to_screen(90.0, -180.0), w1 = ll_to_screen(-90.0, 180.0);
        const ImU32 day_tint = g_map_night ? kNightTint : IM_COL32_WHITE;
        if (g_map_tex) dl->AddImage((ImTextureID)(uintptr_t)g_map_tex, w0, w1, ImVec2(0, 0), ImVec2(1, 1), day_tint);
        else dl->AddRectFilled(w0, w1, IM_COL32(20, 30, 40, 255));
        bool tiles_drawn = false, night_drawn = false;
        if (g_map_online) {
            double t_top, t_left, t_bot, t_right;
            screen_to_ll(pos, t_top, t_left);
            screen_to_ll(pmax, t_bot, t_right);
            const double base_ppd = g_map_tex && g_map_w > 0 ? g_map_w / 360.0 : 0.0;
            tiles_drawn = draw_map_tiles(g_tiles, dl, ll_to_screen, g_map_dpp, base_ppd, t_top, t_left, t_bot,
                                         t_right, 18, day_tint);
            // Black Marble goes up to zoom 8 (~500 m); closer in it's enlarged.
            if (g_map_night)
                night_drawn = draw_map_tiles(g_tiles_night, dl, ll_to_screen, g_map_dpp, 0.0, t_top, t_left,
                                             t_bot, t_right, 8);
        }

        // Grid: finer and labelled as you zoom in.
        const double span = size.x * g_map_dpp;
        const int step = span > 150 ? 30 : span > 60 ? 10 : span > 25 ? 5 : span > 8 ? 2 : 1;
        const ImU32 grid = IM_COL32(255, 255, 255, 40), glabel = IM_COL32(230, 230, 230, 170);
        double vlat_top, vlon_left, vlat_bot, vlon_right;
        screen_to_ll(pos, vlat_top, vlon_left);
        screen_to_ll(pmax, vlat_bot, vlon_right);
        char gl[16];
        for (int lon = -180; lon <= 180; lon += step) {
            if (lon < vlon_left - step || lon > vlon_right + step) continue;
            ImVec2 a = ll_to_screen(0, lon);
            dl->AddLine(ImVec2(a.x, std::max(pos.y, w0.y)), ImVec2(a.x, std::min(pmax.y, w1.y)), grid);
            if (step < 30 && a.x > pos.x + 4 && a.x < pmax.x - 30) {
                snprintf(gl, sizeof(gl), "%d%c", std::abs(lon), lon < 0 ? 'W' : (lon > 0 ? 'E' : ' '));
                dl->AddText(ImVec2(a.x + 2, pos.y + 2), glabel, gl);
            }
        }
        for (int lat = -90; lat <= 90; lat += step) {
            if (lat < vlat_bot - step || lat > vlat_top + step) continue;
            ImVec2 a = ll_to_screen(lat, 0);
            dl->AddLine(ImVec2(std::max(pos.x, w0.x), a.y), ImVec2(std::min(pmax.x, w1.x), a.y), grid);
            if (step < 30 && a.y > pos.y + 16 && a.y < pmax.y - 24) {
                snprintf(gl, sizeof(gl), "%d%c", std::abs(lat), lat < 0 ? 'S' : (lat > 0 ? 'N' : ' '));
                dl->AddText(ImVec2(pos.x + 2, a.y + 1), glabel, gl);
            }
        }

        // Markers, plus the one nearest the cursor for a tooltip.
        const ImVec2 mouse = io.MousePos;
        float best_d2 = 10.0f * 10.0f;
        std::string tip;
        auto consider = [&](const ImVec2 &pt, const std::string &text) {
            const float dx = pt.x - mouse.x, dy = pt.y - mouse.y, d2 = dx * dx + dy * dy;
            if (hovered && d2 < best_d2) { best_d2 = d2; tip = text; }
        };
        char tb[160];
        // Ground spot-beam centers (ring alerts with alt < 100 km).
        const uint64_t now_ns = ring_now_ns();
        for (const auto &g : g_engine.ground_beams()) {
            if (g.lat < -90.0 || g.lat > 90.0) continue;
            const double age = age_s(g.last_seen_ns, now_ns);
            if (is_stale(age)) continue;
            const float fa = fresh_alpha(age);
            const ImVec2 pt = ll_to_screen(g.lat, g.lon);
            dl->AddCircleFilled(pt, 3.0f, with_alpha(IM_COL32(0, 210, 255, 200), fa));
            if (g.pages > 0)
                dl->AddCircle(pt, 7.0f, with_alpha(IM_COL32(255, 60, 220, 230), fa), 0, 1.5f);
            snprintf(tb, sizeof(tb), "Beam %d of S%03d\n%+.3f, %+.3f\nRing alerts: %d   Pages: %d\nLast heard %.0f s ago",
                     g.beam, g.sat, g.lat, g.lon, g.count, g.pages, age);
            consider(pt, tb);
        }
        // Satellites (from decoded ring alerts).
        for (const auto &s : g_engine.satellites()) {
            if (!s.has_pos || s.lat < -90.0 || s.lat > 90.0) continue;
            const double age = age_s(s.last_seen_ns, now_ns);
            if (is_stale(age)) continue;
            const float fa = fresh_alpha(age);
            const ImVec2 pt = ll_to_screen(s.lat, s.lon);
            dl->AddCircleFilled(pt, 4.5f, with_alpha(IM_COL32(255, 170, 0, 235), fa));
            dl->AddCircle(pt, 8.0f, with_alpha(IM_COL32(255, 255, 0, 150), fa), 0, 1.5f);
            char lbl[16];
            snprintf(lbl, sizeof(lbl), "S%03d", s.id);
            dl->AddText(ImVec2(pt.x + 8, pt.y - 8), with_alpha(IM_COL32(255, 255, 140, 255), fa), lbl);
            snprintf(tb, sizeof(tb), "Satellite %03d\n%+.3f, %+.3f   %d km\nBeam %d   Hits %d\nLast heard %.0f s ago",
                     s.id, s.lat, s.lon, s.alt_km, s.beam, s.count, age);
            consider(pt, tb);
        }

        if (!g_map_tex) {
            const char *msg = "Map image unavailable";
            ImVec2 ts = ImGui::CalcTextSize(msg);
            dl->AddText(ImVec2(pos.x + (size.x - ts.x) * 0.5f, pos.y + (size.y - ts.y) * 0.5f),
                        IM_COL32(235, 235, 235, 240), msg);
            const char *hint = "Place worldmap.jpg in an assets folder next to IridiScope.exe";
            ImVec2 hs = ImGui::CalcTextSize(hint);
            dl->AddText(ImVec2(pos.x + (size.x - hs.x) * 0.5f, pos.y + (size.y - hs.y) * 0.5f + 22),
                        IM_COL32(160, 185, 205, 220), hint);
        }
        // Imagery credit (required by Esri) while tiles are in use.
        if (tiles_drawn || night_drawn) {
            char cred[160];
            if (night_drawn)
                snprintf(cred, sizeof(cred), "Night imagery: NASA Black Marble (GIBS)%s",
                         g_tiles_night.offline() ? "  (offline: showing saved tiles)" : "");
            else
                snprintf(cred, sizeof(cred), "Imagery: Esri, Maxar, Earthstar Geographics%s",
                         g_tiles.offline() ? "  (offline: showing saved tiles)" : "");
            const ImVec2 cs = ImGui::CalcTextSize(cred);
            dl->AddRectFilled(ImVec2(pmax.x - cs.x - 12, pmax.y - 22), ImVec2(pmax.x - 4, pmax.y - 4),
                              IM_COL32(0, 0, 0, 140));
            dl->AddText(ImVec2(pmax.x - cs.x - 8, pmax.y - 20), IM_COL32(255, 255, 255, 190), cred);
        }
        // Legend and cursor position
        const char *legend = "orange = satellite   cyan = spot beam   magenta = paging";
        dl->AddRectFilled(ImVec2(pos.x + 4, pmax.y - 22),
                          ImVec2(pos.x + 12 + ImGui::CalcTextSize(legend).x, pmax.y - 4),
                          IM_COL32(0, 0, 0, 140));
        dl->AddText(ImVec2(pos.x + 8, pmax.y - 20), IM_COL32(255, 255, 255, 200), legend);
        if (hovered) {
            double mlat, mlon;
            screen_to_ll(mouse, mlat, mlon);
            if (mlat >= -90 && mlat <= 90 && mlon >= -180 && mlon <= 180) {
                char cp[48];
                snprintf(cp, sizeof(cp), "%+.3f, %+.3f", mlat, mlon);
                const ImVec2 cs = ImGui::CalcTextSize(cp);
                dl->AddRectFilled(ImVec2(pmax.x - cs.x - 12, pmax.y - 22), ImVec2(pmax.x - 4, pmax.y - 4),
                                  IM_COL32(0, 0, 0, 140));
                dl->AddText(ImVec2(pmax.x - cs.x - 8, pmax.y - 20), IM_COL32(255, 255, 255, 200), cp);
            }
        }
        dl->PopClipRect();
        if (!tip.empty()) ImGui::SetTooltip("%s", tip.c_str());
    }
    ImGui::End();
}

void draw_satellites_window() {
    if (!g_show_sats) return;
    ImGui::SetNextWindowSize(ImVec2(420, 240), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Satellites", &g_show_sats)) {
        auto all = g_engine.satellites();
        const uint64_t now_ns = ring_now_ns();
        std::vector<iridi::Satellite> sats;
        for (const auto &s : all)
            if (!is_stale(age_s(s.last_seen_ns, now_ns))) sats.push_back(s);
        std::sort(sats.begin(), sats.end(), [](const iridi::Satellite &a, const iridi::Satellite &b) {
            return a.last_seen_ns > b.last_seen_ns;
        });
        ImGui::TextDisabled("%zu satellites heard in the last %.0f s (%zu total)", sats.size(),
                            g_sat_hide_after_s, all.size());
        ImGui::SameLine();
        ImGui::Checkbox("Show stale", &g_sat_show_stale);
        if (ImGui::BeginTable("sats", 7,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                ImVec2(0, 0))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Sat");
            ImGui::TableSetupColumn("Lat");
            ImGui::TableSetupColumn("Lon");
            ImGui::TableSetupColumn("Alt km");
            ImGui::TableSetupColumn("Beam");
            ImGui::TableSetupColumn("Hits");
            ImGui::TableSetupColumn("Age s");
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
                ImGui::TableSetColumnIndex(6); ImGui::Text("%.0f", age_s(s.last_seen_ns, now_ns));
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void draw_log_window() {
    if (!g_show_log) return;
    ImGui::SetNextWindowSize(ImVec2(820, 240), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Log", &g_show_log)) {
        static bool paused = false;
        static bool follow = true;
        static std::vector<std::string> frozen;
        static char filter[96] = "";
        static std::vector<std::string> cache;
        static uint64_t cache_ver = ~0ull;
        if (!paused) {
            std::lock_guard<std::mutex> lk(g_log_mtx);
            if (cache_ver != g_log_version) { cache = g_log; cache_ver = g_log_version; }
        }
        const std::vector<std::string> &lines = paused ? frozen : cache;
        if (ImGui::Button(paused ? "Resume" : "Pause")) {
            paused = !paused;
            if (paused) { std::lock_guard<std::mutex> lk(g_log_mtx); frozen = g_log; }
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(220);
        ImGui::InputTextWithHint("##logfilter", "Filter (e.g. Airspy, 1626.27)", filter, sizeof(filter));
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear filter")) filter[0] = 0;
        std::string needle(filter);
        for (auto &ch : needle) ch = (char)std::tolower((unsigned char)ch);
        std::vector<const std::string *> shown;
        for (const auto &l : lines) {
            if (!needle.empty()) {
                std::string hay = l;
                for (auto &ch : hay) ch = (char)std::tolower((unsigned char)ch);
                if (hay.find(needle) == std::string::npos) continue;
            }
            if (!g_log_cat[log_category(l)]) continue;
            shown.push_back(&l);
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Copy shown")) {
            std::string out;
            for (const auto *l : shown) { out += *l; out += "\n"; }
            ImGui::SetClipboardText(out.c_str());
        }
        ImGui::SameLine();
        ImGui::Checkbox("Auto-scroll", &follow);
        {
            static const char *names[6] = {"Status", "Alerts", "Radio", "Voice", "Handoffs", "Other"};
            static const char *tips[6] = {
                "Summary every interval: frames, bursts, CPU, memory, drops, noise floor, traffic.",
                "Problems: sample drops, skipped bursts, CPU starved, frame rate or noise floor jumps,\nWAV write failures.",
                "Radio and settings: start/stop, gain changes, sweeps, IQ recording, priority/wideband toggles.",
                "Every saved call: length, speech, frequency, handoffs, announcement, file.",
                "Handoff tracking detail on voice calls (followed / not followed).",
                "Everything else (startup, map, sorting, ...)."};
            ImGui::TextDisabled("Show:");
            for (int i = 0; i < 6; i++) {
                ImGui::SameLine();
                ImGui::Checkbox(names[i], &g_log_cat[i]);
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tips[i]);
            }
            ImGui::SameLine();
            static const char *iv[3] = {"30 s", "1 min", "5 min"};
            ImGui::SetNextItemWidth(70);
            ImGui::Combo("Status every", &g_status_interval_idx, iv, 3);
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%zu lines%s   (also saved to iridiscope.log)", shown.size(), paused ? " (paused)" : "");
        ImGui::BeginChild("##logscroll", ImVec2(0, 0), true, ImGuiWindowFlags_HorizontalScrollbar);
        ImGuiListClipper clip;
        clip.Begin((int)shown.size());
        while (clip.Step())
            for (int i = clip.DisplayStart; i < clip.DisplayEnd; i++) {
                const std::string &l = *shown[(size_t)i];
                const int cat = log_category(l);
                const bool warn = cat == 1 || l.find("ERROR") != std::string::npos || l.find("not followed") != std::string::npos;
                if (warn) ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f), "%s", l.c_str());
                else if (cat == 0) ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "%s", l.c_str());
                else if (cat == 3) ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f), "%s", l.c_str());
                else if (cat == 4) ImGui::TextColored(ImVec4(0.65f, 0.65f, 0.7f, 1.0f), "%s", l.c_str());
                else ImGui::TextUnformatted(l.c_str());
            }
        if (follow && !paused && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 20.0f)
            ImGui::SetScrollHereY(1.0f);
        ImGui::EndChild();
    }
    ImGui::End();
}

void draw_messages_window() {
    if (!g_show_messages) return;
    ImGui::SetNextWindowSize(ImVec2(820, 280), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Messages", &g_show_messages)) {
        // Pause freezes a snapshot; the filter matches type or detail text
        // (case-insensitive) across the last 4000 messages.
        static bool paused = false;
        static std::vector<irid::ParsedMessage> frozen;
        static char filter[96] = "";
        if (ImGui::Button(paused ? "Resume" : "Pause")) {
            paused = !paused;
            if (paused) frozen = g_engine.recent_messages(4000);
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(220);
        ImGui::InputTextWithHint("##msgfilter", "Filter (e.g. handoff_resp, IRA, VOC)", filter, sizeof(filter));
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear")) filter[0] = 0;
        static std::vector<irid::ParsedMessage> live;
        static auto last_fetch = std::chrono::steady_clock::time_point{};
        if (!paused && std::chrono::steady_clock::now() - last_fetch > std::chrono::milliseconds(250)) {
            live = g_engine.recent_messages(4000);
            last_fetch = std::chrono::steady_clock::now();
        }
        const std::vector<irid::ParsedMessage> &src = paused ? frozen : live;
        std::string needle(filter);
        for (auto &ch : needle) ch = (char)std::tolower((unsigned char)ch);
        std::vector<const irid::ParsedMessage *> shown;
        for (auto it = src.rbegin(); it != src.rend(); ++it) {
            if (!needle.empty()) {
                std::string hay = it->type + " " + it->pretty;
                for (auto &ch : hay) ch = (char)std::tolower((unsigned char)ch);
                if (hay.find(needle) == std::string::npos) continue;
            }
            shown.push_back(&*it);
            if (needle.empty() && shown.size() >= 1000) break;
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Copy shown")) {
            std::string out;
            for (size_t i = 0; i < shown.size() && i < 500; i++) {
                char fb[32]; snprintf(fb, sizeof(fb), "%.0f", shown[i]->frequency);
                out += shown[i]->type + "\t" + fb + "\t" + shown[i]->pretty + "\n";
            }
            ImGui::SetClipboardText(out.c_str());
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Copy the listed rows (up to 500) to the clipboard");
        ImGui::SameLine();
        ImGui::TextDisabled("%zu shown%s", shown.size(), paused ? "  (paused)" : "");
        if (ImGui::BeginTable("msgs", 4,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                ImVec2(0, 0))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 52);
            ImGui::TableSetupColumn("Freq (Hz)", ImGuiTableColumnFlags_WidthFixed, 96);
            ImGui::TableSetupColumn("Sat", ImGuiTableColumnFlags_WidthFixed, 40);
            ImGui::TableSetupColumn("Detail", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();
            ImGuiListClipper clip;
            clip.Begin((int)shown.size());
            while (clip.Step())
            for (int row = clip.DisplayStart; row < clip.DisplayEnd; ++row) {
                const auto &m = *shown[(size_t)row];
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

void draw_playback_contents() {
    {
        static const char *labels[] = {"1 s", "2 s", "4 s", "8 s"};
        ImGui::SetNextItemWidth(90);
        ImGui::Combo("Decoder backlog (live)", &g_backlog_idx, labels, 4);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("How far the decoders may fall behind the radio during a traffic peak.\n"
                              "Beyond that the oldest bursts are skipped; the radio itself is never\n"
                              "interrupted. Memory: about 80 MB per second at 10 Msps.\n"
                              "Applies the next time the radio starts.");
        static const char *glabels[] = {"1 GB", "2 GB", "4 GB", "8 GB"};
        ImGui::SetNextItemWidth(90);
        ImGui::Combo("Playback queue memory", &g_queue_budget_idx, glabels, 4);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Playing a recording: memory for bursts waiting for the decoders\n"
                              "(nothing is skipped; playback slows down instead). Applies on Play.");
    }
    if (ImGui::Checkbox("Protect radio from other programs", &g_protect_radio)) {
        add_log("[RADIO] Protect radio from other programs: %s", g_protect_radio ? "on" : "off");
        iridi_set_boost(g_protect_radio);
        iridi_apply_thread_priority(IRIDI_PRIO_NORMAL);   // this (UI) thread
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Runs IridiScope at 'above normal' Windows priority so the threads that move\n"
                          "samples from the radio (USB, conversion, burst detector) outrank any normal\n"
                          "program, e.g. 7-Zip or a game. Decoders, UI and saving are lowered by the\n"
                          "same step, so they still share the CPU equally with other programs.\n"
                          "Takes effect immediately.");
    if (ImGui::Checkbox("Ignore wideband carriers", &g_reject_wide)) {
        ir_set_reject_wide(g_reject_wide);
        add_log("[RADIO] Ignore wideband carriers: %s", g_reject_wide ? "on" : "off");
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Drop detections that sit inside a signal 150 kHz or wider (about 200 kHz\n"
                          "carriers seen around 1619-1622 and 1625-1626 MHz). The classic Iridium\n"
                          "decoders can't read them; each one otherwise reaches the decoders as\n"
                          "several failed bursts. When on, bursts/s counts only what is decoded.\n"
                          "Takes effect immediately.");
    {
        bool uw = irid::irid_uw_ec.load();
        if (ImGui::Checkbox("Sync tolerance (recover damaged frames)", &uw)) irid::irid_uw_ec.store(uw);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Accept frames whose 12-symbol sync pattern (unique word) has up to 3\n"
                              "wrong symbols, as iridium-toolkit's --uw-ec does. Recovers frames that\n"
                              "would otherwise be 'Access code missing'. Chance of a random burst\n"
                              "passing is ~0.08%%, and later checks reject most of those.\n"
                              "Applies immediately.");
    }
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

// Rolling 10-second rates for Stats: good (error-free) frames per second and
// the share of radio samples dropped because processing fell behind.
struct RateSample { std::chrono::steady_clock::time_point t; uint64_t good, rx, dropped, drv; };
static std::deque<RateSample> g_rate_hist;
static float g_good_hist[120];   // last 60 s of good frames/s, 0.5 s apart (sparkline)
static float g_burst_hist[120];  // same for bursts/s
static int g_good_hist_n = 0;
static double g_peak_good = 0, g_peak_bursts = 0;           // session peaks (10 s averages)
static std::chrono::system_clock::time_point g_peak_good_t;

// Car-style tachometer: 270 degree dial, ticks and numbers, a damped needle,
// a peak-hold marker and a big digital readout. The scale only ever steps
// up (so the dial doesn't jump around) when the value or peak outgrows it.
static void draw_gauge(const char *id, const char *label, double value, double peak, float &scale,
                       float width, ImU32 accent) {
    static const float steps[] = {50, 100, 150, 200, 300, 400, 500, 600, 800, 1000, 1500, 2000, 3000, 4000, 6000};
    const float need = (float)std::max(value, peak) * 1.08f;
    if (need > scale) for (float st : steps) if (st >= need) { scale = st; break; }
    const float h = width * 0.80f;
    ImGui::PushID(id);
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("##g", ImVec2(width, h));
    ImDrawList *dl = ImGui::GetWindowDrawList();
    const ImVec2 c(p0.x + width * 0.5f, p0.y + width * 0.47f);
    const float r = width * 0.43f;
    const float a0 = 3.14159265f * 0.75f, a1 = 3.14159265f * 2.25f;   // 135..405 degrees
    auto ang = [&](double v) { return a0 + (a1 - a0) * (float)std::clamp(v / scale, 0.0, 1.0); };
    auto pt = [&](float a, float rr) { return ImVec2(c.x + std::cos(a) * rr, c.y + std::sin(a) * rr); };
    // Face and track.
    dl->AddCircleFilled(c, r * 1.12f, IM_COL32(18, 20, 26, 255), 64);
    dl->AddCircle(c, r * 1.12f, IM_COL32(70, 76, 90, 255), 64, 1.5f);
    dl->PathArcTo(c, r, a0, a1, 64);
    dl->PathStroke(IM_COL32(45, 50, 62, 255), 0, r * 0.10f);
    // Damped needle (eases toward the value, like a real gauge).
    ImGuiStorage *st = ImGui::GetStateStorage();
    float *disp = st->GetFloatRef(ImGui::GetID("needle"), (float)value);
    *disp += ((float)value - *disp) * std::min(1.0f, ImGui::GetIO().DeltaTime * 4.0f);
    // Lit arc up to the needle.
    if (*disp > 0) {
        dl->PathArcTo(c, r, a0, ang(*disp), 48);
        dl->PathStroke(accent, 0, r * 0.10f);
    }
    // Ticks: 10 major with numbers, 4 minor between.
    const float fs = ImGui::GetFontSize();
    for (int i = 0; i <= 50; i++) {
        const float a = a0 + (a1 - a0) * i / 50.0f;
        const bool major = i % 5 == 0;
        const bool red = i >= 43;   // the "redline" top of the dial
        const ImU32 col = red ? IM_COL32(230, 70, 60, 255) : IM_COL32(200, 205, 215, 255);
        dl->AddLine(pt(a, r * (major ? 0.78f : 0.84f)), pt(a, r * 0.90f), col, major ? 2.0f : 1.0f);
        if (major && i % 10 == 0) {
            char t[16]; snprintf(t, sizeof(t), "%.0f", scale * i / 50.0f);
            const float tfs = fs * 0.78f;
            const ImVec2 ts = ImGui::GetFont()->CalcTextSizeA(tfs, FLT_MAX, 0, t);
            const ImVec2 q = pt(a, r * 0.62f);
            dl->AddText(ImGui::GetFont(), tfs, ImVec2(q.x - ts.x * 0.5f, q.y - ts.y * 0.5f),
                        IM_COL32(170, 176, 190, 255), t);
        }
    }
    // Peak-hold marker.
    if (peak > 0) {
        const float ap = ang(peak);
        dl->AddTriangleFilled(pt(ap, r * 1.06f), pt(ap - 0.06f, r * 1.17f), pt(ap + 0.06f, r * 1.17f),
                              IM_COL32(255, 80, 70, 255));
    }
    // Needle.
    const float an = ang(*disp);
    const ImVec2 tip = pt(an, r * 0.95f), back = pt(an + 3.14159265f, r * 0.14f);
    const ImVec2 side(std::cos(an + 1.5708f) * r * 0.035f, std::sin(an + 1.5708f) * r * 0.035f);
    dl->AddTriangleFilled(tip, ImVec2(back.x + side.x, back.y + side.y), ImVec2(back.x - side.x, back.y - side.y),
                          IM_COL32(255, 96, 64, 255));
    dl->AddCircleFilled(c, r * 0.09f, IM_COL32(60, 64, 76, 255), 24);
    dl->AddCircleFilled(c, r * 0.05f, IM_COL32(200, 205, 215, 255), 16);
    // Digital readout and label.
    char vb[24]; snprintf(vb, sizeof(vb), "%.0f", std::max(0.0, value));
    const float big = fs * 1.9f;
    ImVec2 vs = ImGui::GetFont()->CalcTextSizeA(big, FLT_MAX, 0, vb);
    dl->AddText(ImGui::GetFont(), big, ImVec2(c.x - vs.x * 0.5f, c.y + r * 0.30f), accent, vb);
    ImVec2 ls = ImGui::GetFont()->CalcTextSizeA(fs * 0.85f, FLT_MAX, 0, label);
    dl->AddText(ImGui::GetFont(), fs * 0.85f, ImVec2(c.x - ls.x * 0.5f, c.y + r * 0.30f + big),
                IM_COL32(170, 176, 190, 255), label);
    if (peak > 0) {
        char pb[32]; snprintf(pb, sizeof(pb), "peak %.0f", peak);
        ImVec2 ps = ImGui::GetFont()->CalcTextSizeA(fs * 0.78f, FLT_MAX, 0, pb);
        dl->AddText(ImGui::GetFont(), fs * 0.78f, ImVec2(c.x - ps.x * 0.5f, c.y - r * 0.42f),
                    IM_COL32(255, 110, 95, 255), pb);
    }
    ImGui::PopID();
}

static const iridi::LiveIqSource *active_live_source() {
    return g_source_kind == 1 ? &g_sdr : g_source_kind == 2 ? &g_hackrf
         : g_source_kind == 3 ? &g_airspy : g_source_kind == 4 ? &g_rtlsdr : nullptr;
}

// Recent-activity tracking for the drop / queue-full rows in Stats. Updated
// every frame (even with Stats collapsed) so "last" times stay accurate:
// the rows show the last 60 s and when it last happened; totals since Start
// are in the tooltips.
struct EvTrack {
    struct Snap { std::chrono::steady_clock::time_point t; uint64_t app, drv, qf, sk; };
    std::deque<Snap> hist;
    std::chrono::system_clock::time_point last_app, last_drv, last_qf;
    bool have_app = false, have_drv = false, have_qf = false;
    uint64_t win_app = 0, win_drv = 0, win_qf = 0;
};
static EvTrack g_ev;
static uint64_t g_skip_60 = 0;   // bursts skipped in the last 60 s
static void update_event_tracking() {
    uint64_t app = 0, drv = 0, qf = g_engine.queue_full_count(), sk = g_engine.skipped_bursts();
    if (const iridi::LiveIqSource *lv = active_live_source()) {
        app = lv->dropped_samples();
        if (lv->reports_driver_drops()) drv = lv->driver_dropped_samples();
    }
    auto &h = g_ev.hist;
    const auto tnow = std::chrono::steady_clock::now();
    if (!h.empty() && (app < h.back().app || drv < h.back().drv || qf < h.back().qf || sk < h.back().sk)) {
        h.clear(); g_ev.have_app = g_ev.have_drv = g_ev.have_qf = false;   // counters restarted
    }
    if (!h.empty()) {
        const auto wall = std::chrono::system_clock::now();
        if (app > h.back().app) { g_ev.last_app = wall; g_ev.have_app = true; }
        if (drv > h.back().drv) { g_ev.last_drv = wall; g_ev.have_drv = true; }
        if (qf > h.back().qf)   { g_ev.last_qf = wall;  g_ev.have_qf = true; }
    }
    if (h.empty() || tnow - h.back().t >= std::chrono::milliseconds(250)) h.push_back({tnow, app, drv, qf, sk});
    else { h.back().app = app; h.back().drv = drv; h.back().qf = qf; h.back().sk = sk; }
    while (h.size() > 1 && tnow - h[1].t >= std::chrono::seconds(60)) h.pop_front();
    g_ev.win_app = app - h.front().app; g_ev.win_drv = drv - h.front().drv; g_ev.win_qf = qf - h.front().qf;
    g_skip_60 = sk - h.front().sk;
}

// Detector load (% of real time spent detecting; >100% means it can't keep
// up) and detected bursts per second, both over the last ~10 s.
struct LoadSample { std::chrono::steady_clock::time_point t; uint64_t busy, wait, cpu, samples, bursts, reused, fresh, wide; };
static std::deque<LoadSample> g_load_hist;
static double g_det_load = -1.0, g_bursts_ps = -1.0, g_det_wait = -1.0, g_det_cpu = -1.0;
static double g_fresh_ps = -1.0, g_reused_pct = -1.0;   // burst buffers, last ~10 s
static double g_wide_ps = -1.0;                         // wide-carrier detections dropped/s
static size_t g_buf_live = 0, g_buf_idle = 0;           // burst buffer memory now
static void update_detector_load() {
    const auto now = std::chrono::steady_clock::now();
    const uint64_t busy = g_engine.detector_busy_ns(), smp = g_engine.detector_samples();
    const uint64_t wait = g_engine.detector_wait_ns(), cpu = g_engine.detector_cpu_ns();
    const uint64_t bursts = g_engine.stats().bursts;
    uint64_t reused = 0, fresh = 0;
    ir_pipeline_buffer_stats(&reused, &fresh, &g_buf_live, &g_buf_idle);
    if (!g_load_hist.empty() && (smp < g_load_hist.back().samples || bursts < g_load_hist.back().bursts))
        g_load_hist.clear();
    if (g_load_hist.empty() || now - g_load_hist.back().t >= std::chrono::milliseconds(250))
        g_load_hist.push_back({now, busy, wait, cpu, smp, bursts, reused, fresh, ir_wide_dropped()});
    while (g_load_hist.size() > 2 && now - g_load_hist[1].t >= std::chrono::seconds(10)) g_load_hist.pop_front();
    g_det_load = g_bursts_ps = g_det_wait = g_det_cpu = g_fresh_ps = g_reused_pct = g_wide_ps = -1.0;
    if (g_load_hist.size() < 2 || !g_engine.running()) return;
    const auto &a = g_load_hist.front(), &b = g_load_hist.back();
    const double rate = active_source()->sample_rate();
    const double dt = std::chrono::duration<double>(b.t - a.t).count();
    if (rate > 0 && b.samples > a.samples) {
        const double sig = (b.samples - a.samples) / rate;
        g_det_load = 100.0 * ((b.busy - a.busy) / 1e9) / sig;
        g_det_wait = 100.0 * ((b.wait - a.wait) / 1e9) / sig;
        g_det_cpu = 100.0 * ((b.cpu - a.cpu) / 1e9) / sig;
    }
    if (dt > 0.5) {
        g_bursts_ps = (b.bursts - a.bursts) / dt;
        g_wide_ps = b.wide >= a.wide ? (b.wide - a.wide) / dt : 0.0;
        const uint64_t r = b.reused - a.reused, f = b.fresh - a.fresh;
        g_fresh_ps = f / dt;
        if (r + f > 0) g_reused_pct = 100.0 * r / (double)(r + f);
    }
}

void draw_stats_contents() {
    iridi::EngineStats s = g_engine.stats();
    double good_rate = -1.0, drop_pct = -1.0, drv_pct = -1.0;
    {
        const auto now = std::chrono::steady_clock::now();
        const uint64_t parsed = g_engine.parsed_count(), errs = g_engine.parse_error_count();
        const uint64_t good = parsed - std::min(parsed, errs);
        const auto *live = active_live_source();
        const uint64_t rx = live ? live->received_samples() : 0, dr = live ? live->dropped_samples() : 0;
        const uint64_t drv = live ? live->driver_dropped_samples() : 0;
        // Restarted (counters went backwards)? Start the history again.
        if (!g_rate_hist.empty() && (good < g_rate_hist.back().good || rx < g_rate_hist.back().rx)) {
            g_rate_hist.clear(); g_good_hist_n = 0; g_peak_good = g_peak_bursts = 0;
        }
        if (g_rate_hist.empty() || now - g_rate_hist.back().t >= std::chrono::milliseconds(500)) {
            g_rate_hist.push_back({now, good, rx, dr, drv});
            while (g_rate_hist.size() > 2 && now - g_rate_hist[1].t >= std::chrono::seconds(10))
                g_rate_hist.pop_front();
            if (g_rate_hist.size() >= 2) {
                const auto &a = g_rate_hist.front();
                const double dt = std::chrono::duration<double>(now - a.t).count();
                if (dt > 0.9 && s.running) {
                    const float v = (float)((good - a.good) / dt);
                    const float bv = (float)std::max(0.0, g_bursts_ps);
                    if (g_good_hist_n < 120) { g_burst_hist[g_good_hist_n] = bv; g_good_hist[g_good_hist_n++] = v; }
                    else {
                        std::memmove(g_good_hist, g_good_hist + 1, sizeof(float) * 119); g_good_hist[119] = v;
                        std::memmove(g_burst_hist, g_burst_hist + 1, sizeof(float) * 119); g_burst_hist[119] = bv;
                    }
                }
            }
        }
        if (g_rate_hist.size() >= 2 && s.running) {
            const auto &a = g_rate_hist.front();
            const double dt = std::chrono::duration<double>(now - a.t).count();
            if (dt > 0.9) {
                good_rate = (good - a.good) / dt;
                if (live && rx > a.rx) drop_pct = 100.0 * (double)(dr - a.dropped) / (double)(rx - a.rx);
                if (live && live->reports_driver_drops() && rx > a.rx) {
                    const double d = (double)(drv - std::min(drv, a.drv));
                    drv_pct = 100.0 * d / ((double)(rx - a.rx) + d);
                }
            }
        }
    }
    // Headline: two tachometers (good frames/s and bursts/s), the 60 s trend,
    // a one-line health check and the session's highlights. All the detailed
    // counters are under "Details".
    if (good_rate >= 0) {
        if (good_rate > g_peak_good) { g_peak_good = good_rate; g_peak_good_t = std::chrono::system_clock::now(); }
        if (g_bursts_ps > g_peak_bursts) g_peak_bursts = g_bursts_ps;
        static float scale_good = 200, scale_bursts = 1000;
        const float avail = ImGui::GetContentRegionAvail().x;
        const float gw = std::min(220.0f, (avail - ImGui::GetStyle().ItemSpacing.x) * 0.5f);
        const float indent = std::max(0.0f, (avail - 2 * gw - ImGui::GetStyle().ItemSpacing.x) * 0.5f);
        if (indent > 0) ImGui::SetCursorPosX(ImGui::GetCursorPosX() + indent);
        draw_gauge("good", "GOOD FRAMES/s", good_rate, g_peak_good, scale_good, gw, IM_COL32(90, 230, 120, 255));
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Frames decoded without errors per second (10 s average).\nRed marker: session peak.");
        ImGui::SameLine();
        draw_gauge("bursts", "BURSTS/s", std::max(0.0, g_bursts_ps), g_peak_bursts, scale_bursts, gw,
                   IM_COL32(90, 180, 255, 255));
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Bursts the detector finds per second (10 s average): everything above\n"
                              "the noise, including what doesn't decode. Red marker: session peak.");
        if (g_good_hist_n > 1) {
            float mx = 1.0f;
            for (int i = 0; i < g_good_hist_n; i++) mx = std::max(mx, g_good_hist[i]);
            ImGui::PushStyleColor(ImGuiCol_PlotLines, ImVec4(0.35f, 0.9f, 0.47f, 1.0f));
            ImGui::PlotLines("##goodspark", g_good_hist, g_good_hist_n, 0, "good frames/s, last 60 s", 0.0f,
                             mx * 1.1f, ImVec2(-1, 40));
            ImGui::PopStyleColor();
            float mb = 1.0f;
            for (int i = 0; i < g_good_hist_n; i++) mb = std::max(mb, g_burst_hist[i]);
            ImGui::PushStyleColor(ImGuiCol_PlotLines, ImVec4(0.35f, 0.7f, 1.0f, 1.0f));
            ImGui::PlotLines("##burstspark", g_burst_hist, g_good_hist_n, 0, "bursts/s, last 60 s", 0.0f,
                             mb * 1.1f, ImVec2(-1, 28));
            ImGui::PopStyleColor();
        }
        // Health: one line, green when nothing was lost in the last 60 s.
        {
            const iridi::LiveIqSource *live = active_live_source();
            const double sr = live && live->sample_rate() > 0 ? (double)live->sample_rate() : 1.0;
            const double lost_ms = 1000.0 * (double)(g_ev.win_app + g_ev.win_drv) / sr;
            if (lost_ms > 0 || g_skip_60 > 0) {
                ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.35f, 1.0f), "Last 60 s: %.0f ms of radio lost, %llu bursts skipped",
                                   lost_ms, (unsigned long long)g_skip_60);
            } else {
                ImGui::TextColored(ImVec4(0.45f, 1.0f, 0.55f, 1.0f), "Healthy");
                ImGui::SameLine();
                ImGui::TextDisabled("nothing lost in the last 60 s");
            }
            if (g_det_load >= 0) {
                ImGui::SameLine();
                ImGui::TextDisabled("  detector %.0f%%", g_det_load);
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Detector: share of real time the burst detector needs (under 100%% keeps up).\n"
                                  "Radio lost = samples dropped by IridiScope or the Airspy driver.\n"
                                  "Skipped = bursts the decoders let go because they were more than the\n"
                                  "Decoder backlog behind. Details below.");
        }
    } else {
        ImGui::TextDisabled("Good frames/s: (start a source)");
    }
    // Session highlights.
    if (ImGui::BeginTable("highlights", 2, ImGuiTableFlags_None)) {
        ImGui::TableSetupColumn("k", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("v", ImGuiTableColumnFlags_WidthStretch);
        auto hrow = [](const char *k, const char *v, ImVec4 col = ImVec4(1, 1, 1, 1)) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::TextDisabled("%s", k);
            ImGui::TableSetColumnIndex(1); ImGui::TextColored(col, "%s", v);
        };
        char b[128];
        const uint64_t parsed = g_engine.parsed_count(), perr = g_engine.parse_error_count();
        const uint64_t good = parsed - std::min(parsed, perr);
        auto grp = [](uint64_t v, char *out, size_t n) {   // 1234567 -> 1,234,567
            char t[32]; snprintf(t, sizeof(t), "%llu", (unsigned long long)v);
            const int len = (int)strlen(t); int o = 0;
            for (int i = 0; i < len && o < (int)n - 1; i++) { out[o++] = t[i]; if ((len - i - 1) % 3 == 0 && i < len - 1 && o < (int)n - 1) out[o++] = ','; }
            out[o] = 0;
        };
        char g1[40]; grp(good, g1, sizeof(g1));
        snprintf(b, sizeof(b), "%s", g1);
        hrow("Frames decoded", b, ImVec4(0.45f, 1.0f, 0.55f, 1.0f));
        if (g_peak_good > 0) {
            std::time_t tt = std::chrono::system_clock::to_time_t(g_peak_good_t);
            std::tm lt{};
#ifdef _WIN32
            localtime_s(&lt, &tt);
#else
            localtime_r(&tt, &lt);
#endif
            snprintf(b, sizeof(b), "%.0f good frames/s at %02d:%02d", g_peak_good, lt.tm_hour, lt.tm_min);
            hrow("Session peak", b);
        }
        {
            const auto all = g_engine.satellites();
            const uint64_t now_ns = ring_now_ns();
            size_t now_n = 0;
            for (const auto &sv : all) if (age_s(sv.last_seen_ns, now_ns) <= g_sat_hide_after_s) now_n++;
            snprintf(b, sizeof(b), "%zu now, %zu since start", now_n, all.size());
            hrow("Satellites", b);
        }
        {
            const auto &vm = g_engine.voices();
            const double tot = vm.speech_total_s();
            snprintf(b, sizeof(b), "%llu with speech (%d:%02d)", (unsigned long long)vm.speech_calls(),
                     (int)(tot / 60), (int)tot % 60);
            hrow("Voice calls", b);
            if (vm.longest_speech_s() > 0) {
                snprintf(b, sizeof(b), "%.0f s of speech", vm.longest_speech_s());
                hrow("Longest call", b);
            }
            snprintf(b, sizeof(b), "%llu of %llu followed", (unsigned long long)vm.handoffs_followed(),
                     (unsigned long long)vm.handoffs_seen());
            hrow("Handoffs", b);
            if (g_source_kind != 0) {
                snprintf(b, sizeof(b), "%llu voice frames", (unsigned long long)g_engine.voice_recovered());
                hrow("Recovered", b);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Weak voice frames decoded by targeting each call's next burst.");
            }
        }
        {
            const auto &ac = g_engine.acars();
            snprintf(b, sizeof(b), "ACARS %llu  pager %llu  SMS %llu  SBD %llu",
                     (unsigned long long)ac.acars_count(), (unsigned long long)g_engine.pager().count(),
                     (unsigned long long)ac.sms_count(), (unsigned long long)ac.sbd_message_count());
            hrow("Messages", b);
        }
        {
            const int el = (int)s.elapsed_s;
            snprintf(b, sizeof(b), "%d:%02d:%02d", el / 3600, (el / 60) % 60, el % 60);
            hrow("Running", b);
        }
        ImGui::EndTable();
    }
    uint64_t win_app = g_ev.win_app, win_drv = g_ev.win_drv, win_qf = g_ev.win_qf;
    const bool have_app = g_ev.have_app, have_drv = g_ev.have_drv, have_qf = g_ev.have_qf;
    const auto last_app = g_ev.last_app, last_drv = g_ev.last_drv, last_qf = g_ev.last_qf;
    auto ago = [](bool have, std::chrono::system_clock::time_point t, char *out, size_t n) {
        if (!have) { snprintf(out, n, "never"); return; }
        std::time_t tt = std::chrono::system_clock::to_time_t(t);
        std::tm lt{};
#ifdef _WIN32
        localtime_s(&lt, &tt);
#else
        localtime_r(&tt, &lt);
#endif
        snprintf(out, n, "%02d:%02d:%02d", lt.tm_hour, lt.tm_min, lt.tm_sec);
    };
    if (!ImGui::TreeNode("Details")) return;
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
        {
            const auto all = g_engine.satellites();
            const uint64_t now_ns = ring_now_ns();
            size_t now_n = 0;
            for (const auto &sv : all) if (age_s(sv.last_seen_ns, now_ns) <= g_sat_hide_after_s) now_n++;
            snprintf(b, sizeof(b), "%zu now / %zu since start", now_n, all.size());
            row("Satellites", b);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("'now' = heard in the last %.0f s (the map's Hide after setting).", g_sat_hide_after_s);
        }
        {
            const iridi::LiveIqSource *live = active_live_source();
            if (live) {
                const uint64_t rx = live->received_samples(), dr = live->dropped_samples();
                const double sr = live->sample_rate() > 0 ? (double)live->sample_rate() : 1.0;
                // One row per drop source: last 60 s (as signal time) + when it
                // last happened; the hover tooltip has the totals since Start.
                auto drop_row = [&](const char *label, uint64_t win, uint64_t total, double total_pct,
                                    bool have, std::chrono::system_clock::time_point last, const char *what) {
                    char when[16]; ago(have, last, when, sizeof(when));
                    char v[96];
                    snprintf(v, sizeof(v), "%.0f ms   (last: %s)", 1000.0 * (double)win / sr, when);
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(label);
                    const bool hov0 = ImGui::IsItemHovered();
                    ImGui::TableSetColumnIndex(1);
                    if (win) ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.35f, 1.0f), "%s", v);
                    else ImGui::TextUnformatted(v);
                    if (hov0 || ImGui::IsItemHovered())
                        ImGui::SetTooltip("%s\nShown: signal lost in the last 60 s, and when a drop last happened.\n"
                                          "Since Start: %llu samples = %.1f s (%.3f%%)",
                                          what, (unsigned long long)total, (double)total / sr, total_pct);
                };
                drop_row("Dropped, app (60 s)", win_app, dr, rx ? 100.0 * dr / rx : 0.0, have_app, last_app,
                         "Samples IridiScope discarded because decoding fell behind the radio.");
                if (live->reports_driver_drops()) {
                    const uint64_t dd = live->driver_dropped_samples();
                    drop_row("Dropped, driver (60 s)", win_drv, dd, (rx + dd) ? 100.0 * dd / (double)(rx + dd) : 0.0,
                             have_drv, last_drv,
                             "Samples the Airspy driver discarded before IridiScope got them\n"
                             "(its USB conversion thread fell behind).");
                }
                if (live->buffer_seconds() > 0) {
                    snprintf(b, sizeof(b), "%.2f s", live->buffer_seconds());
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Sample buffer");
                    ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(b);
                }
            }
        }
        {
            const uint64_t rec = g_engine.uw_recovered(), tf = g_engine.uw_tolerated_failed();
            const uint64_t good = g_engine.parsed_count() - std::min(g_engine.parsed_count(), g_engine.parse_error_count());
            snprintf(b, sizeof(b), "%llu (%.1f%% of good)", (unsigned long long)rec, good ? 100.0 * rec / good : 0.0);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Recovered by sync tolerance");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Good frames that only parsed because the sync pattern had 1-3 wrong\n"
                                  "symbols. Without the option these would be 'Access code missing'.\n"
                                  "(%llu more were accepted on sync but failed later checks.)",
                                  (unsigned long long)tf);
            ImGui::TableSetColumnIndex(1);
            if (irid::irid_uw_ec.load()) ImGui::TextUnformatted(b); else ImGui::TextDisabled("off");
        }
        if (g_engine.worker_threads() > 0 && g_source_kind != 0) {
            snprintf(b, sizeof(b), "%d  (backlog %.0f s)", g_engine.worker_threads(),
                     kBacklogS[std::clamp(g_backlog_idx, 0, 3)]);
            row("Decoder worker threads", b);
        }
        if (g_engine.running()) {
            if (!g_reject_wide) snprintf(b, sizeof(b), "off");
            else if (g_wide_ps >= 0) snprintf(b, sizeof(b), "%.0f /s  (%llu total)", g_wide_ps, (unsigned long long)ir_wide_dropped());
            else snprintf(b, sizeof(b), "%llu total", (unsigned long long)ir_wide_dropped());
            row("Wideband carrier detections ignored", b);
        }
        if (g_engine.worker_threads() > 0 && g_source_kind == 0) {
            snprintf(b, sizeof(b), "%d  (queue %d bursts)", g_engine.worker_threads(), g_engine.queue_capacity());
            row("Decoder worker threads", b);
            const uint64_t qf = g_engine.queue_full_count();
            char when[16]; ago(have_qf, last_qf, when, sizeof(when));
            snprintf(b, sizeof(b), "%llu   (last: %s)", (unsigned long long)win_qf, when);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Queue-full waits (60 s)");
            const bool hov0 = ImGui::IsItemHovered();
            ImGui::TableSetColumnIndex(1);
            if (win_qf) ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.35f, 1.0f), "%s", b);
            else ImGui::TextUnformatted(b);
            if (hov0 || ImGui::IsItemHovered())
                ImGui::SetTooltip("Bursts that had to wait in the last 60 s because the burst queue was full\n"
                                  "(every decoder worker busy). Short bursts of waits are absorbed by the\n"
                                  "sample buffer; sustained waits stall the detector and samples drop.\n"
                                  "Since Start: %llu", (unsigned long long)qf);
        }
        if (g_source_kind != 0 && g_engine.running()) {
            const uint64_t sk = g_engine.skipped_bursts();
            snprintf(b, sizeof(b), "%llu   (since start: %llu)", (unsigned long long)g_skip_60, (unsigned long long)sk);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Bursts skipped (60 s)");
            const bool hov1 = ImGui::IsItemHovered();
            ImGui::TableSetColumnIndex(1);
            if (g_skip_60) ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.35f, 1.0f), "%s", b);
            else ImGui::TextUnformatted(b);
            if (hov1 || ImGui::IsItemHovered())
                ImGui::SetTooltip("Bursts the decoders skipped because they were further behind the radio\n"
                                  "than the Decoder backlog setting (a traffic peak bigger than the CPU).\n"
                                  "The radio itself keeps flowing; only the oldest bursts are skipped.");
        }
        if (g_source_kind != 0 && g_engine.running()) {
            snprintf(b, sizeof(b), "%llu   (%llu targeted reads)", (unsigned long long)g_engine.voice_recovered(),
                     (unsigned long long)g_engine.targeted_reads());
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Voice frames recovered");
            const bool hov2 = ImGui::IsItemHovered();
            ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(b);
            if (hov2 || ImGui::IsItemHovered())
                ImGui::SetTooltip("Voice frames decoded by reading a call's next burst at its exact time and\n"
                                  "frequency, where the detector found nothing (burst too weak to trigger it).\n"
                                  "Silent stretches of a call have no bursts, so most reads find none.");
        }
        snprintf(b, sizeof(b), "%llu", (unsigned long long)g_engine.voices().handoffs_seen());
        row("Handoff resp. (on calls)", b);
        snprintf(b, sizeof(b), "%llu", (unsigned long long)g_engine.voices().handoffs_followed());
        row("Handoffs followed", b);
        snprintf(b, sizeof(b), "%.1f s", s.elapsed_s);                     row("Elapsed", b);
        snprintf(b, sizeof(b), "%s", s.running ? (s.paused ? "paused" : "running") : "stopped");
        row("State", b);
        ImGui::EndTable();
    }
    ImGui::TreePop();
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
    if (hi < kBandLoMhz || lo > kBandHiMhz) return;
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
                    double max_span = live ? fs_mhz
                                           : std::min(kBandHiMhz - kBandLoMhz, fs_mhz);
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
        if (g_autoplay && (!g_voice_hide_failed || (c.error_ratio <= g_voice_max_err && c.speech_s >= 1.0)) &&
            (!g_voice_hide_short || c.duration_s >= g_voice_min_s))
            g_autoplay_queue.push_back(c.id);
    }
    if (!g_autoplay) {
        g_autoplay_queue.clear();
        return;
    }
    // Wait for a saved call's announcement check before playing it, and skip
    // announcements while they're hidden.
    while (!g_audio.playing() && !g_autoplay_queue.empty()) {
        const int qid = g_autoplay_queue.front();
        bool wait = false, skip = false;
        for (const auto &c : calls)
            if (c.id == qid) {
                if (!c.wav_path.empty() && !c.classified) wait = true;
                if (g_voice_hide_ann && !c.announcement.empty()) skip = true;
            }
        if (wait) break;
        if (skip) { g_autoplay_queue.pop_front(); continue; }
        break;
    }
    if (!g_audio.playing() && !g_autoplay_queue.empty()) {
        bool pending = false;
        for (const auto &c : calls)
            if (c.id == g_autoplay_queue.front() && !c.wav_path.empty() && !c.classified) pending = true;
        if (pending) return;
        int id = g_autoplay_queue.front();
        g_autoplay_queue.pop_front();
        auto pcm = g_engine.voices().pcm(id);
        if (!pcm.empty()) {
            g_audio.play(pcm, 8000);
            g_audio_call_id = id;
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
        // Finished calls above the error threshold are hidden unless asked for;
        // in-progress calls stay visible since their error ratio is still settling.
        auto hidden = [](const iridi::VoiceCall &c) {
            if (c.active) return false;
            if (g_voice_hide_failed && c.error_ratio > g_voice_max_err) return true;
            if (g_voice_hide_failed && c.speech_s < 1.0) return true;   // silence updates only
            if (g_voice_hide_ann && !c.announcement.empty()) return true;
            if (g_voice_hide_short && c.duration_s < g_voice_min_s) return true;
            return false;
        };
        int nHidden = 0; for (auto &c : calls) if (hidden(c)) nHidden++;
        ImGui::TextDisabled("%zu voice calls (%d active, %d hidden)  [%s]", calls.size(), active,
                            nHidden, g_audio.playing() ? "playing" : "idle");
        ImGui::SameLine();
        ImGui::Checkbox("Hide failed", &g_voice_hide_failed);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Hide finished calls whose Err is above the threshold, and calls with\n"
                              "under 1 s of speech (only the network's once-a-second silence\n"
                              "updates: the person on this side never spoke).");
        ImGui::SameLine();
        ImGui::BeginDisabled(!g_voice_hide_failed);
        ImGui::SetNextItemWidth(90);
        ImGui::SliderFloat("Max Err", &g_voice_max_err, 0.05f, 1.0f, "%.2f");
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::Checkbox("Hide <", &g_voice_hide_short);
        ImGui::SameLine(0, 4);
        ImGui::BeginDisabled(!g_voice_hide_short);
        ImGui::SetNextItemWidth(56);
        ImGui::DragFloat("s##minlen", &g_voice_min_s, 0.1f, 0.0f, 120.0f, "%.1f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Hide finished calls shorter than this many seconds (drag, or\n"
                              "double-click to type). Autoplay skips them too.");
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::Checkbox("Hide announcements", &g_voice_hide_ann);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Hide calls that are only a recorded network announcement (\"The subscriber\n"
                              "cannot be reached...\", \"...911, 112 or 000...\"). Their WAVs go to\n"
                              "recordings\\announcements\\<name>\\. Reference recordings: assets\\announcements\\\n"
                              "(add a WAV there to recognise another announcement). %d loaded.",
                              iridi::AnnouncementMatcher::instance().templates());
        ImGui::SameLine();
        {
            bool fh = g_engine.voices().follow_handoffs();
            if (ImGui::Checkbox("Follow handoffs", &fh)) g_engine.voices().set_follow_handoffs(fh);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Keep a call together when the network hands it to a new\n"
                                  "channel (beam/satellite handoff). Uses the handoff response\n"
                                  "sent on the call's own channel, as iridium-toolkit's\n"
                                  "voc-cluster.py does. HO column = handoffs followed.\n"
                                  "Applies to new calls; only works if the new channel is\n"
                                  "inside the receiver's bandwidth.");
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Hide finished calls shorter than 1 second\n"
                              "(fragments from handoffs or brief bursts).");
        ImGui::SameLine();
        if (g_audio.playing()) {
            if (ImGui::Button(g_audio.paused() ? "Resume##aud" : "Pause##aud"))
                g_audio.paused() ? g_audio.resume() : g_audio.pause();
            ImGui::SameLine();
        }
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
        // Local start time of a call, for the Time column.
        auto hhmmss = [](uint64_t ns, char *out, size_t n) {
            const std::time_t t = (std::time_t)(ns / 1000000000ULL);
            std::tm lt{};
#ifdef _WIN32
            if (localtime_s(&lt, &t) != 0) { snprintf(out, n, "-"); return; }
#else
            if (!localtime_r(&t, &lt)) { snprintf(out, n, "-"); return; }
#endif
            std::strftime(out, n, "%H:%M:%S", &lt);
        };
        auto select_row = [](int id) {
            if (g_selected_voice != id) { g_selected_voice = id; g_voice_cache_id = -1; }
        };

        // In-progress calls get their own small table, so calls starting and
        // ending never shift the list of finished recordings.
        std::vector<iridi::VoiceCall> act, fin;
        for (auto &c : calls) {
            if (c.active) act.push_back(c);
            else if (!hidden(c)) fin.push_back(c);
        }
        std::sort(act.begin(), act.end(), [](const auto &x, const auto &y) { return x.start_ns > y.start_ns; });
        ImGui::TextDisabled("Active (%zu)", act.size());
        if (ImGui::BeginTable("active_calls", 6, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders |
                                                 ImGuiTableFlags_ScrollY,
                              ImVec2(0, ImGui::GetTextLineHeightWithSpacing() * 4.6f))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, 52);
            ImGui::TableSetupColumn("Started", ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableSetupColumn("MHz", ImGuiTableColumnFlags_WidthFixed, 88);
            ImGui::TableSetupColumn("Dur s", ImGuiTableColumnFlags_WidthFixed, 56);
            ImGui::TableSetupColumn("Err", ImGuiTableColumnFlags_WidthFixed, 50);
            ImGui::TableSetupColumn("HO");
            ImGui::TableHeadersRow();
            for (const auto &cv : act) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                char lbl[32]; snprintf(lbl, sizeof(lbl), "#%d##a", cv.id);
                if (ImGui::Selectable(lbl, g_selected_voice == cv.id, ImGuiSelectableFlags_SpanAllColumns))
                    select_row(cv.id);
                char tb[16]; hhmmss(cv.start_ns, tb, sizeof(tb));
                ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(tb);
                ImGui::TableSetColumnIndex(2); ImGui::Text("%.4f", cv.channel_hz / 1e6);
                ImGui::TableSetColumnIndex(3); ImGui::Text("%.1f", cv.duration_s);
                ImGui::TableSetColumnIndex(4); ImGui::Text("%.2f", cv.error_ratio);
                ImGui::TableSetColumnIndex(5);
                if (cv.handoffs > 0) ImGui::Text("%d", cv.handoffs); else ImGui::TextDisabled("-");
            }
            ImGui::EndTable();
        }

        // Player for the selected call: controls, then its waveform, above the
        // finished list.
        if (g_selected_voice >= 0) {
            static auto cache_t = std::chrono::steady_clock::now();
            const auto tnow = std::chrono::steady_clock::now();
            bool sel_active = false;
            for (auto &c : calls) if (c.id == g_selected_voice && c.active) sel_active = true;
            // An in-progress call keeps growing: refresh its audio once a second.
            if (g_voice_cache_id != g_selected_voice || (sel_active && tnow - cache_t > std::chrono::seconds(1))) {
                g_voice_cache = g_engine.voices().pcm(g_selected_voice);
                g_voice_cache_id = g_selected_voice;
                cache_t = tnow;
            }
            if (!g_voice_cache.empty()) {
                // Play -> Pause -> Resume for the selected call; Stop resets.
                const bool sel_loaded = g_audio.playing() && g_audio_call_id == g_selected_voice;
                const char *pl = !sel_loaded ? "Play##selp" : g_audio.paused() ? "Resume##selp" : "Pause##selp";
                if (ImGui::Button(pl)) {
                    if (!sel_loaded) { g_audio.play(g_voice_cache, 8000); g_audio_call_id = g_selected_voice; }
                    else if (g_audio.paused()) g_audio.resume();
                    else g_audio.pause();
                }
                ImGui::SameLine();
                if (ImGui::Button("Stop")) g_audio.stop();
                ImGui::SameLine();
                ImGui::Text("Call #%d  %.1f s", g_selected_voice, g_voice_cache.size() / 8000.0);
                // Waveform: mirrored peak envelope, one bar per pixel, scaled to
                // the call's own loudest moment so quiet calls are visible too,
                // with a cursor while it plays.
                const float wh = ImGui::GetFrameHeight() * 3.2f;
                const float ww = ImGui::GetContentRegionAvail().x;
                const ImVec2 p0 = ImGui::GetCursorScreenPos();
                const bool wclick = ImGui::InvisibleButton("##selwave", ImVec2(std::max(1.0f, ww), wh));
                ImDrawList *wdl = ImGui::GetWindowDrawList();
                const ImVec2 p1(p0.x + ww, p0.y + wh);
                wdl->AddRectFilled(p0, p1, IM_COL32(16, 20, 28, 255), 3.0f);
                const float mid = p0.y + wh * 0.5f;
                wdl->AddLine(ImVec2(p0.x, mid), ImVec2(p1.x, mid), IM_COL32(60, 70, 90, 255));
                const auto &pcm = g_voice_cache;
                const int n = (int)pcm.size();
                int peak = 1;
                for (int16_t v : pcm) peak = std::max(peak, std::abs((int)v));
                const float scale = (wh * 0.48f) / (float)std::max(peak, 1500);
                const int cols = std::max(1, (int)ww);
                for (int c = 0; c < cols; c++) {
                    const int i0 = (int)((long long)c * n / cols), i1 = (int)((long long)(c + 1) * n / cols);
                    int pk = 0;
                    for (int k = i0; k < i1 && k < n; k++) pk = std::max(pk, std::abs((int)pcm[k]));
                    if (pk < 2) continue;
                    const float hgt = std::max(1.0f, pk * scale);
                    wdl->AddLine(ImVec2(p0.x + c + 0.5f, mid - hgt), ImVec2(p0.x + c + 0.5f, mid + hgt),
                                 IM_COL32(90, 200, 255, 230));
                }
                if (g_audio.playing() && g_audio_call_id == g_selected_voice && n > 0) {
                    const double t = g_audio.position_sec();
                    const float x = p0.x + (float)std::min(1.0, t * 8000.0 / n) * ww;
                    wdl->AddLine(ImVec2(x, p0.y), ImVec2(x, p1.y), IM_COL32(255, 120, 80, 255), 2.0f);
                }
                if (ImGui::IsItemHovered()) {
                    const float fx = (ImGui::GetIO().MousePos.x - p0.x) / std::max(1.0f, ww);
                    ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                    ImGui::SetTooltip("%.1f s  (click to play from here)", std::clamp(fx, 0.0f, 1.0f) * n / 8000.0f);
                }
                if (wclick && n > 0) {
                    // Jump within the loaded call, or start the selected one there.
                    const float at = std::clamp((ImGui::GetIO().MousePos.x - p0.x) / std::max(1.0f, ww), 0.0f, 1.0f) * n / 8000.0f;
                    if (g_audio_call_id != g_selected_voice || !g_audio.playing()) {
                        if (g_audio_call_id != g_selected_voice) { g_audio.play(g_voice_cache, 8000); g_audio_call_id = g_selected_voice; }
                    }
                    g_audio.seek(at);
                }
            } else {
                ImGui::TextDisabled("Call #%d: no audio captured yet", g_selected_voice);
            }
        }

        // Finished calls: sortable, and frozen while the pointer is over the
        // list so rows don't move under the mouse.
        static std::vector<iridi::VoiceCall> fin_shown;
        static bool fin_hovered = false;
        if (!fin_hovered) fin_shown = std::move(fin);
        ImGui::TextDisabled("Finished (%zu shown)%s", fin_shown.size(),
                            fin_hovered ? "   - list paused while the pointer is over it" : "");
        const float fin_h = std::max(140.0f, ImGui::GetContentRegionAvail().y);
        const ImVec2 fin_p0 = ImGui::GetCursorScreenPos();
        const ImVec2 fin_p1(fin_p0.x + ImGui::GetContentRegionAvail().x, fin_p0.y + fin_h);
        if (ImGui::BeginTable("calls", 11,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY |
                ImGuiTableFlags_Sortable | ImGuiTableFlags_SortTristate,
                ImVec2(0, fin_h))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, 52);
            ImGui::TableSetupColumn("Started", ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_DefaultSort |
                                               ImGuiTableColumnFlags_PreferSortDescending, 70);
            ImGui::TableSetupColumn("MHz", ImGuiTableColumnFlags_WidthFixed, 88);
            ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 46);
            ImGui::TableSetupColumn("Dur s", ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_PreferSortDescending, 56);
            ImGui::TableSetupColumn("Speech", ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_PreferSortDescending, 56);
            ImGui::TableSetupColumn("Frames", ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_PreferSortDescending, 56);
            ImGui::TableSetupColumn("Err", ImGuiTableColumnFlags_WidthFixed, 50);
            ImGui::TableSetupColumn("Miss", ImGuiTableColumnFlags_WidthFixed, 44);
            ImGui::TableSetupColumn("HO", ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_PreferSortDescending, 34);
            ImGui::TableSetupColumn("Saved", ImGuiTableColumnFlags_NoSort);
            ImGui::TableHeadersRow();
            if (ImGuiTableSortSpecs *ss = ImGui::TableGetSortSpecs()) {
                if (ss->SpecsCount > 0) {
                    const ImGuiTableColumnSortSpecs spec = ss->Specs[0];
                    auto key = [&](const iridi::VoiceCall &c) -> double {
                        switch (spec.ColumnIndex) {
                        case 0: return c.id;
                        case 2: return c.channel_hz;
                        case 3: return c.subtype.empty() ? 0 : (double)(unsigned char)c.subtype.back();
                        case 4: return c.duration_s;
                        case 5: return c.speech_s;
                        case 6: return c.frames;
                        case 7: return c.error_ratio;
                        case 8: return c.missing;
                        case 9: return c.handoffs;
                        default: return (double)c.start_ns;
                        }
                    };
                    const bool asc = spec.SortDirection == ImGuiSortDirection_Ascending;
                    std::stable_sort(fin_shown.begin(), fin_shown.end(), [&](const auto &x, const auto &y) {
                        const double kx = key(x), ky = key(y);
                        if (kx != ky) return asc ? kx < ky : kx > ky;
                        return x.start_ns > y.start_ns;
                    });
                } else {
                    std::stable_sort(fin_shown.begin(), fin_shown.end(),
                                     [](const auto &x, const auto &y) { return x.start_ns > y.start_ns; });
                }
            }
            for (const auto &cv : fin_shown) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                char lbl[32]; snprintf(lbl, sizeof(lbl), "#%d", cv.id);
                if (ImGui::Selectable(lbl, g_selected_voice == cv.id, ImGuiSelectableFlags_SpanAllColumns))
                    select_row(cv.id);
                if (!cv.wav_path.empty() && ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s", cv.wav_path.c_str());
                char tb[16]; hhmmss(cv.start_ns, tb, sizeof(tb));
                ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(tb);
                ImGui::TableSetColumnIndex(2); ImGui::Text("%.4f", cv.channel_hz / 1e6);
                ImGui::TableSetColumnIndex(3); ImGui::TextUnformatted(cv.subtype.c_str());
                ImGui::TableSetColumnIndex(4); ImGui::Text("%.1f", cv.duration_s);
                ImGui::TableSetColumnIndex(5);
                if (cv.speech_s > 0) ImGui::Text("%.1f", cv.speech_s); else ImGui::TextDisabled("-");
                ImGui::TableSetColumnIndex(6); ImGui::Text("%d", cv.frames);
                ImGui::TableSetColumnIndex(7);
                ImGui::TextColored(cv.error_ratio < 0.5f ? ImVec4(0.5f, 1, 0.5f, 1) : ImVec4(0.9f, 0.7f, 0.4f, 1),
                                   "%.2f", cv.error_ratio);
                ImGui::TableSetColumnIndex(8);
                if (cv.missing > 0) ImGui::Text("%d", cv.missing); else ImGui::TextDisabled("-");
                ImGui::TableSetColumnIndex(9);
                if (cv.handoffs > 0) ImGui::Text("%d", cv.handoffs); else ImGui::TextDisabled("-");
                ImGui::TableSetColumnIndex(10);
                if (!cv.announcement.empty()) {
                    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.95f, 1.0f), "ANN");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Network announcement: %s\n%s", cv.announcement.c_str(), cv.wav_path.c_str());
                } else if (!cv.wav_path.empty()) ImGui::TextUnformatted("WAV");
                else {
                    ImGui::TextDisabled("-");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Not saved: under 1 s of speech (silence or the network's\n"
                                          "once-a-second silence updates only).");
                }
            }
            ImGui::EndTable();
        }
        // Pause the list while the pointer is over it and this window is the
        // one under the mouse (not a popup or another window on top).
        fin_hovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) &&
                      ImGui::IsMouseHoveringRect(fin_p0, fin_p1, false);
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

    const iridi::SdrDeviceInfo *selected = nullptr;
    if (!devs.empty()) {
        if (g_sdr_device_idx < 0 || g_sdr_device_idx >= (int)devs.size()) g_sdr_device_idx = 0;
    } else {
        g_sdr_device_idx = 0;
    }

    if (scanning) {
        ImGui::TextDisabled("Scanning for SDRplay devices...");
    } else if (devs.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f), "No SDRplay device found.");
    } else {
        std::vector<const char *> names;
        names.reserve(devs.size());
        for (auto &d : devs) names.push_back(d.name.c_str());
        ImGui::SetNextItemWidth(200);
        if (ImGui::Combo("Device", &g_sdr_device_idx, names.data(), (int)names.size())) {
            g_sdr_antenna = 0;
        }
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(scanning);
    if (ImGui::Button("Refresh")) request_sdr_device_scan();
    ImGui::EndDisabled();

    selected = devs.empty() ? nullptr : &devs[g_sdr_device_idx];
    if (selected && !selected->ports.empty()) {
        if (g_sdr_antenna < 0 || g_sdr_antenna >= (int)selected->ports.size())
            g_sdr_antenna = 0;
        std::vector<const char *> ports;
        ports.reserve(selected->ports.size());
        for (auto &p : selected->ports) ports.push_back(p.c_str());
        ImGui::SetNextItemWidth(200);
        if (ImGui::Combo("Antenna port", &g_sdr_antenna, ports.data(), (int)ports.size())) {
            // Re-open the device so the port is applied through the same
            // start-up path; live antenna updates are unstable on RSPdx/RSPduo.
            if (g_sdr.is_open()) {
                add_log("[RADIO] Restarting SDRplay for antenna port: %s",
                        selected->ports[g_sdr_antenna].c_str());
                g_engine.stop();
                g_sdr.close();
                start_sdr();
            }
        }
    }

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
            add_log("[RADIO] SDRplay stopped");
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
            add_log("[RADIO] HackRF stopped");
        }
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.4f, 1, 0.4f, 1), "streaming: %s",
                           g_hackrf.device_name().c_str());
    }
#ifndef HAS_HACKRF
    ImGui::EndDisabled();
#endif
}

// ---- Airspy gain sweep ------------------------------------------------------
// Steps through gain settings in shuffled order over several rounds and
// measures good (error-free) frames per second at each, so satellite passes
// coming and going average out instead of biasing the result.
struct GainSweep {
    bool running = false;
    int mode = 0;                 // 0 = Sensitivity, 1 = Linearity
    int lo = 4, hi = 19, step = 3;
    int dwell_s = 10, rounds = 20;
    int settle_s = 2;
    std::vector<int> order;       // full schedule: gain values
    size_t pos = 0;
    bool settled = false;
    std::chrono::steady_clock::time_point t0;
    uint64_t parsed0 = 0, err0 = 0, ok0 = 0, fail0 = 0;
    int restore_gain = 10;
    struct Acc {
        double secs = 0; uint64_t good = 0, parsed = 0, errs = 0, demod_ok = 0, demod_fail = 0;
        std::vector<double> rates; // good frames/s of each individual test
        // Standard error of the mean across tests: how much the average could
        // move from traffic variation alone.
        double sem() const {
            if (rates.size() < 2) return 0.0;
            double m = 0; for (double r : rates) m += r; m /= rates.size();
            double v = 0; for (double r : rates) v += (r - m) * (r - m); v /= (rates.size() - 1);
            return std::sqrt(v / rates.size());
        }
    };
    std::map<int, Acc> acc;
    int best = -1;
};
GainSweep g_sweep;

static void sweep_apply_gain(int g) {
    if (g_sweep.mode == 0) { g_ap_sense = g; g_airspy.set_sense_gain(g); }
    else { g_ap_linear = g; g_airspy.set_linear_gain(g); }
}

static void sweep_compute_best() {
    double best_rate = -1.0;
    g_sweep.best = -1;
    for (const auto &kv : g_sweep.acc) {
        if (kv.second.secs <= 0) continue;
        const double r = kv.second.good / kv.second.secs;
        if (r > best_rate) { best_rate = r; g_sweep.best = kv.first; }
    }
}

static void sweep_finish(bool completed) {
    g_sweep.running = false;
    sweep_compute_best();
    sweep_apply_gain(g_sweep.restore_gain);
    if (completed && g_sweep.best >= 0) {
        add_log("[RADIO] Gain sweep done (%s mode). Results, good frames/s:",
                g_sweep.mode == 0 ? "Sensitivity" : "Linearity");
        for (const auto &kv : g_sweep.acc) {
            const auto &a = kv.second;
            if (a.secs <= 0) continue;
            add_log("  gain %2d: %6.1f +/- %4.1f good frames/s   parse errors %4.1f%%   demod failed %4.1f%%   (%.0f s)",
                    kv.first, a.good / a.secs, a.sem(), a.parsed ? 100.0 * a.errs / a.parsed : 0.0,
                    (a.demod_ok + a.demod_fail) ? 100.0 * a.demod_fail / (a.demod_ok + a.demod_fail) : 0.0,
                    a.secs);
        }
        add_log("[RADIO] Gain sweep best: %d (gain restored to %d until you apply it)", g_sweep.best, g_sweep.restore_gain);
    } else {
        add_log("[RADIO] Gain sweep stopped; gain restored to %d", g_sweep.restore_gain);
    }
}

static void sweep_snapshot() {
    const auto st = g_engine.stats();
    g_sweep.parsed0 = g_engine.parsed_count();
    g_sweep.err0 = g_engine.parse_error_count();
    g_sweep.ok0 = st.frames_ok;
    g_sweep.fail0 = st.frames_fail;
}

static void sweep_start() {
    if (g_ap_gain_mode != 0 && g_ap_gain_mode != 1) { add_log("Gain sweep: switch Gain mode to Sensitivity or Linearity first"); return; }
    if (!g_airspy.is_open() || !g_engine.running()) { add_log("Gain sweep: start the Airspy first"); return; }
    g_sweep.mode = g_ap_gain_mode;
    g_sweep.restore_gain = g_sweep.mode == 0 ? g_ap_sense : g_ap_linear;
    g_sweep.acc.clear();
    g_sweep.order.clear();
    std::vector<int> gains;
    for (int g = std::min(g_sweep.lo, g_sweep.hi); g <= std::max(g_sweep.lo, g_sweep.hi); g += std::max(1, g_sweep.step))
        gains.push_back(g);
    std::mt19937 rng((unsigned)std::chrono::steady_clock::now().time_since_epoch().count());
    for (int r = 0; r < std::max(1, g_sweep.rounds); r++) {
        std::shuffle(gains.begin(), gains.end(), rng);
        g_sweep.order.insert(g_sweep.order.end(), gains.begin(), gains.end());
    }
    g_sweep.pos = 0;
    g_sweep.settled = false;
    g_sweep.best = -1;
    g_sweep.running = true;
    sweep_apply_gain(g_sweep.order[0]);
    g_sweep.t0 = std::chrono::steady_clock::now();
    add_log("[RADIO] Gain sweep started: %zu settings x %d rounds, %d s each (~%.0f min)", gains.size(),
            std::max(1, g_sweep.rounds), g_sweep.dwell_s, g_sweep.order.size() * g_sweep.dwell_s / 60.0);
}

// Called every frame.
static void sweep_tick() {
    if (!g_sweep.running) return;
    if (!g_airspy.is_open() || !g_engine.running()) { sweep_finish(false); return; }
    const auto now = std::chrono::steady_clock::now();
    const double el = std::chrono::duration<double>(now - g_sweep.t0).count();
    if (!g_sweep.settled) {
        if (el >= g_sweep.settle_s) { sweep_snapshot(); g_sweep.settled = true; g_sweep.t0 = now; }
        return;
    }
    if (el < g_sweep.dwell_s) return;
    const auto st = g_engine.stats();
    auto &a = g_sweep.acc[g_sweep.order[g_sweep.pos]];
    const uint64_t parsed = g_engine.parsed_count() - g_sweep.parsed0;
    const uint64_t errs = g_engine.parse_error_count() - g_sweep.err0;
    a.secs += el;
    a.parsed += parsed;
    a.errs += errs;
    a.good += parsed - std::min(parsed, errs);
    a.rates.push_back(el > 0 ? (parsed - std::min(parsed, errs)) / el : 0.0);
    a.demod_ok += st.frames_ok - g_sweep.ok0;
    a.demod_fail += st.frames_fail - g_sweep.fail0;
    sweep_compute_best();
    if (++g_sweep.pos >= g_sweep.order.size()) { sweep_finish(true); return; }
    sweep_apply_gain(g_sweep.order[g_sweep.pos]);
    g_sweep.settled = false;
    g_sweep.t0 = now;
}

static void draw_gain_sweep() {
    if (!ImGui::CollapsingHeader("Gain sweep (find best gain)")) return;
    ImGui::TextWrapped("Tests each gain many times in shuffled order and measures good "
                       "(error-free) frames per second. Short tests with many rounds spread "
                       "changing satellite traffic evenly across all settings. Uses the current "
                       "Gain mode (Sensitivity or Linearity). Keep the antenna still while it runs.");
    ImGui::BeginDisabled(g_sweep.running);
    ImGui::SetNextItemWidth(160); ImGui::DragIntRange2("Gain range", &g_sweep.lo, &g_sweep.hi, 0.2f, 0, 21);
    ImGui::SetNextItemWidth(160); ImGui::SliderInt("Step", &g_sweep.step, 1, 5);
    ImGui::SetNextItemWidth(160); ImGui::SliderInt("Seconds per test", &g_sweep.dwell_s, 5, 120);
    ImGui::SetNextItemWidth(160); ImGui::SliderInt("Rounds", &g_sweep.rounds, 1, 40);
    ImGui::EndDisabled();
    {
        int n = 0;
        for (int g = std::min(g_sweep.lo, g_sweep.hi); g <= std::max(g_sweep.lo, g_sweep.hi); g += std::max(1, g_sweep.step)) n++;
        ImGui::TextDisabled("%d settings x %d rounds x %d s = about %.0f min", n, g_sweep.rounds, g_sweep.dwell_s + g_sweep.settle_s,
                            n * g_sweep.rounds * (g_sweep.dwell_s + g_sweep.settle_s) / 60.0);
    }
    if (!g_sweep.running) {
        ImGui::BeginDisabled(!g_airspy.is_open() || !g_engine.running());
        if (ImGui::Button("Start sweep")) sweep_start();
        ImGui::EndDisabled();
    } else {
        if (ImGui::Button("Stop sweep")) sweep_finish(false);
        ImGui::SameLine();
        ImGui::Text("Test %zu / %zu: gain %d", g_sweep.pos + 1, g_sweep.order.size(), g_sweep.order[g_sweep.pos]);
        ImGui::ProgressBar((float)g_sweep.pos / (float)std::max<size_t>(1, g_sweep.order.size()), ImVec2(-1, 0));
    }
    if (g_sweep.acc.empty()) return;

    // Results
    std::vector<double> xs, ys, es;
    for (const auto &kv : g_sweep.acc)
        if (kv.second.secs > 0) {
            xs.push_back(kv.first); ys.push_back(kv.second.good / kv.second.secs); es.push_back(kv.second.sem());
        }
    if (!xs.empty() && ImPlot::BeginPlot("##sweep", ImVec2(-1, 160), ImPlotFlags_NoLegend | ImPlotFlags_NoMouseText)) {
        ImPlot::SetupAxes(g_sweep.mode == 0 ? "Sensitivity gain" : "Linearity gain", "good frames/s",
                          ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
        ImPlot::PlotBars("good", xs.data(), ys.data(), (int)xs.size(), 0.7);
        {
            ImPlotSpec es_spec;
            es_spec.LineColor = ImVec4(1.0f, 1.0f, 1.0f, 0.85f);
            es_spec.LineWeight = 1.5f;
            ImPlot::PlotErrorBars("spread", xs.data(), ys.data(), es.data(), (int)xs.size(), es_spec);
        }
        if (g_sweep.best >= 0) {
            double bx = g_sweep.best, by = 0;
            for (size_t i = 0; i < xs.size(); i++) if (xs[i] == bx) by = ys[i];
            ImPlotSpec spec;
            spec.FillColor = ImVec4(0.3f, 1.0f, 0.4f, 1.0f);
            spec.LineColor = ImVec4(0.3f, 1.0f, 0.4f, 1.0f);
            ImPlot::PlotBars("best", &bx, &by, 1, 0.7, spec);
        }
        ImPlot::EndPlot();
    }
    if (ImGui::BeginTable("sweepres", 6, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                          ImVec2(0, 150))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Gain");
        ImGui::TableSetupColumn("Good/s");
        ImGui::TableSetupColumn("+/-");
        ImGui::TableSetupColumn("Parse err %");
        ImGui::TableSetupColumn("Demod fail %");
        ImGui::TableSetupColumn("Tested s");
        ImGui::TableHeadersRow();
        for (const auto &kv : g_sweep.acc) {
            const auto &a = kv.second;
            if (a.secs <= 0) continue;
            ImGui::TableNextRow();
            const bool isBest = kv.first == g_sweep.best;
            const ImVec4 c = isBest ? ImVec4(0.4f, 1.0f, 0.5f, 1.0f) : ImGui::GetStyleColorVec4(ImGuiCol_Text);
            ImGui::TableSetColumnIndex(0); ImGui::TextColored(c, "%d%s", kv.first, isBest ? "  best" : "");
            ImGui::TableSetColumnIndex(1); ImGui::TextColored(c, "%.1f", a.good / a.secs);
            ImGui::TableSetColumnIndex(2);
            if (a.rates.size() >= 2) ImGui::Text("%.1f", a.sem()); else ImGui::TextDisabled("-");
            ImGui::TableSetColumnIndex(3); ImGui::Text("%.1f", a.parsed ? 100.0 * a.errs / a.parsed : 0.0);
            ImGui::TableSetColumnIndex(4);
            ImGui::Text("%.1f", (a.demod_ok + a.demod_fail) ? 100.0 * a.demod_fail / (a.demod_ok + a.demod_fail) : 0.0);
            ImGui::TableSetColumnIndex(5); ImGui::Text("%.0f", a.secs);
        }
        ImGui::EndTable();
    }
    if (!g_sweep.running && g_sweep.best >= 0) {
        char lbl[48];
        snprintf(lbl, sizeof(lbl), "Apply best (%d)", g_sweep.best);
        if (ImGui::Button(lbl)) {
            sweep_apply_gain(g_sweep.best);
            add_log("[RADIO] Applied gain %d from sweep", g_sweep.best);
        }
        ImGui::SameLine();
        ImGui::TextDisabled("Overlapping error bars = no real difference; prefer the lower gain.");
    }
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

    {
        // Rates offered: what the selected device reports, else the generic list.
        std::vector<double> rates;
        bool isMini = false;
        if (!devs.empty()) {
            const int idx = (g_ap_device_idx >= 0 && g_ap_device_idx < (int)devs.size()) ? g_ap_device_idx : 0;
            rates = devs[idx].rates;
            isMini = devs[idx].name.find("Mini") != std::string::npos;
        }
        if (rates.empty()) rates.assign(kAirspyRates, kAirspyRates + kNumAirspyRates);
        if (isMini) {
            // Airspy lists 10 Msps for the Mini, but its firmware only
            // advertises 6 / 3, so offer it as an explicit opt-in.
            ImGui::BeginDisabled(g_airspy.is_open());
            ImGui::Checkbox("Unofficial 10 Msps mode", &g_ap_unofficial10);
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                ImGui::SetTooltip("Runs the Mini's ADC at 20 MHz for 10 Msps IQ (covers the whole\n"
                                  "Iridium band). Not officially supported: expect a few fixed\n"
                                  "spurs, more heat, and possibly dropped samples.\n"
                                  "Change while the Airspy is stopped.");
            if (g_ap_unofficial10 && std::find(rates.begin(), rates.end(), 10.0e6) == rates.end())
                rates.push_back(10.0e6);
            if (!g_ap_unofficial10 && g_ap_rate_hz == 10.0e6) g_ap_rate_hz = 0.0;
        }
        std::sort(rates.rbegin(), rates.rend());
        if (std::find(rates.begin(), rates.end(), g_ap_rate_hz) == rates.end())
            g_ap_rate_hz = rates.front(); // e.g. 10 Msps chosen, Mini plugged in -> 6
        char cur[32];
        std::snprintf(cur, sizeof(cur), "%.1f", g_ap_rate_hz / 1e6);
        ImGui::SetNextItemWidth(200);
        if (ImGui::BeginCombo("Sample rate (Msps)", cur)) {
            for (double r : rates) {
                char lbl[32];
                std::snprintf(lbl, sizeof(lbl), "%.1f", r / 1e6);
                if (ImGui::Selectable(lbl, r == g_ap_rate_hz) && r != g_ap_rate_hz) {
                    g_ap_rate_hz = r;
                    if (g_airspy.is_open()) {
                        apply_sample_rate(3, r);
                        g_ap_rate_hz = (double)g_airspy.sample_rate();
                    }
                }
            }
            ImGui::EndCombo();
        }
        {
            ImGui::TextDisabled("%.1f Msps covers %.2f-%.2f MHz around the VFO.", g_ap_rate_hz / 1e6,
                                g_vfo_hz / 1e6 - g_ap_rate_hz / 2e6, g_vfo_hz / 1e6 + g_ap_rate_hz / 2e6);
            // Ring alert, paging and messaging channels sit at 1626.0-1626.5 MHz.
            // Keep them ~0.6 MHz inside the band edge, where the filter is flat.
            const double upper = (1626.5e6 + 0.6e6) - g_ap_rate_hz / 2.0;
            if (std::fabs(g_vfo_hz - upper) > 1000.0) {
                char btn[64];
                std::snprintf(btn, sizeof(btn), "Centre on upper band (%.3f MHz)", upper / 1e6);
                if (ImGui::SmallButton(btn)) {
                    // Move the spectrum view with the VFO; otherwise sync_view()
                    // sees the old view outside the new passband and retunes back.
                    const double shift = (upper - g_vfo_hz) / 1e6;
                    retune_vfo(upper);
                    if (g_view_hi_mhz > g_view_lo_mhz) {
                        g_view_lo_mhz += shift;
                        g_view_hi_mhz += shift;
                        g_view_dirty = true;
                    } else {
                        g_view_reset = true;
                    }
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Covers the ring alert / paging / messaging channels\n"
                                      "(1626.0-1626.5 MHz) plus the duplex channels below them.");
            }
        }
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
    draw_gain_sweep();

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
            add_log("[RADIO] Airspy stopped");
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
            add_log("[RADIO] RTL-SDR stopped");
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
    ImGui::SetNextWindowSize(ImVec2(460, 680), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Control###Source", &g_show_source)) {
        if (ImGui::CollapsingHeader("Input", ImGuiTreeNodeFlags_DefaultOpen)) {
        const char *kinds[] = {"IQ file", "SDRplay", "HackRF", "Airspy", "RTL-SDR"};
        ImGui::SetNextItemWidth(160);
        if (ImGui::Combo("Type", &g_source_kind, kinds, 5)) {
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
            // Raw IQ recording of the live radio.
            const auto &rec = g_engine.iq_recorder();
            if (!rec.active()) {
                ImGui::BeginDisabled(!g_engine.running());
                if (ImGui::Button("Record IQ")) {
                    std::string err;
                    if (g_engine.start_iq_recording("recordings/iq", err))
                        add_log("[RADIO] IQ recording started: %s", rec.current_file().c_str());
                    else add_log("[RADIO] IQ recording failed: %s", err.c_str());
                }
                ImGui::EndDisabled();
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                    const double rate = active_source()->sample_rate();
                    ImGui::SetTooltip("Record the radio's raw IQ to recordings/iq as 16-bit WAV\n"
                                      "(plays back in IridiScope with its real time and frequency).\n"
                                      "Uses %.0f MB per second at the current rate; a new file\n"
                                      "starts every ~2 GB.", rate * 4 / 1e6);
                }
            } else {
                if (ImGui::Button("Stop IQ recording")) {
                    g_engine.stop_iq_recording();
                    add_log("[RADIO] IQ recording stopped: %.1f s, %.2f GB in %d file(s)%s", rec.seconds_recorded(),
                            rec.bytes_written() / 1e9, rec.files_written(),
                            rec.dropped_blocks() ? " (some blocks dropped: disk too slow)" : "");
                }
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "REC %.0f s  %.2f GB", rec.seconds_recorded(),
                                   rec.bytes_written() / 1e9);
                if (rec.dropped_blocks()) {
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.35f, 1.0f), "(%llu blocks dropped: disk behind)",
                                       (unsigned long long)rec.dropped_blocks());
                }
                std::string f = rec.current_file();
                const size_t sl = f.find_last_of("/\\");
                ImGui::TextDisabled("%s", sl == std::string::npos ? f.c_str() : f.c_str() + sl + 1);
            }
        }
        }
        ImGui::Separator();
        if (ImGui::CollapsingHeader("Playback", ImGuiTreeNodeFlags_DefaultOpen))
            draw_playback_contents();
        ImGui::Separator();
        if (ImGui::CollapsingHeader("Stats", ImGuiTreeNodeFlags_DefaultOpen))
            draw_stats_contents();

        ImGui::Separator();
        if (ImGui::CollapsingHeader("Wireshark output")) {
            if (ImGui::Checkbox("Enable GSMTAP", &g_gsmtap_on)) {
                if (g_gsmtap_on) {
                    std::string err;
                    if (!iridi::gsmtap_start(g_gsmtap_host, g_gsmtap_port, err)) {
                        g_gsmtap_on = false;
                        g_gsmtap_status = err;
                        add_log("GSMTAP: %s", err.c_str());
                    } else {
                        g_gsmtap_status = "sending";
                    }
                } else {
                    iridi::gsmtap_stop();
                    g_gsmtap_status = "stopped";
                }
            }
            ImGui::SetNextItemWidth(150);
            ImGui::InputText("Host", g_gsmtap_host, sizeof(g_gsmtap_host));
            ImGui::SetNextItemWidth(80);
            if (ImGui::InputInt("Port", &g_gsmtap_port))
                g_gsmtap_port = std::clamp(g_gsmtap_port, 1, 65535);
            if (ImGui::Button("Apply")) {
                if (g_gsmtap_on) {
                    iridi::gsmtap_stop();
                    std::string err;
                    if (iridi::gsmtap_start(g_gsmtap_host, g_gsmtap_port, err)) {
                        g_gsmtap_status = "sending";
                    } else {
                        g_gsmtap_on = false;
                        g_gsmtap_status = err;
                    }
                }
            }
            ImGui::SameLine();
            ImGui::TextDisabled("%s", g_gsmtap_status.c_str());
            ImGui::TextDisabled("Wireshark filter: gsmtap  |  %llu packets",
                                (unsigned long long)iridi::gsmtap_packets());
            ImGui::TextDisabled("ICMP port-unreachable is normal until a local listener binds UDP %d.",
                                g_gsmtap_port);
        }

        ImGui::Separator();
        if (ImGui::CollapsingHeader("Data")) {
            if (ImGui::Button("Save decoded data...")) {
                std::string path = save_file_dialog();
                if (!path.empty()) save_decoded_csv(path);
            }
            if (!g_save_status.empty())
                ImGui::TextWrapped("%s", g_save_status.c_str());
            ImGui::TextDisabled("CSV: ACARS, SBD, SMS, pager, L3 and parsed frames");
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
        ImGui::TextWrapped("macOS/Linux build support by Andy Lawendel (@lawendel), PR #10.");
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

    // Layout: Control on the left, Map in the centre, Messages/Log/Satellites/
    // Paging/Spectrum on the right, and the decoders (Waterfall, Voice, ACARS,
    // ...) across the full width at the bottom.
    ImGuiID top = dockspace_id;
    ImGuiID bottom = ImGui::DockBuilderSplitNode(top, ImGuiDir_Down, 0.36f, nullptr, &top);
    ImGuiID center = top;
    ImGuiID left   = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.19f, nullptr, &center);
    ImGuiID right  = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.225f, nullptr, &center);

    ImGui::DockBuilderDockWindow("Control###Source", left);
    ImGui::DockBuilderDockWindow("Map", center);
    for (const char *w : {"Messages", "Log", "Satellites", "Paging", "Spectrum"})
        ImGui::DockBuilderDockWindow(w, right);
    for (const char *w : {"Waterfall", "Bursts", "Voice", "ACARS", "Pager", "SMS", "SBD",
                          "Signaling", "Broadcast", "ITL"})
        ImGui::DockBuilderDockWindow(w, bottom);
    g_layout_focus_frames = 2;
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

static int pager_dump(const char *path, double seconds) {
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
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    auto msgs = eng.pager().messages();
    printf("pager_messages=%zu\n", msgs.size());
    for (const auto &m : msgs) {
        printf("%.6f ric=%07d fmt=%d seq=%d parts=%d ok=%d freq=%.0f text=%s\n",
               m.timestamp_ns / 1e9, m.ric, m.fmt, m.seq, m.parts, m.ok ? 1 : 0,
               m.frequency, m.text.c_str());
    }
    eng.stop();
    return 0;
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
    for (auto &d : devs) {
        printf("  [%d] %s  sn=%s", d.index, d.name.c_str(), d.serial.c_str());
        if (!d.ports.empty()) {
            printf("  ports=");
            for (size_t i = 0; i < d.ports.size(); ++i)
                printf("%s%s", i ? "|" : "", d.ports[i].c_str());
        }
        printf("\n");
    }
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

// ---- Saved settings ---------------------------------------------------------
// Input, radio, decoding, voice and display settings are stored in
// iridiscope.ini (section [IridiScope][Settings]) next to the window layout,
// so IridiScope starts the way it was left.
namespace {
constexpr int kLayoutVersion = 3;   // bump to apply a new default layout once
int g_saved_layout_version = 0;      // from the ini (0 = none saved)
bool g_settings_loaded = false;

struct SettingRef {
    const char *key;
    enum Kind { I, F, D, B, S } kind;
    void *ptr;
    size_t cap; // for strings (char arrays)
};

std::vector<SettingRef> settings_table() {
    using K = SettingRef::Kind;
    return {
        {"source_kind", K::I, &g_source_kind, 0},
        {"vfo_hz", K::D, &g_vfo_hz, 0},
        {"sdr_rate_idx", K::I, &g_sdr_rate_idx, 0}, {"sdr_device_idx", K::I, &g_sdr_device_idx, 0},
        {"sdr_antenna", K::I, &g_sdr_antenna, 0}, {"sdr_agc", K::B, &g_sdr_agc, 0},
        {"sdr_gr", K::F, &g_sdr_gr, 0}, {"sdr_bias", K::B, &g_sdr_bias, 0},
        {"hrf_rate_idx", K::I, &g_hrf_rate_idx, 0}, {"hrf_device_idx", K::I, &g_hrf_device_idx, 0},
        {"hrf_lna", K::I, &g_hrf_lna, 0}, {"hrf_vga", K::I, &g_hrf_vga, 0},
        {"hrf_amp", K::B, &g_hrf_amp, 0}, {"hrf_bias", K::B, &g_hrf_bias, 0},
        {"ap_rate_hz", K::D, &g_ap_rate_hz, 0}, {"ap_rate_idx", K::I, &g_ap_rate_idx, 0},
        {"ap_unofficial10", K::B, &g_ap_unofficial10, 0}, {"ap_device_idx", K::I, &g_ap_device_idx, 0},
        {"ap_gain_mode", K::I, &g_ap_gain_mode, 0}, {"ap_sense", K::I, &g_ap_sense, 0},
        {"ap_linear", K::I, &g_ap_linear, 0}, {"ap_lna", K::I, &g_ap_lna, 0},
        {"ap_mixer", K::I, &g_ap_mixer, 0}, {"ap_vga", K::I, &g_ap_vga, 0},
        {"ap_lna_agc", K::B, &g_ap_lna_agc, 0}, {"ap_mixer_agc", K::B, &g_ap_mixer_agc, 0},
        {"ap_bias", K::B, &g_ap_bias, 0},
        {"rtl_rate_idx", K::I, &g_rtl_rate_idx, 0}, {"rtl_device_idx", K::I, &g_rtl_device_idx, 0},
        {"rtl_agc", K::B, &g_rtl_agc, 0}, {"rtl_gain", K::F, &g_rtl_gain, 0},
        {"rtl_ppm", K::I, &g_rtl_ppm, 0}, {"rtl_bias", K::B, &g_rtl_bias, 0},
        {"queue_budget_idx", K::I, &g_queue_budget_idx, 0},
        {"backlog_idx", K::I, &g_backlog_idx, 0}, {"reject_wide", K::B, &g_reject_wide, 0}, {"protect_radio", K::B, &g_protect_radio, 0},
        {"voice_hide_failed", K::B, &g_voice_hide_failed, 0}, {"voice_max_err", K::F, &g_voice_max_err, 0},
        {"voice_hide_short", K::B, &g_voice_hide_short, 0}, {"voice_min_s", K::F, &g_voice_min_s, 0}, {"voice_autoplay", K::B, &g_autoplay, 0},
        {"log_status", K::B, &g_log_cat[0], 0}, {"log_alerts", K::B, &g_log_cat[1], 0}, {"log_radio", K::B, &g_log_cat[2], 0},
        {"log_voice", K::B, &g_log_cat[3], 0}, {"log_handoffs", K::B, &g_log_cat[4], 0}, {"log_other", K::B, &g_log_cat[5], 0},
        {"log_status_interval", K::I, &g_status_interval_idx, 0},
        {"fft_idx", K::I, &g_fft_idx, 0}, {"avg_alpha", K::F, &g_avg_alpha, 0},
        {"autoscale", K::B, &g_autoscale, 0}, {"db_min", K::F, &g_db_min, 0}, {"db_max", K::F, &g_db_max, 0},
        {"dc_block", K::B, &g_dc_block, 0},
        {"gsmtap_host", K::S, g_gsmtap_host, sizeof(g_gsmtap_host)}, {"gsmtap_port", K::I, &g_gsmtap_port, 0},
        {"lookup_enabled", K::B, &g_lookup_enabled, 0},
        {"sweep_lo", K::I, &g_sweep.lo, 0}, {"sweep_hi", K::I, &g_sweep.hi, 0},
        {"sweep_step", K::I, &g_sweep.step, 0}, {"sweep_dwell_s", K::I, &g_sweep.dwell_s, 0},
        {"sweep_rounds", K::I, &g_sweep.rounds, 0},
        {"show_map", K::B, &g_show_map, 0}, {"show_source", K::B, &g_show_source, 0},
        {"show_spectrum", K::B, &g_show_spectrum, 0}, {"show_waterfall", K::B, &g_show_waterfall, 0},
        {"show_bursts", K::B, &g_show_bursts, 0}, {"show_messages", K::B, &g_show_messages, 0},
        {"show_log", K::B, &g_show_log, 0}, {"show_sats", K::B, &g_show_sats, 0},
        {"show_voice", K::B, &g_show_voice, 0}, {"show_acars", K::B, &g_show_acars, 0},
        {"show_pager", K::B, &g_show_pager, 0}, {"show_sms", K::B, &g_show_sms, 0},
        {"show_sbd", K::B, &g_show_sbd, 0}, {"show_signaling", K::B, &g_show_signaling, 0},
        {"show_broadcast", K::B, &g_show_broadcast, 0}, {"show_itl", K::B, &g_show_itl, 0},
        {"show_paging", K::B, &g_show_paging, 0},
        {"map_online", K::B, &g_map_online, 0},
        {"map_night", K::B, &g_map_night, 0},
        {"msgdb_on", K::B, &g_msgdb_on, 0},
        {"voice_hide_ann", K::B, &g_voice_hide_ann, 0}, {"voice_sort_ann", K::B, &g_voice_sort_ann, 0},
        {"sat_hide_after_s", K::F, &g_sat_hide_after_s, 0}, {"sat_show_stale", K::B, &g_sat_show_stale, 0},
    };
}

// Settings held outside plain globals.
bool g_set_sync_tol = true, g_set_follow = true, g_set_realtime = false;

void *settings_read_open(ImGuiContext *, ImGuiSettingsHandler *, const char *name) {
    return std::strcmp(name, "Settings") == 0 ? (void *)1 : nullptr;
}

void settings_read_line(ImGuiContext *, ImGuiSettingsHandler *, void *, const char *line) {
    const char *eq = std::strchr(line, '=');
    if (!eq) return;
    const std::string key(line, eq - line);
    const char *v = eq + 1;
    g_settings_loaded = true;
    if (key == "layout_version") { g_saved_layout_version = std::atoi(v); return; }
    if (key == "sync_tolerance") { g_set_sync_tol = std::atoi(v) != 0; return; }
    if (key == "follow_handoffs") { g_set_follow = std::atoi(v) != 0; return; }
    if (key == "realtime") { g_set_realtime = std::atoi(v) != 0; return; }
    for (const auto &r : settings_table()) {
        if (key != r.key) continue;
        switch (r.kind) {
        case SettingRef::I: *(int *)r.ptr = std::atoi(v); break;
        case SettingRef::F: *(float *)r.ptr = (float)std::atof(v); break;
        case SettingRef::D: *(double *)r.ptr = std::atof(v); break;
        case SettingRef::B: *(bool *)r.ptr = std::atoi(v) != 0; break;
        case SettingRef::S: std::snprintf((char *)r.ptr, r.cap, "%s", v); break;
        }
        return;
    }
}

void settings_apply_all(ImGuiContext *, ImGuiSettingsHandler *) {
    // Keep loaded values in their valid ranges.
    g_source_kind = std::clamp(g_source_kind, 0, 4);
    g_sdr_rate_idx = std::clamp(g_sdr_rate_idx, 0, kNumSdrplayRates - 1);
    g_hrf_rate_idx = std::clamp(g_hrf_rate_idx, 0, kNumHackrfRates - 1);
    g_ap_rate_idx = std::clamp(g_ap_rate_idx, 0, kNumAirspyRates - 1);
    g_rtl_rate_idx = std::clamp(g_rtl_rate_idx, 0, kNumRtlRates - 1);
    g_ap_gain_mode = std::clamp(g_ap_gain_mode, 0, 2);
    g_ap_sense = std::clamp(g_ap_sense, 0, 21);
    g_ap_linear = std::clamp(g_ap_linear, 0, 21);
    g_queue_budget_idx = std::clamp(g_queue_budget_idx, 0, 3);
    g_backlog_idx = std::clamp(g_backlog_idx, 0, 3);
    ir_set_reject_wide(g_reject_wide);
    iridi_set_boost(g_protect_radio);
    iridi_apply_thread_priority(IRIDI_PRIO_NORMAL);
    g_voice_min_s = std::clamp(g_voice_min_s, 0.0f, 120.0f);
    g_fft_idx = std::clamp(g_fft_idx, 0, kNumFftSizes - 1);
    if (!(g_vfo_hz > 1e6)) g_vfo_hz = kDefaultVfoHz;
    irid::irid_uw_ec.store(g_set_sync_tol);
    g_engine.voices().set_follow_handoffs(g_set_follow);
    g_engine.set_realtime(g_set_realtime);
    g_lookup.set_enabled(g_lookup_enabled);
}

void settings_write_all(ImGuiContext *, ImGuiSettingsHandler *handler, ImGuiTextBuffer *buf) {
    buf->appendf("[%s][Settings]\n", handler->TypeName);
    buf->appendf("layout_version=%d\n", kLayoutVersion);
    buf->appendf("sync_tolerance=%d\n", irid::irid_uw_ec.load() ? 1 : 0);
    buf->appendf("follow_handoffs=%d\n", g_engine.voices().follow_handoffs() ? 1 : 0);
    buf->appendf("realtime=%d\n", g_engine.realtime() ? 1 : 0);
    for (const auto &r : settings_table()) {
        switch (r.kind) {
        case SettingRef::I: buf->appendf("%s=%d\n", r.key, *(int *)r.ptr); break;
        case SettingRef::F: buf->appendf("%s=%.6g\n", r.key, *(float *)r.ptr); break;
        case SettingRef::D: buf->appendf("%s=%.3f\n", r.key, *(double *)r.ptr); break;
        case SettingRef::B: buf->appendf("%s=%d\n", r.key, *(bool *)r.ptr ? 1 : 0); break;
        case SettingRef::S: buf->appendf("%s=%s\n", r.key, (const char *)r.ptr); break;
        }
    }
    buf->append("\n");
}

void register_settings_handler() {
    ImGuiSettingsHandler h;
    h.TypeName = "IridiScope";
    h.TypeHash = ImHashStr("IridiScope");
    h.ReadOpenFn = settings_read_open;
    h.ReadLineFn = settings_read_line;
    h.ApplyAllFn = settings_apply_all;
    h.WriteAllFn = settings_write_all;
    ImGui::AddSettingsHandler(&h);
}

// Settings changes don't move windows, so tell ImGui to save when they change.
void mark_settings_dirty_if_changed() {
    static std::string last;
    ImGuiTextBuffer b;
    ImGuiSettingsHandler h; h.TypeName = "IridiScope";
    settings_write_all(nullptr, &h, &b);
    if (last != b.c_str()) {
        if (!last.empty()) ImGui::MarkIniSettingsDirty();
        last = b.c_str();
    }
}
} // namespace

// ---- Log: periodic status summary and automatic alerts --------------------

// Whole-machine CPU busy % and IridiScope's share of the machine since the
// previous call (Windows; false elsewhere or on the first call).
struct CpuSampler {
    uint64_t sys_busy = 0, sys_total = 0, proc = 0;
    bool primed = false;
    bool sample(double &sys_pct, double &proc_pct) {
#ifdef _WIN32
        FILETIME idle, kern, user, c, e, pk, pu;
        if (!GetSystemTimes(&idle, &kern, &user) || !GetProcessTimes(GetCurrentProcess(), &c, &e, &pk, &pu)) return false;
        auto u64 = [](const FILETIME &f) { return ((uint64_t)f.dwHighDateTime << 32) | f.dwLowDateTime; };
        const uint64_t total = u64(kern) + u64(user);          // kernel time includes idle
        const uint64_t busy = total - u64(idle), pr = u64(pk) + u64(pu);
        const bool ok = primed && total > sys_total;
        if (ok) {
            sys_pct = 100.0 * (double)(busy - sys_busy) / (double)(total - sys_total);
            proc_pct = 100.0 * (double)(pr - proc) / (double)(total - sys_total);
        }
        sys_busy = busy; sys_total = total; proc = pr; primed = true;
        return ok;
#else
        (void)sys_pct; (void)proc_pct;
        return false;
#endif
    }
};

static double process_ram_mb() {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS pmc{};
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) return pmc.WorkingSetSize / 1048576.0;
#endif
    return -1.0;
}

void log_status_tick() {
    using clk = std::chrono::steady_clock;
    const auto now = clk::now();
    // Snapshot of the counters at the last summary.
    struct Snap {
        clk::time_point t; uint64_t parsed = 0, errs = 0, bursts = 0, wide = 0, skipped = 0, app = 0, drv = 0;
        uint64_t msgs[7] = {}; uint64_t ann = 0, ho_seen = 0, ho_fol = 0, vrec = 0; double speech = 0;
    };
    static Snap last;
    static bool have_last = false, was_running = false;
    static std::deque<double> good_hist, burst_hist;   // rates of recent summaries
    static float last_floor = 0.0f;
    static bool have_floor = false;
    static CpuSampler cpu_status, cpu_alert;
    static clk::time_point next_sec = now;
    static int sys_hot_s = 0, det_hot_s = 0, silent_s = 0;
    static clk::time_point cool_cpu{}, cool_det{}, cool_silent{}, cool_drop{}, cool_floor{};
    static uint64_t silent_ref_good = 0;
    const auto cooled = [&](clk::time_point &t) {
        if (t != clk::time_point{} && now - t < std::chrono::minutes(5)) return false;
        t = now; return true;
    };

    const bool running = g_engine.running() && !g_engine.paused();
    const auto *live = active_live_source();
    auto take = [&]() {
        Snap n; n.t = now;
        n.parsed = g_engine.parsed_count(); n.errs = g_engine.parse_error_count();
        n.bursts = g_engine.stats().bursts; n.wide = ir_wide_dropped(); n.skipped = g_engine.skipped_bursts();
        n.app = live ? live->dropped_samples() : 0; n.drv = live ? live->driver_dropped_samples() : 0;
        for (int i = 1; i <= 6; i++) n.msgs[i] = iridi::MsgDb::instance().count(i);
        n.ann = g_engine.voices().announcements();
        n.ho_seen = g_engine.voices().handoffs_seen(); n.ho_fol = g_engine.voices().handoffs_followed();
        n.vrec = g_engine.voice_recovered(); n.speech = g_engine.voices().speech_total_s();
        return n;
    };
    if (!running) { was_running = false; have_last = false; good_hist.clear(); burst_hist.clear(); have_floor = false; return; }
    if (!was_running) {
        was_running = true; last = take(); have_last = true;
        { double a, b; cpu_status.sample(a, b); }   // prime
        sys_hot_s = det_hot_s = silent_s = 0; silent_ref_good = last.parsed - last.errs;
        return;
    }

    // ---- once a second: alerts ----
    if (now >= next_sec) {
        next_sec = now + std::chrono::seconds(1);
        double sys = 0, proc = 0;
        if (cpu_alert.sample(sys, proc)) {
            sys_hot_s = sys >= 90.0 ? sys_hot_s + 1 : 0;
            if (sys_hot_s == 10 && cooled(cool_cpu))
                add_log("[ALERT] Computer CPU at %.0f%% for 10 s (IridiScope %.0f%% of it): another program is "
                        "using the CPU; decoding may fall behind", sys, proc);
        }
        det_hot_s = g_det_load > 90.0 ? det_hot_s + 1 : 0;
        if (det_hot_s == 10 && cooled(cool_det))
            add_log("[ALERT] Detector load %.0f%% for 10 s (CPU %.0f%%, bursts/s %.0f): the radio thread is close "
                    "to falling behind", g_det_load, std::max(0.0, g_det_cpu), g_bursts_ps);
        const uint64_t good = g_engine.parsed_count() - std::min(g_engine.parsed_count(), g_engine.parse_error_count());
        if (good != silent_ref_good) { silent_ref_good = good; silent_s = 0; }
        else if (++silent_s == 30 && cooled(cool_silent))
            add_log("[ALERT] No good frames for 30 s (bursts/s %.0f, noise floor %.1f dB): check antenna, LNA power "
                    "(bias-T), cable and gain", std::max(0.0, g_bursts_ps), (double)g_engine.detector_noise_floor());
    }

    // ---- every interval: summary ----
    static const int kIv[3] = {30, 60, 300};
    const int iv = kIv[std::clamp(g_status_interval_idx, 0, 2)];
    if (!have_last || now - last.t < std::chrono::seconds(iv)) return;
    const Snap n = take();
    const double dt = std::chrono::duration<double>(n.t - last.t).count();
    if (dt <= 0) return;
    auto d = [](uint64_t a, uint64_t b) { return a >= b ? a - b : a; };   // counters restart on a new run
    const uint64_t parsed = d(n.parsed, last.parsed), errs = d(n.errs, last.errs);
    const uint64_t good = parsed - std::min(parsed, errs);
    const double good_ps = good / dt, bursts_ps = d(n.bursts, last.bursts) / dt, wide_ps = d(n.wide, last.wide) / dt;
    const double rate = live && live->sample_rate() > 0 ? (double)live->sample_rate() : 1.0;
    const uint64_t skipped = d(n.skipped, last.skipped), app = d(n.app, last.app), drv = d(n.drv, last.drv);
    int sats = 0;
    const uint64_t newest = g_engine.last_timestamp();
    for (const auto &st : g_engine.satellites())
        if (newest && st.last_seen_ns + (uint64_t)iv * 1000000000ULL >= newest) sats++;
    const float floor_db = g_engine.detector_noise_floor();
    double sys = -1, proc = -1;
    const bool have_cpu = cpu_status.sample(sys, proc);
    const double ram = process_ram_mb();
    char cpu_s[96] = "";
    if (have_cpu) snprintf(cpu_s, sizeof(cpu_s), " | CPU: computer %.0f%%, IridiScope %.0f%%", sys, proc);
    char ram_s[48] = "";
    if (ram >= 0) snprintf(ram_s, sizeof(ram_s), ", RAM %.0f MB", ram);
    char ivs[16]; if (iv < 60) snprintf(ivs, sizeof(ivs), "%d s", iv); else snprintf(ivs, sizeof(ivs), "%d min", iv / 60);
    add_log("[STATUS] Last %s: good %.0f/s, bursts %.0f/s%s, parse errors %.1f%%, satellites %d | detector %.0f%% "
            "(CPU %.0f%%) | skipped bursts %llu, drops: IridiScope %.0f ms, driver %.0f ms%s%s | noise floor %.1f dB, %s",
            ivs, good_ps, bursts_ps,
            g_reject_wide ? (" (+" + std::to_string((int)std::lround(wide_ps)) + "/s wideband ignored)").c_str() : "",
            parsed ? 100.0 * errs / parsed : 0.0, sats, std::max(0.0, g_det_load), std::max(0.0, g_det_cpu),
            (unsigned long long)skipped, 1000.0 * app / rate, 1000.0 * drv / rate, cpu_s, ram_s, (double)floor_db,
            gain_desc().c_str());
    const uint64_t calls = d(n.msgs[5], last.msgs[5]);
    add_log("[TRAFFIC] Last %s: voice calls saved %llu (%.0f s speech, %llu announcements), voice frames recovered %llu, "
            "handoffs %llu (%llu followed) | ACARS %llu, SMS %llu, SBD %llu, pager %llu, ring alerts %llu",
            ivs, (unsigned long long)calls, std::max(0.0, n.speech - last.speech), (unsigned long long)d(n.ann, last.ann),
            (unsigned long long)d(n.vrec, last.vrec), (unsigned long long)d(n.ho_seen, last.ho_seen),
            (unsigned long long)d(n.ho_fol, last.ho_fol), (unsigned long long)d(n.msgs[1], last.msgs[1]),
            (unsigned long long)d(n.msgs[2], last.msgs[2]), (unsigned long long)d(n.msgs[3], last.msgs[3]),
            (unsigned long long)d(n.msgs[4], last.msgs[4]), (unsigned long long)d(n.msgs[6], last.msgs[6]));

    // Sudden change in decoded traffic vs. the recent average.
    if (good_hist.size() >= 3) {
        double avg = 0, bavg = 0;
        for (double g : good_hist) avg += g;
        for (double b : burst_hist) bavg += b;
        avg /= good_hist.size(); bavg /= burst_hist.size();
        if (avg >= 50.0 && good_ps < 0.5 * avg && cooled(cool_drop))
            add_log("[ALERT] Good frames fell to %.0f/s from ~%.0f/s. %s", good_ps, avg,
                    bursts_ps < 0.6 * bavg
                        ? "Bursts fell too: fewer satellites in view, or antenna/LNA/cable/gain."
                        : "Bursts didn't fall: decoding is failing (interference, overload, or CPU - see skipped/drops).");
    }
    good_hist.push_back(good_ps); burst_hist.push_back(bursts_ps);
    while (good_hist.size() > 5) { good_hist.pop_front(); burst_hist.pop_front(); }
    // Noise floor jump with the gain unchanged: antenna, LNA power, interference.
    const bool gain_recent = now - g_gain_changed_at < std::chrono::seconds(iv + 5);
    if (have_floor && !gain_recent && std::fabs(floor_db - last_floor) >= 3.0f && cooled(cool_floor))
        add_log("[ALERT] Noise floor moved %+.1f dB (%.1f -> %.1f dB) with the gain unchanged: antenna, LNA power "
                "(bias-T), cable, or local interference", (double)(floor_db - last_floor), (double)last_floor, (double)floor_db);
    last_floor = floor_db; have_floor = true;
    last = n;
}

int main(int argc, char **argv) {
    if (getenv("IRIDI_DEBUG")) irid::irid_debug = 1;
    if (argc >= 2) {
        std::string a1 = argv[1];
        if (a1 == "--info" || a1 == "--parse" || a1 == "--frame" || a1 == "--scan" ||
            a1 == "--sdr-list" || a1 == "--sdr-scan" || a1 == "--hrf-list" || a1 == "--hrf-scan" ||
            a1 == "--ap-list" || a1 == "--ap-scan" || a1 == "--rtl-list" || a1 == "--rtl-scan" ||
            a1 == "--check-update" || a1 == "--lookup" || a1 == "--sms" || a1 == "--raw" ||
            a1 == "--pager")
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
    if (argc >= 3 && std::string(argv[1]) == "--pager")
        return pager_dump(argv[2], argc >= 4 ? atof(argv[3]) : 60.0);

#ifndef _WIN32
    ensure_writable_workdir();
#endif
#if defined(__APPLE__)
    // GLFW would otherwise chdir into the .app's Contents/Resources, so the
    // layout and voice recordings would be written inside the app bundle.
    glfwInitHint(GLFW_COCOA_CHDIR_RESOURCES, GLFW_FALSE);
#endif
    if (!glfwInit()) { fprintf(stderr, "Failed to initialize GLFW\n"); return 1; }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#if defined(__APPLE__)
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE); // required on macOS
#endif

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
    register_settings_handler();

    // Seed the window layout from the bundled default on first run, so the
    // shipped docking arrangement is the default.
    const char *ini_path = "iridiscope.ini";
    bool force_default = false;
    // No saved layout yet: build the default arrangement in code.
#ifdef _WIN32
    if (GetFileAttributesA(ini_path) == INVALID_FILE_ATTRIBUTES) force_default = true;
#else
    if (!file_exists(ini_path)) force_default = true;
#endif
    io.IniFilename = ini_path;
    bool dock_built = false;
    // Every launch opens on the Voice and Messages tabs (and the Map).
    g_layout_focus_frames = 3;
    apply_style();

    ImGui_ImplGlfw_InitForOpenGL(g_window, true);
    ImGui_ImplOpenGL3_Init("#version 150");

    add_log("IridiScope - Iridium signal analysis workstation");
    add_log("OpenGL: %s", glGetString(GL_VERSION) ? (const char *)glGetString(GL_VERSION) : "?");

    g_map_tex = load_map_texture(g_map_w, g_map_h);
    // Defaults (settings_apply_all re-applies saved values once the .ini loads).
    iridi_set_boost(g_protect_radio);
    iridi_apply_thread_priority(IRIDI_PRIO_NORMAL);
    ir_set_reject_wide(g_reject_wide);
    iridi::MsgDb::instance().set_enabled(g_msgdb_on);
    {
        const int n = iridi::AnnouncementMatcher::instance().load("assets/announcements");
        add_log("Announcement recognition: %d reference recording(s) loaded from assets/announcements", n);
        g_engine.voices().set_sort_announcements(g_voice_sort_ann);
    }
    g_tiles.start();
    g_tiles_night.configure(L"gibs.earthdata.nasa.gov",
                            "/wmts/epsg3857/best/VIIRS_Black_Marble/default/2016-01-01/GoogleMapsCompatible_Level8/%d/%d/%d.png",
                            "mapcache/blackmarble", "png");
    g_tiles_night.start(2);

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
        if (g_sort_done.load() >= 0) {
            add_log("Sorting existing recordings: %d of %d WAVs were network announcements, moved to "
                    "recordings/announcements/", g_sort_moved.load(), g_sort_done.load());
            g_sort_done = -1;
        }
        g_tiles.upload_ready();
        g_tiles_night.upload_ready();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiID dock_id = ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());
        // A layout saved by an older version gets the new default once.
        if (!dock_built && g_saved_layout_version < kLayoutVersion) force_default = true;
        if (force_default && !dock_built) {
            build_default_dock_layout(dock_id); dock_built = true;
            g_saved_layout_version = kLayoutVersion;
            ImGui::MarkIniSettingsDirty();
        }
        if (g_reset_layout) { g_reset_layout = false; build_default_dock_layout(dock_id); add_log("Layout reset to default"); }

        // After building the default layout, pick the visible tab in each
        // group (focus selects a docked window's tab). A user's own saved
        // layout is never rearranged.
        if (g_layout_focus_frames > 0 && --g_layout_focus_frames == 0) {
            ImGui::SetWindowFocus("Voice");
            ImGui::SetWindowFocus("Messages");
            ImGui::SetWindowFocus("Map");
        }

        { static int n = 0; if (++n % 30 == 0) mark_settings_dirty_if_changed(); }
        draw_menu_bar();
        draw_map_window();
        draw_spectrum_window();
        draw_waterfall_window();
        draw_bursts_window();
        draw_messages_window();
        draw_log_window();
        sweep_tick(); // runs even when the Control panel is hidden
        update_detector_load();
        update_event_tracking();
        {
            // Log sample drops as they happen (at most once a second) so their
            // timing is visible: startup, traffic peaks, other programs, ...
            static uint64_t last_app = 0, last_drv = 0;
            static const void *last_src = nullptr;
            static auto last_check = std::chrono::steady_clock::now();
            const auto now = std::chrono::steady_clock::now();
            const auto *live = active_live_source();
            if (live != last_src) { last_src = live; last_app = live ? live->dropped_samples() : 0;
                                    last_drv = live ? live->driver_dropped_samples() : 0; }
            if (live && now - last_check >= std::chrono::seconds(1)) {
                last_check = now;
                const uint64_t a = live->dropped_samples(), d = live->driver_dropped_samples();
                if (a < last_app) last_app = 0;   // restarted
                if (d < last_drv) last_drv = 0;
                const double rate = live->sample_rate() > 0 ? (double)live->sample_rate() : 1.0;
                if (a > last_app)
                    add_log("[ALERT] Dropped %llu samples (%.0f ms) in IridiScope: the detector fell behind the radio; "
                            "detector load %.0f%% (CPU %.0f%%), bursts/s %.0f",
                            (unsigned long long)(a - last_app), 1000.0 * (a - last_app) / rate,
                            g_det_load, std::max(0.0, g_det_cpu), g_bursts_ps);
                {
                    static uint64_t last_skip = 0;
                    const uint64_t sk = g_engine.skipped_bursts();
                    if (sk < last_skip) last_skip = 0;
                    if (sk > last_skip)
                        add_log("[ALERT] Skipped %llu bursts: decoders more than %.0f s behind (bursts/s %.0f)",
                                (unsigned long long)(sk - last_skip), kBacklogS[std::clamp(g_backlog_idx, 0, 3)],
                                g_bursts_ps);
                    last_skip = sk;
                }
                if (d > last_drv)
                    add_log("[ALERT] Dropped %llu samples (%.0f ms) in the Airspy driver (before IridiScope)",
                            (unsigned long long)(d - last_drv), 1000.0 * (d - last_drv) / rate);
                last_app = a; last_drv = d;
            }
        }
        {
            // Any gain change (manual or sweep, any SDR) shifts the whole
            // spectrum; the burst detector's learned noise floor only updates
            // when no bursts are active, so reset it rather than wait.
            static std::string last_gain;
            // Reset only after samples captured at the old gain have drained
            // (IridiScope's buffer plus the driver's), so the new noise floor
            // is learned purely at the new gain. Slider drags push it out.
            static bool reset_pending = false;
            static std::chrono::steady_clock::time_point reset_at;
            char sig[160];
            snprintf(sig, sizeof(sig), "%d|%d|%.1f|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d",
                     g_source_kind, (int)g_sdr_agc, (double)g_sdr_gr, g_hrf_lna, g_hrf_vga, (int)g_hrf_amp,
                     g_ap_gain_mode, g_ap_sense, g_ap_linear, g_ap_lna, g_ap_mixer, g_ap_vga,
                     (int)g_ap_lna_agc, (int)g_ap_mixer_agc, (int)g_rtl_agc, (int)g_rtl_gain);
            if (last_gain != sig) {
                if (!last_gain.empty()) {
                    g_gain_changed_at = std::chrono::steady_clock::now();
                    if (g_engine.running()) add_log("[RADIO] Gain changed: %s", gain_desc().c_str());
                    reset_pending = true;
                    reset_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
                }
                last_gain = sig;
            }
            if (reset_pending && std::chrono::steady_clock::now() >= reset_at) {
                reset_pending = false;
                if (g_engine.running()) {
                    iridi::LiveIqSource *live = g_source_kind == 1 ? &g_sdr : g_source_kind == 2 ? &g_hackrf
                                              : g_source_kind == 3 ? &g_airspy : g_source_kind == 4 ? &g_rtlsdr : nullptr;
                    if (live) live->clear_buffer(); // drop anything still queued from before
                    g_engine.reset_noise_floor();
                }
            }
        }
        draw_satellites_window();
        draw_paging_window();
        draw_voice_window();
        for (const auto &d : g_engine.voices().drain_diagnostics())
            add_log(d.rfind("[", 0) == 0 ? "%s" : "[HANDOFF] %s", d.c_str());
        log_status_tick();
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
    g_tiles.stop();
    g_tiles_night.stop();
    iridi::MsgDb::instance().shutdown();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    if (g_map_tex) glDeleteTextures(1, &g_map_tex);
    glfwDestroyWindow(g_window);
    glfwTerminate();
    return 0;
}
