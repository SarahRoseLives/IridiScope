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

// Voice auto-play: newly completed calls are queued and played in order.
bool g_autoplay = false;
int g_autoplay_last_id = -1;
std::deque<int> g_autoplay_queue;

// Active input: 0 = IQ file, 1 = SDRplay, 2 = HackRF, 3 = Airspy.
int g_source_kind = 0;
iridi::IqSource *active_source() {
    if (g_source_kind == 1) return (iridi::IqSource *)&g_sdr;
    if (g_source_kind == 2) return (iridi::IqSource *)&g_hackrf;
    if (g_source_kind == 3) return (iridi::IqSource *)&g_airspy;
    return (iridi::IqSource *)&g_file;
}

// All SDRs are locked to the Iridium band.
const double kIridiumCenterHz = 1623326935.0;

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

GLuint load_texture(const std::string &path, int &out_w, int &out_h) {
    int w = 0, h = 0, comp = 0;
    unsigned char *pixels = stbi_load(path.c_str(), &w, &h, &comp, 4);
    if (!pixels) { add_log("Failed to load texture: %s", path.c_str()); return 0; }
    GLuint tex = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    glBindTexture(GL_TEXTURE_2D, 0);
    stbi_image_free(pixels);
    out_w = w; out_h = h;
    add_log("Loaded map texture %dx%d from %s", w, h, path.c_str());
    return tex;
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

void open_iq_file(const std::string &path) {
    if (path.empty()) return;
    g_engine.stop();
    g_sdr.close();
    g_hackrf.close();
    g_airspy.close();
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

    if (g_auto_start) start_engine();
}

void start_sdr() {
    g_engine.stop();
    g_file.close();
    g_hackrf.close();
    g_airspy.close();
    g_source_kind = 1;
    g_sdr.set_gain(g_sdr_agc ? -1.0 : (double)g_sdr_gr);
    g_sdr.set_bias_tee(g_sdr_bias);
    std::string err;
    if (!g_sdr.open(g_sdr_device_idx, kIridiumCenterHz, kSdrplayRates[g_sdr_rate_idx], err)) {
        add_log("ERROR: %s", err.c_str());
        return;
    }
    add_log("SDRplay started: %s @ %.6f MHz, %.1f Msps", g_sdr.device_name().c_str(),
            kIridiumCenterHz / 1e6, kSdrplayRates[g_sdr_rate_idx] / 1e6);
    g_waterfall.clear();
    g_spec_seq = 0;
    g_freq_mhz.clear();
    start_engine();
}

void start_hackrf() {
    g_engine.stop();
    g_file.close();
    g_sdr.close();
    g_airspy.close();
    g_source_kind = 2;
    g_hackrf.set_lna_gain(g_hrf_lna);
    g_hackrf.set_vga_gain(g_hrf_vga);
    g_hackrf.set_amp(g_hrf_amp);
    g_hackrf.set_bias_tee(g_hrf_bias);
    std::string err;
    if (!g_hackrf.open(g_hrf_device_idx, kIridiumCenterHz, kHackrfRates[g_hrf_rate_idx], err)) {
        add_log("ERROR: %s", err.c_str());
        return;
    }
    add_log("HackRF started: %s @ %.6f MHz, %.1f Msps (LNA %d, VGA %d, amp %s)",
            g_hackrf.device_name().c_str(), kIridiumCenterHz / 1e6,
            kHackrfRates[g_hrf_rate_idx] / 1e6, g_hrf_lna, g_hrf_vga, g_hrf_amp ? "on" : "off");
    g_waterfall.clear();
    g_spec_seq = 0;
    g_freq_mhz.clear();
    start_engine();
}

void start_airspy() {
    g_engine.stop();
    g_file.close();
    g_sdr.close();
    g_hackrf.close();
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
    if (!g_airspy.open(g_ap_device_idx, kIridiumCenterHz, kAirspyRates[g_ap_rate_idx], err)) {
        add_log("ERROR: %s", err.c_str());
        return;
    }
    add_log("Airspy started: %s @ %.6f MHz, %.1f Msps", g_airspy.device_name().c_str(),
            kIridiumCenterHz / 1e6, kAirspyRates[g_ap_rate_idx] / 1e6);
    g_waterfall.clear();
    g_spec_seq = 0;
    g_freq_mhz.clear();
    start_engine();
}

// ---- panels ---------------------------------------------------------------

void draw_menu_bar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Open IQ file...", "Ctrl+O")) g_request_open = true;
            if (ImGui::MenuItem("Close")) { g_engine.stop(); g_file.close(); g_sdr.close(); g_waterfall.clear(); add_log("Closed source"); }
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
            ImGui::MenuItem("Voice", nullptr, &g_show_voice);
            ImGui::MenuItem("ACARS", nullptr, &g_show_acars);
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

void draw_spectrum_window() {
    if (!g_show_spectrum) return;
    ImGui::SetNextWindowSize(ImVec2(760, 280), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Spectrum", &g_show_spectrum)) {
        draw_spectrum_controls();
        std::vector<float> avg = g_engine.spectrum_avg();
        int N = (int)avg.size();
        if (N < 2) {
            ImGui::TextDisabled("Waiting for data...");
            ImGui::End();
            return;
        }
        if ((int)g_freq_mhz.size() != N) {
            g_freq_mhz.resize(N);
            double fc = active_source()->center_frequency();
            double fs = active_source()->sample_rate() ? active_source()->sample_rate() : 1.0;
            for (int i = 0; i < N; i++)
                g_freq_mhz[i] = (float)((fc + ((double)i - N / 2) * fs / N) / 1e6);
        }
        if (ImPlot::BeginPlot("##spectrum", ImVec2(-1, -1), ImPlotFlags_NoLegend)) {
            ImPlot::SetupAxes("MHz", "dBm", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_None);
            float x0 = g_freq_mhz.front(), x1 = g_freq_mhz.back();
            ImPlot::SetupAxisLimits(ImAxis_X1, x0, x1, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, g_db_min, g_db_max, ImGuiCond_Always);
            ImPlot::PlotLine("PSD", g_freq_mhz.data(), avg.data(), N);
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
        ImVec2 avail = ImGui::GetContentRegionAvail();
        g_waterfall.draw(avail);
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

        if (ImGui::BeginTable("acars", 6,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                ImVec2(0, 0))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableSetupColumn("Dir", ImGuiTableColumnFlags_WidthFixed, 30);
            ImGui::TableSetupColumn("Reg", ImGuiTableColumnFlags_WidthFixed, 84);
            ImGui::TableSetupColumn("Label", ImGuiTableColumnFlags_WidthFixed, 96);
            ImGui::TableSetupColumn("bID", ImGuiTableColumnFlags_WidthFixed, 28);
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
        if (ImGui::BeginTable("calls", 6,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                ImVec2(0, 140))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, 46);
            ImGui::TableSetupColumn("MHz", ImGuiTableColumnFlags_WidthFixed, 88);
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
                ImGui::TableSetColumnIndex(2); ImGui::Text("%.1f", cv.duration_s);
                ImGui::TableSetColumnIndex(3); ImGui::Text("%d", cv.frames);
                ImGui::TableSetColumnIndex(4); ImGui::Text("%.2f", cv.error_ratio);
                ImGui::TableSetColumnIndex(5);
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

    ImGui::BeginDisabled(running);
    ImGui::SetNextItemWidth(200);
    if (ImGui::Combo("Sample rate (Msps)", &g_sdr_rate_idx, kSdrplayRateLabels, kNumSdrplayRates)) {
        if (g_sdr.is_open()) g_sdr.set_sample_rate(kSdrplayRates[g_sdr_rate_idx]);
    }
    ImGui::EndDisabled();

    ImGui::Text("Center: %.6f MHz (locked)", kIridiumCenterHz / 1e6);

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
    bool running = g_engine.running();
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

    ImGui::BeginDisabled(running);
    ImGui::SetNextItemWidth(200);
    if (ImGui::Combo("Sample rate (Msps)", &g_hrf_rate_idx, kHackrfRateLabels, kNumHackrfRates)) {
        if (g_hackrf.is_open()) g_hackrf.set_sample_rate(kHackrfRates[g_hrf_rate_idx]);
    }
    ImGui::EndDisabled();

    ImGui::Text("Center: %.6f MHz (locked)", kIridiumCenterHz / 1e6);

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
    bool running = g_engine.running();
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

    ImGui::BeginDisabled(running);
    ImGui::SetNextItemWidth(200);
    if (ImGui::Combo("Sample rate (Msps)", &g_ap_rate_idx, kAirspyRateLabels, kNumAirspyRates)) {
        if (g_airspy.is_open()) g_airspy.set_sample_rate(kAirspyRates[g_ap_rate_idx]);
    }
    ImGui::EndDisabled();

    ImGui::Text("Center: %.6f MHz (locked)", kIridiumCenterHz / 1e6);

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

void draw_source_window() {
    if (!g_show_source) return;
    ImGui::SetNextWindowSize(ImVec2(520, 300), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Source", &g_show_source)) {
        const char *kinds[] = {"IQ file", "SDRplay", "HackRF", "Airspy"};
        ImGui::SetNextItemWidth(160);
        if (ImGui::Combo("Source", &g_source_kind, kinds, 4)) {
            g_engine.stop();
            g_file.close();
            g_sdr.close();
            g_hackrf.close();
            g_airspy.close();
            g_freq_mhz.clear();
            if (g_source_kind == 1) g_sdr_scan_requested = false; // allow a fresh scan
            if (g_source_kind == 2) g_hrf_scan_requested = false;
            if (g_source_kind == 3) g_ap_scan_requested = false;
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
        } else {
            draw_airspy_source_controls();
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
    ImGui::DockBuilderDockWindow("Playback", left);
    ImGui::DockBuilderDockWindow("Stats", left);
    ImGui::DockBuilderDockWindow("Satellites", left);
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

static int scan_live(iridi::SdrKind kind, double seconds, double rate_hz) {
    iridi::LiveIqSource sdr(kind);
    const char *label = (kind == iridi::SdrKind::HackRF) ? "HackRF" : "SDRplay";
    if (!sdr.available()) { fprintf(stderr, "%s support not compiled in\n", label); return 2; }
    std::string err;
    if (!sdr.open(0, 1623326935.0, rate_hz, err)) {
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
    if (argc >= 2) {
        std::string a1 = argv[1];
        if (a1 == "--info" || a1 == "--parse" || a1 == "--frame" || a1 == "--scan" ||
            a1 == "--sdr-list" || a1 == "--sdr-scan" || a1 == "--hrf-list" || a1 == "--hrf-scan" ||
            a1 == "--ap-list" || a1 == "--ap-scan" || a1 == "--check-update" || a1 == "--lookup")
            attach_console();
    }
    if (argc >= 2 && std::string(argv[1]) == "--check-update") return check_update();
    if (argc >= 3 && std::string(argv[1]) == "--lookup") return lookup_registration(argv[2]);
    if (argc >= 2 && std::string(argv[1]) == "--sdr-list") return sdr_list_devices();
    if (argc >= 2 && std::string(argv[1]) == "--hrf-list") return hrf_list_devices();
    if (argc >= 2 && std::string(argv[1]) == "--ap-list") return ap_list_devices();
    if (argc >= 2 && std::string(argv[1]) == "--sdr-scan")
        return scan_live(iridi::SdrKind::Sdrplay, argc >= 3 ? atof(argv[2]) : 10.0,
                         argc >= 4 ? atof(argv[3]) : 2000000.0);
    if (argc >= 2 && std::string(argv[1]) == "--hrf-scan")
        return scan_live(iridi::SdrKind::HackRF, argc >= 3 ? atof(argv[2]) : 10.0,
                         argc >= 4 ? atof(argv[3]) : 10000000.0);
    if (argc >= 2 && std::string(argv[1]) == "--ap-scan")
        return scan_live(iridi::SdrKind::Airspy, argc >= 3 ? atof(argv[2]) : 10.0,
                         argc >= 4 ? atof(argv[3]) : 10000000.0);
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

    std::string map_path = std::string(IRIDISCOPE_ASSETS) + "/worldmap.jpg";
    g_map_tex = load_texture(map_path, g_map_w, g_map_h);

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

        draw_menu_bar();
        draw_map_window();
        draw_playback_window();
        draw_stats_window();
        draw_spectrum_window();
        draw_waterfall_window();
        draw_bursts_window();
        draw_messages_window();
        draw_satellites_window();
        draw_voice_window();
        draw_acars_window();
        draw_aircraft_info_window();
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
