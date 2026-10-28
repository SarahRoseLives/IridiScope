#include "voice/voice_manager.h"
#include "store/msg_db.h"
#include "core/priority.h"
#include "voice/announce.h"
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
#include <filesystem>

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <map>

#if defined(_WIN32)
#include <direct.h>
#else
#include <sys/stat.h>
#endif

extern "C" {
#include "ambe.h" // osmo-ir77 (C linkage)
}

namespace iridi {

struct VoiceManager::Active {
    int id = 0;
    double channel = 0;
    uint64_t start = 0, last = 0;
    struct ir77_ambe_decoder *dec = nullptr;
    std::vector<int16_t> pcm;
    std::string subtype;
    int frames = 0; // subframes (2 per superframe)
    int ok = 0;
    // Handoff following: the next channel announced by handoff_resp.
    double next_channel = 0;
    uint64_t next_until = 0;
    int handoffs = 0;
    int next_sb = 0, next_fa = 0;   // announced destination (dedupe repeats)
    double next_nominal = 0;        // nominal centre of the announced channel
    uint64_t next_ts = 0;           // when the handoff response was received
    // Reorder buffer: superframes keyed by burst time. Decoder threads can
    // deliver frames out of order; they are decoded in time order once they
    // are older than the hold window.
    std::map<uint64_t, std::array<uint8_t, 39>> pend;
    uint64_t newest = 0;            // newest superframe time seen
    uint64_t decoded_ts = 0;        // time of the last superframe decoded
    int missing = 0;                // superframes filled with silence
    int dups = 0, late = 0;
    // Timeslot: each call owns one slot of the 90 ms Iridium frame, so its
    // bursts arrive at a fixed phase (to ~0.1 ms). Up to 4 calls can share a
    // frequency in different slots (8.3 ms apart); the phase keeps them apart.
    double phase = -1;              // ns within the 90 ms frame (-1 = not yet known)
    uint64_t relock_until = 0;      // after a timeslot-change handoff: accept a new phase
    bool channel_changed_pending = false; // followed to a new channel: lock its phase
    // Speech vs silence updates. While the person on this side says nothing,
    // the network only sends a frame (or two) about once a second to keep the
    // background noise going; decoded, those come out as loud ~0.1-0.2 s
    // blips. A run of <= 2 superframes with >= 4 empty slots on both sides is
    // such an update and is muted; longer runs are speech.
    size_t run_start = 0;           // pcm index where the current run began
    int run_len = 0;                // superframes in the current run
    long long gap_before = 1 << 30; // empty slots before the current run
    int speech_sf = 0, comfort_sf = 0;
};

namespace {
// Iridium duplex plan (iridium-toolkit util.py): 30 sub-bands x 8 frequency
// accesses of 10 MHz / 240 wide, starting at 1616 MHz.
constexpr double kBase = 1616.0e6;
constexpr double kChanW = 1.0e7 / 240.0;
double nominal_channel(int subband, int access) {
    return kBase + kChanW / 2 + kChanW * 8 * (subband - 1) + kChanW * (access - 1);
}
// Offset of a measured burst from the centre of the channel it sits in
// (Doppler + receiver error), carried over to predict the new channel.
double channel_offset(double f) {
    const double k = std::floor((f - kBase) / kChanW);
    return f - (kBase + k * kChanW + kChanW / 2);
}
// One AMBE superframe = 720 samples (90 ms at 8 kHz), one per 90 ms frame.
constexpr uint64_t kFrameNs = 90000000ULL;
// Hold frames this long before decoding so late (out-of-order) ones slot in.
constexpr uint64_t kHoldNs = 1000000000ULL;
// Two detections of the same burst land within a few us; real frames are 90 ms apart.
constexpr uint64_t kDupNs = 10000000ULL;
// Bursts of one call land within this of its slot phase (measured jitter is
// ~0.1 ms; neighbouring slots are 8.3 ms apart).
constexpr double kPhaseTolNs = 2.5e6;
double frame_phase(uint64_t ts) { return (double)(ts % kFrameNs); }
// Signed shortest distance b - a around the 90 ms frame.
double phase_delta(double a, double b) {
    double d = std::fmod(b - a, (double)kFrameNs);
    if (d > kFrameNs / 2.0) d -= kFrameNs;
    if (d < -(double)kFrameNs / 2.0) d += kFrameNs;
    return d;
}
// Silence-update detection (see Active): runs this short, this isolated.
constexpr int kComfortMaxRun = 2;
constexpr long long kComfortMinGap = 4;
// Fill at most this much missing time with silence (a longer gap is a new call
// segment after a handoff wait; keep it bounded).
constexpr uint64_t kMaxFillNs = 10000000000ULL;

std::string fmt_time(uint64_t ns, bool utc) {
    if (ns == 0) return "";
    const std::time_t t = (std::time_t)(ns / 1000000000ULL);
    std::tm tmv{};
#ifdef _WIN32
    if (utc ? gmtime_s(&tmv, &t) != 0 : localtime_s(&tmv, &t) != 0) return "";
#else
    if (!(utc ? gmtime_r(&t, &tmv) : localtime_r(&t, &tmv))) return "";
#endif
    char b[40];
    std::strftime(b, sizeof(b), utc ? "%Y-%m-%dT%H:%M:%SZ" : "%Y-%m-%d %H:%M:%S", &tmv);
    return b;
}
} // namespace

// Create recordings/ in the current working directory (no-op if it exists).
// Called again before every write: the manager can be constructed before
// main() picks the working directory (e.g. a Finder launch on macOS).
static void ensure_recordings_dir() {
#if defined(_WIN32)
    _mkdir("recordings");
#else
    mkdir("recordings", 0755);
#endif
}

VoiceManager::VoiceManager() {
    ensure_recordings_dir();
    writer_ = std::thread(&VoiceManager::writer_main, this);
}

VoiceManager::~VoiceManager() {
    flush_all();
    {
        std::lock_guard<std::mutex> lk(wmtx_);
        wstop_ = true;
    }
    wcv_.notify_all();
    if (writer_.joinable()) writer_.join();
}

void VoiceManager::flush_all() {
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto &a : active_) finalize(a, true);
    active_.clear();
}

// Writes queued recordings (WAV + a line in recordings/recordings.csv).
void VoiceManager::writer_main() {
    // Saving WAVs and the announcement check never compete with decoding.
    int prio_gen = 0;
    for (;;) {
        iridi_refresh_thread_priority(IRIDI_PRIO_BACKGROUND, &prio_gen);
        SaveJob job;
        {
            std::unique_lock<std::mutex> lk(wmtx_);
            wcv_.wait(lk, [this] { return wstop_ || !wqueue_.empty(); });
            if (wqueue_.empty()) return; // stopping and nothing left
            job = std::move(wqueue_.front());
            wqueue_.pop_front();
        }
        // Network announcement? File it under recordings/announcements/<name>/.
        std::string ann;
        if (sort_ann_.load()) ann = AnnouncementMatcher::instance().classify(job.pcm);
        if (!ann.empty()) {
            const std::string dir = "recordings/announcements/" + ann;
            std::error_code ec;
            std::filesystem::create_directories(dir, ec);
            const std::string fname = job.path.substr(job.path.find_last_of('/') + 1);
            job.path = dir + "/" + fname;
            job.csv_line = "announcements/" + ann + "/" + job.csv_line;
            ann_count_.fetch_add(1);
        }
        job.call.wav_path = job.path;
        job.call.announcement = ann;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            for (auto &d : done_)
                if (d.id == job.call.id) { d.wav_path = job.path; d.announcement = ann; d.classified = true; }
        }
        MsgDb::instance().voice(job.call);
        auto vlog = [this](const std::string &l) {
            std::lock_guard<std::mutex> lk(mtx_);
            diag_.push_back(l);
            if (diag_.size() > 200) diag_.pop_front();
        };
        ensure_recordings_dir();
        FILE *f = std::fopen(job.path.c_str(), "wb");
        if (!f) { vlog("[ALERT] Could not create " + job.path + " (disk full or folder not writable?)"); continue; }
        const uint32_t data_bytes = (uint32_t)(job.pcm.size() * 2);
        const uint32_t riff = 36 + data_bytes, rate = 8000, byte_rate = 16000, fsz = 16;
        const uint16_t block_align = 2, bits = 16, fmt = 1, ch = 1;
        bool ok = true;
        ok &= std::fwrite("RIFF", 1, 4, f) == 4; ok &= std::fwrite(&riff, 4, 1, f) == 1;
        ok &= std::fwrite("WAVE", 1, 4, f) == 4; ok &= std::fwrite("fmt ", 1, 4, f) == 4;
        ok &= std::fwrite(&fsz, 4, 1, f) == 1; ok &= std::fwrite(&fmt, 2, 1, f) == 1;
        ok &= std::fwrite(&ch, 2, 1, f) == 1; ok &= std::fwrite(&rate, 4, 1, f) == 1;
        ok &= std::fwrite(&byte_rate, 4, 1, f) == 1; ok &= std::fwrite(&block_align, 2, 1, f) == 1;
        ok &= std::fwrite(&bits, 2, 1, f) == 1; ok &= std::fwrite("data", 1, 4, f) == 4;
        ok &= std::fwrite(&data_bytes, 4, 1, f) == 1;
        ok &= std::fwrite(job.pcm.data(), 1, data_bytes, f) == data_bytes;
        ok &= std::fclose(f) == 0;
        if (!ok) { vlog("[ALERT] Writing " + job.path + " failed (disk full?)"); continue; }
        {
            char l[512];
            std::snprintf(l, sizeof(l), "[VOICE] Saved call #%d: %.1f s (%.1f s speech), %.4f MHz, %d handoff(s)%s%s -> %s",
                          job.call.id, job.call.duration_s, job.call.speech_s, job.call.channel_hz / 1e6,
                          job.call.handoffs, ann.empty() ? "" : ", announcement: ", ann.c_str(), job.path.c_str());
            vlog(l);
        }
        const char *csv = "recordings/recordings.csv";
        FILE *probe = std::fopen(csv, "rb");
        const bool fresh = !probe;
        if (probe) std::fclose(probe);
        if (FILE *c = std::fopen(csv, "ab")) {
            if (fresh)
                std::fputs("file,start_local,start_utc,end_utc,duration_s,freq_mhz,error_pct,"
                           "frames,missing_frames,handoffs,type\n", c);
            std::fputs(job.csv_line.c_str(), c);
            std::fclose(c);
        }
    }
}

void VoiceManager::reset() {
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto &a : active_)
        if (a->dec) ir77_ambe_decode_release(a->dec);
    active_.clear();
    done_.clear();
    done_pcm_.clear();
    next_id_ = 0;
    speech_calls_.store(0); speech_ms_.store(0); longest_ms_.store(0);
}

void VoiceManager::close_run(Active &a, long long gap_after) {
    if (a.run_len <= 0) return;
    // The run spans from its first superframe to the end of the audio so far
    // (short gaps inside it were already filled with silence).
    const size_t b = std::min(a.run_start, a.pcm.size()), e = a.pcm.size();
    if (a.run_len <= kComfortMaxRun && a.gap_before >= kComfortMinGap && gap_after >= kComfortMinGap) {
        std::fill(a.pcm.begin() + (long)b, a.pcm.begin() + (long)e, (int16_t)0);
        a.comfort_sf += a.run_len;
    } else {
        // Speech = superframes that actually carry sound (VOZ/blank ones don't).
        for (size_t i = b; i + 720 <= e; i += 720) {
            int pk = 0;
            for (size_t k = i; k < i + 720; k++) { const int m = a.pcm[k] < 0 ? -(int)a.pcm[k] : a.pcm[k]; if (m > pk) pk = m; }
            if (pk > 64) a.speech_sf++;
        }
    }
    a.run_len = 0;
}

void VoiceManager::finalize(std::unique_ptr<Active> &a, bool have_end) {
    decode_ready(*a, true); // anything still held in the reorder buffer
    close_run(*a, 1 << 30); // the call ended: its last run is complete
    VoiceCall c;
    c.id = a->id;
    c.channel_hz = a->channel;
    c.start_ns = a->start;
    c.end_ns = a->last;
    c.frames = a->frames;
    c.error_ratio = a->frames > 0 ? (1.0 - (double)a->ok / (double)a->frames) : 1.0;
    c.subtype = a->subtype;
    c.duration_s = (double)a->pcm.size() / 8000.0;
    c.handoffs = a->handoffs;
    c.missing = a->missing;
    c.speech_s = a->speech_sf * 0.09;
    if (c.speech_s >= 1.0) {
        speech_calls_.fetch_add(1);
        speech_ms_.fetch_add((uint64_t)(c.speech_s * 1000.0));
        uint64_t prev = longest_ms_.load();
        const uint64_t me = (uint64_t)(c.speech_s * 1000.0);
        while (me > prev && !longest_ms_.compare_exchange_weak(prev, me)) {}
    }
    if (!have_end && c.error_ratio > 0.6) {
        // Rejected/aborted: keep the record but flag low quality via error ratio.
    }
    // Skip saving calls that decoded to (near) silence: the AMBE decoder
    // outputs zeros for frames it cannot correct, so failed calls and
    // non-speech traffic labelled VOC produce silent files.
    int peak = 0;
    for (int16_t v : a->pcm) { int m = v < 0 ? -(int)v : (int)v; if (m > peak) peak = m; }
    // Also skip calls with under 1 s of speech: only the network's silence
    // updates (the person on this side never spoke) - blips, not a call.
    const bool audible = peak > 64 && c.error_ratio < 0.9 && c.speech_s >= 1.0;
    if (!a->pcm.empty() && audible) {
        // Name by the call's local start date/time plus its number, so files
        // never collide across restarts (the number restarts at 0 each run).
        char stamp[32] = "nodate-000000";
        {
            const std::time_t t = (std::time_t)(a->start / 1000000000ULL);
            std::tm lt{};
#ifdef _WIN32
            const bool ok = t > 0 && localtime_s(&lt, &t) == 0;
#else
            const bool ok = t > 0 && localtime_r(&t, &lt) != nullptr;
#endif
            if (ok) std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &lt);
        }
        char path[160];
        snprintf(path, sizeof(path), "recordings/voice-%s-%04d.wav", stamp, a->id);
        // Never overwrite an existing recording.
        for (int n = 2; n < 100; n++) {
            FILE *probe = std::fopen(path, "rb");
            if (!probe) break;
            std::fclose(probe);
            snprintf(path, sizeof(path), "recordings/voice-%s-%04d-%d.wav", stamp, a->id, n);
        }
        c.wav_path = path;
        SaveJob job;
        job.path = path;
        job.pcm = a->pcm;
        char line[512];
        const char *fname = path + 11; // after "recordings/"
        snprintf(line, sizeof(line), "%s,%s,%s,%s,%.2f,%.6f,%.1f,%d,%d,%d,%s\n",
                 fname, fmt_time(a->start, false).c_str(), fmt_time(a->start, true).c_str(),
                 fmt_time(a->last, true).c_str(), c.duration_s, a->channel / 1e6,
                 100.0 * c.error_ratio, a->frames / 2, a->missing, a->handoffs,
                 a->subtype.empty() ? "VOC" : a->subtype.c_str());
        job.csv_line = line;
        job.call = c;
        {
            std::lock_guard<std::mutex> wl(wmtx_);
            wqueue_.push_back(std::move(job));
        }
        wcv_.notify_one();
    }
    if (c.wav_path.empty()) MsgDb::instance().voice(c);   // saved ones: logged by the writer
    done_.push_back(c);
    done_pcm_.emplace_back(a->id, std::move(a->pcm));
    if (done_.size() > 500) done_.pop_front();
    if (done_pcm_.size() > 500) done_pcm_.pop_front();
    if (a->dec) ir77_ambe_decode_release(a->dec);
}

void VoiceManager::feed(const irid::ParsedMessage &pm) {
    std::lock_guard<std::mutex> lk(mtx_);
    const bool follow = follow_handoffs_.load();
    // Worker threads can deliver frames slightly out of order: compare times
    // symmetrically rather than with an unsigned difference.
    const double ph = frame_phase(pm.timestamp_ns);
    // Same frequency AND same timeslot (phase) as the call.
    auto on_channel = [&](const Active &c) {
        const uint64_t dt = pm.timestamp_ns > c.last ? pm.timestamp_ns - c.last : c.last - pm.timestamp_ns;
        if (std::fabs(c.channel - pm.frequency) >= 20000.0 || dt >= 20000000000ULL) return false;
        return c.phase < 0 || std::fabs(phase_delta(c.phase, ph)) <= kPhaseTolNs;
    };
    // Same frequency, different slot, right after a timeslot-change handoff.
    auto relocking = [&](const Active &c) {
        return pm.timestamp_ns <= c.relock_until && std::fabs(c.channel - pm.frequency) < 20000.0;
    };

    // Handoff response on a call's channel: remember where it is moving to.
    if (follow && pm.has_handoff && !pm.ho_denied &&
        pm.ho_sband_dn >= 1 && pm.ho_sband_dn <= 30 && pm.ho_access >= 1 && pm.ho_access <= 8) {
        for (auto &c : active_) {
            if (!on_channel(*c)) continue;
            const double nominal = nominal_channel(pm.ho_sband_dn, pm.ho_access);
            const double predicted = nominal + channel_offset(pm.frequency);
            // The same response is usually repeated; count/record it once.
            const bool repeat = c->next_sb == pm.ho_sband_dn && c->next_fa == pm.ho_access &&
                                pm.timestamp_ns <= c->next_until;
            if (repeat) break; // keep the first prediction and its deadline
            c->next_sb = pm.ho_sband_dn;
            c->next_fa = pm.ho_access;
            c->next_nominal = nominal;
            c->next_ts = pm.timestamp_ns;
            if (std::fabs(predicted - pm.frequency) < 20000.0) {
                // Same frequency, new timeslot: the call stays on this channel.
                c->next_channel = 0;
                c->relock_until = pm.timestamp_ns + 3000000000ULL;
                c->next_until = pm.timestamp_ns + 10000000000ULL; // dedupe window
                c->handoffs++;
                ho_seen_.fetch_add(1);
                ho_followed_.fetch_add(1);
                char line[320];
                snprintf(line, sizeof(line),
                         "Handoff followed: call #%d stays on %.4f MHz (timeslot change only, slot %d)",
                         c->id, pm.frequency / 1e6, pm.ho_slot);
                diag_.push_back(line);
                if (diag_.size() > 200) diag_.pop_front();
                break;
            }
            c->next_channel = predicted;
            c->next_until = pm.timestamp_ns + 10000000000ULL; // switch happens within seconds
            if (!repeat) {
                ho_seen_.fetch_add(1);
                char line[320];
                snprintf(line, sizeof(line),
                         "Handoff on voice call #%d: %.4f MHz -> sb%d/fa%d slot %d, watching %.4f MHz",
                         c->id, pm.frequency / 1e6, pm.ho_sband_dn, pm.ho_access, pm.ho_slot, predicted / 1e6);
                diag_.push_back(line);
                if (diag_.size() > 200) diag_.pop_front();
                PendingHo h;
                h.from = pm.frequency; h.nominal = nominal; h.predicted = predicted;
                h.sb = pm.ho_sband_dn; h.fa = pm.ho_access; h.ts = pm.timestamp_ns;
                h.call_id = c->id;
                pending_ho_.push_back(h);
                if (pending_ho_.size() > 64) pending_ho_.pop_front();
            }
            break;
        }
    }

    if (pm.voice_payload.size() != 39) return;
    // VOD frames are not raw AMBE superframes (iridium-toolkit reassembles
    // them across frames/channels: vod-stitcher.py); only VOC/VOZ are speech.
    if (pm.voice_subtype == "VOD") return;

    Active *a = nullptr;
    // 1) Same channel and timeslot (a frame on an existing call's slot always wins).
    for (auto &c : active_) {
        if (on_channel(*c)) { a = c.get(); break; }
    }
    // 1b) A call that was just moved to another timeslot on this frequency.
    bool relock = false;
    if (!a && follow) {
        for (auto &c : active_) {
            if (relocking(*c)) { a = c.get(); relock = true; break; }
        }
    }
    // 2) The channel a call was told to hand off to.
    if (!a && follow) {
        for (auto &c : active_) {
            if (c->next_channel > 0 && pm.timestamp_ns <= c->next_until &&
                std::fabs(c->next_channel - pm.frequency) < 25000.0) {
                a = c.get();
                a->handoffs++;
                a->next_channel = 0;
                a->channel_changed_pending = true;
                ho_followed_.fetch_add(1);
                for (auto &h : pending_ho_)
                    if (!h.resolved && std::fabs(h.predicted - pm.frequency) < 25000.0) {
                        h.resolved = true;
                        char line[320];
                        snprintf(line, sizeof(line),
                                 "Handoff followed: call #%d now on %.4f MHz (%.1f s after, %+.1f kHz from predicted)",
                                 a->id, pm.frequency / 1e6, (pm.timestamp_ns - h.ts) / 1e9,
                                 (pm.frequency - h.predicted) / 1e3);
                        diag_.push_back(line);
                        if (diag_.size() > 200) diag_.pop_front();
                    }
                break;
            }
        }
    }
    // 3) Handoff to a different satellite: Doppler (tens of kHz at L-band)
    //    changes, so the carried-over offset is wrong. Accept a voice frame
    //    within +/-40 kHz of the announced channel's nominal centre, but only
    //    within 3 s of the handoff response so a neighbouring call isn't taken.
    bool doppler_match = false;
    if (!a && follow) {
        for (auto &c : active_) {
            if (c->next_channel > 0 && c->next_nominal > 0 && pm.timestamp_ns >= c->next_ts &&
                pm.timestamp_ns - c->next_ts <= 3000000000ULL &&
                std::fabs(pm.frequency - c->next_nominal) < 40000.0) {
                a = c.get();
                a->handoffs++;
                a->channel_changed_pending = true;
                ho_followed_.fetch_add(1);
                doppler_match = true;
                for (auto &h : pending_ho_)
                    if (!h.resolved && h.call_id == a->id) {
                        h.resolved = true;
                        char line[320];
                        snprintf(line, sizeof(line),
                                 "Handoff followed (different satellite): call #%d now on %.4f MHz "
                                 "(%.1f s after; offset %+.1f kHz vs %+.1f kHz before)",
                                 a->id, pm.frequency / 1e6, (pm.timestamp_ns - h.ts) / 1e9,
                                 (pm.frequency - h.nominal) / 1e3, (h.predicted - h.nominal) / 1e3);
                        diag_.push_back(line);
                        if (diag_.size() > 200) diag_.pop_front();
                    }
                a->next_channel = 0;
                break;
            }
        }
    }
    (void)doppler_match;
    if (a) {
        a->channel = pm.frequency; // track the current channel (and drift)
        // Track the slot phase (new channel or slot after a handoff: lock anew).
        const bool moved = relock || a->channel_changed_pending;
        if (a->phase < 0 || moved) a->phase = ph;
        else a->phase = std::fmod(a->phase + 0.2 * phase_delta(a->phase, ph) + kFrameNs, (double)kFrameNs);
        if (relock) a->relock_until = 0;
        a->channel_changed_pending = false;
    }
    if (!a) {
        // Diagnostic: a new call starting soon after an unfollowed handoff may
        // be that handoff's destination. Report where it landed relative to
        // the announced channel so the prediction can be checked.
        for (auto &h : pending_ho_) {
            if (h.resolved || pm.timestamp_ns < h.ts || pm.timestamp_ns - h.ts > 10000000000ULL) continue;
            // Only a call that starts within a few channels of the prediction
            // is plausibly this handoff's destination.
            if (std::fabs(pm.frequency - h.predicted) > 250000.0) continue;
            h.resolved = true;
            char line[320];
            snprintf(line, sizeof(line),
                     "Handoff missed: new call started near the predicted channel: %.4f -> sb%d/fa%d (nominal %.4f, predicted %.4f); new call at %.4f MHz "
                     "after %.1f s: %+.1f kHz from nominal, %+.1f kHz from predicted",
                     h.from / 1e6, h.sb, h.fa, h.nominal / 1e6, h.predicted / 1e6, pm.frequency / 1e6,
                     (pm.timestamp_ns - h.ts) / 1e9, (pm.frequency - h.nominal) / 1e3,
                     (pm.frequency - h.predicted) / 1e3);
            diag_.push_back(line);
            if (diag_.size() > 100) diag_.pop_front();
            break;
        }
        auto up = std::make_unique<Active>();
        up->id = next_id_++;
        up->channel = pm.frequency;
        up->start = pm.timestamp_ns;
        up->last = pm.timestamp_ns;
        up->dec = ir77_ambe_decode_alloc();
        if (!up->dec) return;
        a = up.get();
        active_.push_back(std::move(up));
    }

    // The AMBE superframe is the payload with each byte's bits reversed
    // (matches iridium-toolkit's bits_to_dfs.py before ir77_ambe_decode).
    std::array<uint8_t, 39> sf{};
    for (int i = 0; i < 39; i++) {
        uint8_t b = pm.voice_payload[i], r = 0;
        for (int k = 0; k < 8; k++) r = (uint8_t)((r << 1) | ((b >> k) & 1));
        sf[i] = r;
    }
    if (a->subtype.empty() && !pm.voice_subtype.empty())
        a->subtype = pm.voice_subtype;
    if (pm.timestamp_ns > a->last) a->last = pm.timestamp_ns;

    const uint64_t ts = pm.timestamp_ns;
    // Too late: that point of the call has already been decoded.
    if (a->decoded_ts && ts <= a->decoded_ts + kDupNs) {
        if (ts + kDupNs >= a->decoded_ts) a->dups++; else a->late++;
        return;
    }
    // Same burst detected twice: keep one copy.
    auto nearby = a->pend.lower_bound(ts > kDupNs ? ts - kDupNs : 0);
    if (nearby != a->pend.end() && nearby->first <= ts + kDupNs) { a->dups++; return; }
    a->pend.emplace(ts, sf);
    if (ts > a->newest) a->newest = ts;
    decode_ready(*a, false);
}

// Decode buffered superframes in time order: all of them, or those older
// than the hold window. Missing 90 ms frames become silence so the recording
// keeps real timing.
void VoiceManager::decode_ready(Active &a, bool all) {
    while (!a.pend.empty()) {
        auto it = a.pend.begin();
        if (!all && it->first + kHoldNs > a.newest) break;
        const uint64_t ts = it->first;
        long long empty = 1 << 30;   // empty slots since the previous superframe
        if (a.decoded_ts) {
            const uint64_t gap = ts - a.decoded_ts;
            const long long miss = (long long)((gap + kFrameNs / 2) / kFrameNs) - 1;
            empty = miss > 0 ? miss : 0;
            if (empty >= kComfortMinGap && a.run_len > 0) close_run(a, empty);
            if (miss > 0 && gap <= kMaxFillNs) {
                a.pcm.insert(a.pcm.end(), (size_t)miss * 720, (int16_t)0);
                a.missing += (int)miss;
            }
        }
        if (a.run_len == 0) { a.run_start = a.pcm.size(); a.gap_before = empty; }
        int16_t audio[720];
        const int rv = ir77_ambe_decode_superframe(a.dec, audio, 720, it->second.data());
        if (rv >= 0) {
            a.pcm.insert(a.pcm.end(), audio, audio + 720);
            a.frames += 2;
            a.ok += rv;
            a.run_len++;
        } else if (a.run_len == 0) {
            a.run_start = a.pcm.size();
        }
        a.decoded_ts = ts;
        a.pend.erase(it);
    }
}

void VoiceManager::sweep(uint64_t now_ns) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto &h : pending_ho_) {
        if (h.resolved || now_ns < h.ts || now_ns - h.ts <= 10000000000ULL) continue;
        h.resolved = true;
        // Did the call keep going on its original channel after the handoff?
        const char *fate = "call ended (no voice on the old channel either)";
        for (const auto &c : active_)
            if (c->id == h.call_id) {
                if (c->last > h.ts + 2000000000ULL && std::fabs(c->channel - h.from) < 20000.0)
                    fate = "call continued on its original channel";
                else
                    fate = "call went silent after the handoff";
                break;
            }
        for (const auto &d : done_)
            if (d.id == h.call_id) {
                fate = d.end_ns > h.ts + 2000000000ULL ? "call continued on its original channel, then ended"
                                                      : "call went silent after the handoff";
                break;
            }
        char line[320];
        snprintf(line, sizeof(line),
                 "Handoff not followed (call #%d): no voice on %.4f MHz (sb%d/fa%d) within 10 s; %s",
                 h.call_id, h.predicted / 1e6, h.sb, h.fa, fate);
        diag_.push_back(line);
        if (diag_.size() > 200) diag_.pop_front();
    }
    for (size_t i = 0; i < active_.size();) {
        const bool awaiting_handoff = active_[i]->next_channel > 0 && now_ns <= active_[i]->next_until;
        if (!awaiting_handoff && now_ns > active_[i]->last && (now_ns - active_[i]->last) > 3000000000ULL) {
            finalize(active_[i], true);
            active_.erase(active_.begin() + i);
        } else ++i;
    }
}

std::vector<VoiceCall> VoiceManager::calls() const {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<VoiceCall> out(done_.begin(), done_.end());
    for (const auto &a : active_) {
        VoiceCall c;
        c.id = a->id; c.channel_hz = a->channel; c.start_ns = a->start; c.end_ns = a->last;
        c.frames = a->frames;
        c.error_ratio = a->frames > 0 ? (1.0 - (double)a->ok / (double)a->frames) : 1.0;
        c.active = true;
        c.duration_s = (double)a->pcm.size() / 8000.0;
        c.handoffs = a->handoffs;
        c.missing = a->missing;
        c.speech_s = a->speech_sf * 0.09;
        c.subtype = a->subtype;
        out.push_back(c);
    }
    return out;
}

std::vector<std::string> VoiceManager::drain_diagnostics() {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<std::string> out(diag_.begin(), diag_.end());
    diag_.clear();
    return out;
}

std::vector<int16_t> VoiceManager::pcm(int id) const {
    std::lock_guard<std::mutex> lk(mtx_);
    for (const auto &kv : done_pcm_)
        if (kv.first == id) return kv.second;
    for (const auto &a : active_)
        if (a->id == id) return a->pcm;
    return {};
}

} // namespace iridi
