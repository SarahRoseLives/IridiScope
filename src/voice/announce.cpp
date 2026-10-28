#include "voice/announce.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstring>
#include <filesystem>

namespace iridi {

namespace {
constexpr int kHop = 160, kN = 256, kBands = 20, kSf = 720;
constexpr int kMinOverlap = 100;        // 2 s of speech in common
constexpr size_t kMaxCall = 8000 * 90;  // look at the first 90 s of a call

void fft256(std::complex<float> *a) {
    for (int i = 1, j = 0; i < kN; i++) {
        int bit = kN >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (int len = 2; len <= kN; len <<= 1) {
        const float ang = -2.0f * 3.14159265f / len;
        const std::complex<float> wl(std::cos(ang), std::sin(ang));
        for (int i = 0; i < kN; i += len) {
            std::complex<float> w(1, 0);
            for (int k = 0; k < len / 2; k++) {
                const std::complex<float> u = a[i + k], v = a[i + k + len / 2] * w;
                a[i + k] = u + v; a[i + k + len / 2] = u - v;
                w *= wl;
            }
        }
    }
}

bool read_wav(const std::string &path, std::vector<int16_t> &out) {
    FILE *f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    char hdr[12];
    if (std::fread(hdr, 1, 12, f) != 12 || std::memcmp(hdr, "RIFF", 4) || std::memcmp(hdr + 8, "WAVE", 4)) {
        std::fclose(f); return false;
    }
    uint16_t ch = 1, bits = 16; uint32_t rate = 8000;
    for (;;) {
        char id[4]; uint32_t sz;
        if (std::fread(id, 1, 4, f) != 4 || std::fread(&sz, 4, 1, f) != 1) break;
        if (!std::memcmp(id, "fmt ", 4)) {
            std::vector<char> b(sz);
            if (std::fread(b.data(), 1, sz, f) != sz) break;
            std::memcpy(&ch, b.data() + 2, 2); std::memcpy(&rate, b.data() + 4, 4); std::memcpy(&bits, b.data() + 14, 2);
        } else if (!std::memcmp(id, "data", 4)) {
            if (ch != 1 || bits != 16 || rate != 8000) break;
            out.resize(sz / 2);
            const size_t got = std::fread(out.data(), 2, out.size(), f);
            out.resize(got);
            std::fclose(f);
            return !out.empty();
        } else {
            std::fseek(f, (long)sz + (sz & 1), SEEK_CUR);
        }
    }
    std::fclose(f);
    return false;
}
} // namespace

AnnouncementMatcher &AnnouncementMatcher::instance() {
    static AnnouncementMatcher m;
    return m;
}

int AnnouncementMatcher::templates() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return (int)t_.size();
}

// Per 20 ms frame: log energy in 20 bands (100-3800 Hz). Silent frames and the
// network's isolated once-a-second silence updates are marked "off".
AnnouncementMatcher::Feat AnnouncementMatcher::features(const std::vector<int16_t> &pcm, size_t max_samples) const {
    Feat F;
    const size_t len = std::min(pcm.size(), max_samples);
    F.n = (int)(len / kHop);
    F.v.assign((size_t)F.n * kBands, 0.0f);
    F.on.assign(F.n, 0);
    // Which 90 ms superframes carry sound, minus isolated 1-2 superframe blips.
    const int nsf = (int)(len / kSf) + 1;
    std::vector<char> sf(nsf, 0);
    for (int s = 0; s < nsf; s++)
        for (size_t i = (size_t)s * kSf; i < std::min(len, (size_t)(s + 1) * kSf); i++)
            if (pcm[i] > 1 || pcm[i] < -1) { sf[s] = 1; break; }
    for (int s = 0; s < nsf;) {
        if (!sf[s]) { s++; continue; }
        int e = s; while (e < nsf && sf[e]) e++;
        int gb = 0; for (int k = s - 1; k >= 0 && !sf[k]; k--) gb++;
        int ga = 0; for (int k = e; k < nsf && !sf[k]; k++) ga++;
        if (e - s <= 2 && (s == 0 || gb >= 4) && (e == nsf || ga >= 4)) for (int k = s; k < e; k++) sf[k] = 0;
        s = e;
    }
    static float edges[kBands + 1];
    static bool init = false;
    if (!init) { for (int b = 0; b <= kBands; b++) edges[b] = 100.0f * std::pow(38.0f, b / (float)kBands); init = true; }
    std::complex<float> buf[kN];
    for (int i = 0; i < F.n; i++) {
        const size_t s0 = (size_t)i * kHop;
        if (!sf[s0 / kSf]) continue;
        bool any = false;
        for (int k = 0; k < kHop; k++) if (pcm[s0 + k] > 1 || pcm[s0 + k] < -1) { any = true; break; }
        if (!any) continue;
        for (int k = 0; k < kN; k++) {
            const float w = 0.5f - 0.5f * std::cos(2 * 3.14159265f * k / (kN - 1));
            buf[k] = s0 + k < len ? std::complex<float>(pcm[s0 + k] * w, 0) : 0;
        }
        fft256(buf);
        float *v = &F.v[(size_t)i * kBands];
        double band[kBands] = {};
        for (int k = 1; k < kN / 2; k++) {
            const float hz = k * 8000.0f / kN;
            for (int b = 0; b < kBands; b++)
                if (hz >= edges[b] && hz < edges[b + 1]) { band[b] += std::norm(buf[k]); break; }
        }
        for (int b = 0; b < kBands; b++) v[b] = (float)std::log10(band[b] + 1e-3);
        F.on[i] = 1;
    }
    return F;
}

static void normalise(std::vector<float> &v, const std::vector<char> &on, const std::vector<float> &mean) {
    const int n = (int)on.size();
    for (int i = 0; i < n; i++) {
        float *x = &v[(size_t)i * kBands];
        if (!on[i]) { std::fill(x, x + kBands, 0.0f); continue; }
        double m = 0;
        for (int b = 0; b < kBands; b++) { x[b] -= mean[b]; m += x[b]; }
        m /= kBands;
        double nn = 0;
        for (int b = 0; b < kBands; b++) { x[b] -= (float)m; nn += x[b] * x[b]; }
        nn = std::sqrt(nn);
        for (int b = 0; b < kBands; b++) x[b] = nn > 0 ? (float)(x[b] / nn) : 0.0f;
    }
}

int AnnouncementMatcher::load(const std::string &dir) {
    std::vector<Tmpl> t;
    std::error_code ec;
    for (auto &e : std::filesystem::directory_iterator(dir, ec)) {
        if (!e.is_regular_file()) continue;
        auto p = e.path();
        std::string ext = p.extension().string();
        for (auto &c : ext) c = (char)std::tolower((unsigned char)c);
        if (ext != ".wav") continue;
        std::vector<int16_t> pcm;
        if (!read_wav(p.string(), pcm)) continue;
        Tmpl tm;
        tm.name = p.stem().string();
        tm.f = features(pcm, pcm.size());
        t.push_back(std::move(tm));
    }
    // Common reference spectrum: the templates' average band levels.
    std::vector<float> mean(kBands, 0.0f);
    int cnt = 0;
    for (auto &tm : t) {
        std::vector<double> s(kBands, 0); int n = 0;
        for (int i = 0; i < tm.f.n; i++) if (tm.f.on[i]) { for (int b = 0; b < kBands; b++) s[b] += tm.f.v[(size_t)i * kBands + b]; n++; }
        if (n) { for (int b = 0; b < kBands; b++) mean[b] += (float)(s[b] / n); cnt++; }
    }
    if (cnt) for (auto &m : mean) m /= cnt;
    for (auto &tm : t) normalise(tm.f.v, tm.f.on, mean);
    std::lock_guard<std::mutex> lk(mtx_);
    t_ = std::move(t);
    mean_ = mean;
    return (int)t_.size();
}

double AnnouncementMatcher::best_match(const Feat &c, const Feat &t) {
    double best = -1.0;
    for (int lag = -c.n + kMinOverlap; lag <= t.n - kMinOverlap; lag++) {
        const int a0 = std::max(0, -lag), b0 = std::max(0, lag);
        const int L = std::min(c.n - a0, t.n - b0);
        if (L < kMinOverlap) continue;
        double sum = 0; int k = 0;
        for (int i = 0; i < L; i++) {
            if (!c.on[a0 + i] || !t.on[b0 + i]) continue;
            const float *x = &c.v[(size_t)(a0 + i) * kBands], *y = &t.v[(size_t)(b0 + i) * kBands];
            float d = 0;
            for (int b = 0; b < kBands; b++) d += x[b] * y[b];
            sum += d; k++;
        }
        if (k >= kMinOverlap && sum / k > best) best = sum / k;
    }
    return best;
}

std::string AnnouncementMatcher::classify(const std::vector<int16_t> &pcm, double *score) const {
    std::lock_guard<std::mutex> lk(mtx_);
    if (score) *score = -1;
    if (t_.empty()) return "";
    Feat c = features(pcm, kMaxCall);
    int on = 0; for (char o : c.on) on += o;
    if (on < kMinOverlap) return "";
    normalise(c.v, c.on, mean_);
    std::string name; double best = -1;
    for (const auto &tm : t_) {
        const double s = best_match(c, tm.f);
        if (s > best) { best = s; name = tm.name; }
    }
    if (score) *score = best;
    return best >= kThreshold ? name : "";
}

} // namespace iridi
