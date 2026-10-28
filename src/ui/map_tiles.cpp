#include "ui/map_tiles.h"
#include "util/http_get.h"
#include "core/priority.h"

#include <chrono>
#include <cstdio>
#include <string>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <winhttp.h>
#include <direct.h>
#define MT_MKDIR(p) _mkdir(p)
#else
#include <sys/stat.h>
#define MT_MKDIR(p) mkdir(p, 0755)
#endif

#include <GL/gl.h>
#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif

#include "stb/stb_image.h"

namespace iridi {

namespace {
void make_dirs(const std::string &file) {
    for (size_t i = 0; i < file.size(); i++)
        if (file[i] == '/') MT_MKDIR(file.substr(0, i).c_str());
}

bool read_file(const std::string &path, std::vector<unsigned char> &out) {
    FILE *f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    std::fseek(f, 0, SEEK_END);
    long n = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (n <= 0) { std::fclose(f); return false; }
    out.resize((size_t)n);
    const bool ok = std::fread(out.data(), 1, (size_t)n, f) == (size_t)n;
    std::fclose(f);
    return ok;
}

#if defined(_WIN32)
bool http_get(const std::wstring &host, const std::string &path, std::vector<unsigned char> &out) {
    out.clear();
    HINTERNET s = WinHttpOpen(L"IridiScope (map tiles)", WINHTTP_ACCESS_TYPE_DEFAULT_PROXY,
                              WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
    if (!s) return false;
    WinHttpSetTimeouts(s, 6000, 6000, 8000, 8000);
    HINTERNET c = WinHttpConnect(s, host.c_str(), INTERNET_DEFAULT_HTTPS_PORT, 0);
    if (!c) { WinHttpCloseHandle(s); return false; }
    const std::wstring wpath(path.begin(), path.end());
    HINTERNET r = WinHttpOpenRequest(c, L"GET", wpath.c_str(), nullptr, WINHTTP_NO_REFERER,
                                     WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE);
    bool ok = r && WinHttpSendRequest(r, WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA, 0, 0, 0) &&
              WinHttpReceiveResponse(r, nullptr);
    if (ok) {
        DWORD status = 0, sz = sizeof(status);
        WinHttpQueryHeaders(r, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
                            WINHTTP_HEADER_NAME_BY_INDEX, &status, &sz, WINHTTP_NO_HEADER_INDEX);
        ok = status == 200;
    }
    while (ok) {
        DWORD avail = 0;
        if (!WinHttpQueryDataAvailable(r, &avail)) { ok = false; break; }
        if (avail == 0) break;
        const size_t at = out.size();
        out.resize(at + avail);
        DWORD got = 0;
        if (!WinHttpReadData(r, out.data() + at, avail, &got)) { ok = false; break; }
        out.resize(at + got);
        if (out.size() > 4u << 20) { ok = false; break; } // a tile is never this big
    }
    if (r) WinHttpCloseHandle(r);
    WinHttpCloseHandle(c);
    WinHttpCloseHandle(s);
    return ok && !out.empty();
}
#else
// macOS / Linux: libcurl through util/http_get (no-op without libcurl).
bool http_get(const std::wstring &host, const std::string &path, std::vector<unsigned char> &out) {
    out.clear();
    std::string body, err;
    int status = 0;
    const std::string url = "https://" + std::string(host.begin(), host.end()) + path;
    if (!http_get_portable(url, body, status, &err) || status != 200 || body.empty()) return false;
    if (body.size() > 4u << 20) return false; // a tile is never this big
    out.assign(body.begin(), body.end());
    return true;
}
#endif
} // namespace

void MapTiles::start(int threads) {
    if (!threads_.empty()) return;
    stop_ = false;
    for (int i = 0; i < threads; i++) threads_.emplace_back(&MapTiles::worker, this);
}

void MapTiles::stop() {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        stop_ = true;
    }
    cv_.notify_all();
    for (auto &t : threads_) if (t.joinable()) t.join();
    threads_.clear();
}

void MapTiles::begin_frame() {
    std::lock_guard<std::mutex> lk(mtx_);
    frame_++;
}

int MapTiles::pending() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return (int)queued_.size();
}

unsigned int MapTiles::get(int z, int x, int y) {
    const uint64_t k = key(z, x, y);
    auto it = tex_.find(k);
    if (it != tex_.end()) {
        lru_.splice(lru_.begin(), lru_, it->second.lru);
        return it->second.id;
    }
    std::lock_guard<std::mutex> lk(mtx_);
    wanted_frame_[k] = frame_;
    if (queued_.count(k)) return 0;
    auto f = failed_.find(k);
    if (f != failed_.end()) {
        if (std::chrono::steady_clock::now() - f->second < std::chrono::seconds(60)) return 0;
        failed_.erase(f);
    }
    queued_.insert(k);
    queue_.push_front(k);
    cv_.notify_one();
    return 0;
}

void MapTiles::upload_ready(int max_per_frame) {
    std::deque<Decoded> batch;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        while (!ready_.empty() && (int)batch.size() < max_per_frame) {
            queued_.erase(ready_.front().k);
            batch.push_back(std::move(ready_.front()));
            ready_.pop_front();
        }
    }
    for (auto &d : batch) {
        unsigned int id = 0;
        glGenTextures(1, &id);
        glBindTexture(GL_TEXTURE_2D, id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, d.w, d.h, 0, GL_RGBA, GL_UNSIGNED_BYTE, d.rgba.data());
        glBindTexture(GL_TEXTURE_2D, 0);
        lru_.push_front(d.k);
        tex_[d.k] = Tex{id, lru_.begin()};
        while (tex_.size() > max_tex_) {
            const uint64_t old = lru_.back();
            lru_.pop_back();
            auto it = tex_.find(old);
            if (it != tex_.end()) { glDeleteTextures(1, &it->second.id); tex_.erase(it); }
        }
    }
}

std::string MapTiles::cache_path(int z, int x, int y) const {
    char b[200];
    std::snprintf(b, sizeof(b), "%s/%d/%d/%d.%s", cache_dir_.c_str(), z, y, x, ext_.c_str());
    return b;
}

bool MapTiles::fetch(int z, int x, int y, std::vector<unsigned char> &bytes, bool &from_disk) {
    const std::string path = cache_path(z, x, y);
    if (read_file(path, bytes)) { from_disk = true; return true; }
    from_disk = false;
    char p[320];
    std::snprintf(p, sizeof(p), path_fmt_.c_str(), z, y, x);
    if (!http_get(host_, p, bytes)) return false;
    make_dirs(path);
    if (FILE *f = std::fopen(path.c_str(), "wb")) {
        std::fwrite(bytes.data(), 1, bytes.size(), f);
        std::fclose(f);
    }
    return true;
}

void MapTiles::worker() {
    int prio_gen = 0;
    for (;;) {
        iridi_refresh_thread_priority(IRIDI_PRIO_NORMAL, &prio_gen);
        uint64_t k;
        {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait(lk, [this] { return stop_ || !queue_.empty(); });
            if (stop_) return;
            k = queue_.front();
            queue_.pop_front();
            // No longer on screen: drop it (it is requested again if needed).
            auto w = wanted_frame_.find(k);
            if (w == wanted_frame_.end() || frame_ - w->second > 2) {
                queued_.erase(k);
                if (w != wanted_frame_.end()) wanted_frame_.erase(w);
                continue;
            }
        }
        const int z = (int)(k >> 58), x = (int)((k >> 29) & 0x1FFFFFFF), y = (int)(k & 0x1FFFFFFF);
        std::vector<unsigned char> bytes;
        bool disk = false;
        bool ok = fetch(z, x, y, bytes, disk);
        Decoded d{k, 0, 0, {}};
        if (ok) {
            int comp = 0;
            unsigned char *px = stbi_load_from_memory(bytes.data(), (int)bytes.size(), &d.w, &d.h, &comp, 4);
            if (px && d.w > 0 && d.h > 0) {
                d.rgba.assign(px, px + (size_t)d.w * d.h * 4);
            } else ok = false;
            if (px) stbi_image_free(px);
        }
        if (ok) {
            (disk ? from_disk_ : downloaded_).fetch_add(1);
            if (!disk) consecutive_fail_.store(0);
        } else if (!disk) {
            consecutive_fail_.fetch_add(1);
        }
        std::lock_guard<std::mutex> lk(mtx_);
        wanted_frame_.erase(k);
        // A decoded tile stays in queued_ until upload_ready() turns it into a
        // texture, so get() can't queue it a second time in between.
        if (ok) ready_.push_back(std::move(d));
        else { queued_.erase(k); failed_[k] = std::chrono::steady_clock::now(); }
    }
}

} // namespace iridi
