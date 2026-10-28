// Online satellite imagery for the map: Esri World Imagery tiles (Web
// Mercator, 256 px), fetched on background threads, kept on disk in
// mapcache/ (so places you've looked at stay sharp offline) and in GPU
// textures (recently used ones). The built-in world image stays underneath as
// the offline base; tiles sharpen it wherever they're available.
#ifndef IRIDISCOPE_MAP_TILES_H
#define IRIDISCOPE_MAP_TILES_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <list>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace iridi {

class MapTiles {
public:
    MapTiles() = default;
    ~MapTiles() { stop(); }

    // Tile server: host, path printf format taking (z, y, x), cache folder,
    // file extension. Call before start(). Default: Esri World Imagery.
    void configure(const std::wstring &host, const std::string &path_fmt, const std::string &cache_dir,
                   const std::string &ext) {
        host_ = host; path_fmt_ = path_fmt; cache_dir_ = cache_dir; ext_ = ext;
    }
    void start(int threads = 4);
    void stop();

    // Main thread, once per frame: returns the GL texture for a tile if it
    // is ready, otherwise queues it (newest requests first) and returns 0.
    unsigned int get(int z, int x, int y);
    // Main thread: the texture if already loaded, without requesting it
    // (used to show a coarser tile while the sharper one loads).
    unsigned int peek(int z, int x, int y) const {
        auto it = tex_.find(key(z, x, y));
        return it == tex_.end() ? 0 : it->second.id;
    }
    // Main thread: upload tiles decoded since the last frame (bounded per
    // frame so a burst of tiles can't stall the UI).
    void upload_ready(int max_per_frame = 12);
    // Called at the start of each frame's requests: older queued requests
    // that are no longer on screen get dropped.
    void begin_frame();

    // Status for the UI.
    int pending() const;
    bool offline() const { return consecutive_fail_.load() >= 6; }
    uint64_t downloaded() const { return downloaded_.load(); }
    uint64_t from_disk() const { return from_disk_.load(); }

private:
    static uint64_t key(int z, int x, int y) {
        return ((uint64_t)z << 58) | ((uint64_t)(uint32_t)x << 29) | (uint64_t)(uint32_t)y;
    }
    struct Decoded { uint64_t k; int w, h; std::vector<unsigned char> rgba; };
    struct Tex { unsigned int id; std::list<uint64_t>::iterator lru; };

    void worker();
    bool fetch(int z, int x, int y, std::vector<unsigned char> &bytes, bool &from_disk);
    std::string cache_path(int z, int x, int y) const;

    // GPU cache (main thread only).
    std::unordered_map<uint64_t, Tex> tex_;
    std::list<uint64_t> lru_;                 // front = most recently used
    size_t max_tex_ = 384;                    // ~100 MB of RGBA tiles

    // Work queue (shared with workers).
    mutable std::mutex mtx_;
    std::condition_variable cv_;
    std::deque<uint64_t> queue_;              // front = most recent request
    std::unordered_set<uint64_t> queued_;     // queued or in flight
    std::unordered_map<uint64_t, std::chrono::steady_clock::time_point> failed_;
    std::deque<Decoded> ready_;
    std::vector<std::thread> threads_;
    bool stop_ = false;
    uint64_t frame_ = 0;
    std::unordered_map<uint64_t, uint64_t> wanted_frame_; // last frame a tile was requested

    std::wstring host_ = L"server.arcgisonline.com";
    std::string path_fmt_ = "/ArcGIS/rest/services/World_Imagery/MapServer/tile/%d/%d/%d";
    std::string cache_dir_ = "mapcache/esri";
    std::string ext_ = "jpg";
    std::atomic<int> consecutive_fail_{0};
    std::atomic<uint64_t> downloaded_{0}, from_disk_{0};
};

} // namespace iridi

#endif // IRIDISCOPE_MAP_TILES_H
