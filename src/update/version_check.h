// Background update check: GETs a version endpoint, compares the published
// version to the local one, and reports whether an update is available.
// Adapted from AirScope's src/update/version_check.{h,cpp}.
#ifndef IRIDISCOPE_VERSION_CHECK_H
#define IRIDISCOPE_VERSION_CHECK_H

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

namespace iridi {

class VersionCheck {
public:
    enum State { Idle, Checking, UpToDate, UpdateAvailable, Unreleased, Error };

    ~VersionCheck();

    // Kick off the check once (no-op if already started). `url` is the full
    // version endpoint; `local_version` is the running app's version.
    void start(const std::string &url, const std::string &local_version);

    State state() const { return state_.load(); }
    std::string latest_version();
    std::string product_url();
    std::string error();

private:
    void run(std::string url, std::string local_version);

    std::atomic<State> state_{Idle};
    std::atomic<bool> started_{false};
    std::thread thread_;
    std::mutex mtx_;
    std::string latest_, url_, err_;
};

} // namespace iridi

#endif // IRIDISCOPE_VERSION_CHECK_H
