// Asynchronous aircraft/registration and flight-route lookups via adsbdb.com
// (keyless). Runs on a background thread with an in-memory cache and a modest
// rate limit so the UI never blocks and the public API is not hammered.
#ifndef IRIDISCOPE_AIRCRAFT_LOOKUP_H
#define IRIDISCOPE_AIRCRAFT_LOOKUP_H

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

namespace iridi {

struct AircraftInfo {
    enum Status { None, Pending, Ok, NotFound, Error };

    Status status = None;

    // Aircraft (registration) lookup.
    std::string registration;
    std::string owner;       // registered_owner / operator
    std::string type;        // e.g. "747 428FSCD"
    std::string icao_type;   // e.g. "B744"
    std::string manufacturer;
    std::string mode_s;      // ICAO 24-bit hex
    std::string country;
    std::string operator_code;
    std::string photo_url;
    std::string photo_thumb;

    // Flight-route (callsign) lookup.
    std::string airline;
    std::string origin;      // IATA code
    std::string destination; // IATA code

    std::string error;
};

class AircraftLookup {
public:
    AircraftLookup() = default;
    ~AircraftLookup();

    AircraftLookup(const AircraftLookup &) = delete;
    AircraftLookup &operator=(const AircraftLookup &) = delete;

    void start();
    void stop();

    void set_enabled(bool on) { enabled_.store(on); }
    bool enabled() const { return enabled_.load(); }

    // Queue a lookup (cached results are returned via get_* immediately).
    void request_aircraft(const std::string &registration, bool force = false);
    void request_route(const std::string &flight, bool force = false);

    // Snapshot of the current result (Status::None if never requested).
    AircraftInfo get_aircraft(const std::string &registration);
    AircraftInfo get_route(const std::string &flight);

private:
    struct Job {
        enum Kind { Aircraft, Route } kind;
        std::string key;
    };

    void worker();
    void do_aircraft(const std::string &reg);
    void do_route(const std::string &flight);

    std::atomic<bool> enabled_{true};
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::mutex mtx_;
    std::condition_variable cv_;

    std::deque<Job> queue_;
    std::unordered_map<std::string, AircraftInfo> aircraft_;
    std::unordered_map<std::string, AircraftInfo> routes_;
};

} // namespace iridi

#endif // IRIDISCOPE_AIRCRAFT_LOOKUP_H
