#include "acars/aircraft_lookup.h"
#include "util/json_mini.h"
#include "util/http_get.h"

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <winhttp.h>
#endif

#include <cctype>
#include <chrono>
#include <cstdio>
#include <vector>

namespace iridi {

namespace {

std::string norm_key(const std::string &in) {
    size_t a = 0, b = in.size();
    while (a < b && std::isspace((unsigned char)in[a])) ++a;
    while (b > a && std::isspace((unsigned char)in[b - 1])) --b;
    std::string s = in.substr(a, b - a);
    for (char &c : s) c = (char)std::toupper((unsigned char)c);
    return s;
}

std::string url_encode(const std::string &s) {
    static const char *hex = "0123456789ABCDEF";
    std::string out;
    for (unsigned char c : s) {
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
            c == '-' || c == '_' || c == '.' || c == '~') {
            out += (char)c;
        } else {
            out += '%';
            out += hex[c >> 4];
            out += hex[c & 0x0F];
        }
    }
    return out;
}

#if defined(_WIN32)
bool http_get(const std::string &url, std::string &body, int &status) {
    status = 0;
    body.clear();

    std::wstring wurl;
    wurl.reserve(url.size());
    for (char c : url) wurl += (wchar_t)(unsigned char)c;

    URL_COMPONENTS uc{};
    uc.dwStructSize = sizeof(uc);
    wchar_t host[256] = {0}, path[1024] = {0}, extra[1024] = {0};
    uc.lpszHostName = host;
    uc.dwHostNameLength = 255;
    uc.lpszUrlPath = path;
    uc.dwUrlPathLength = 1023;
    uc.lpszExtraInfo = extra;
    uc.dwExtraInfoLength = 1023;
    if (!WinHttpCrackUrl(wurl.c_str(), (DWORD)wurl.size(), 0, &uc) || host[0] == 0)
        return false;
    std::wstring full_path(path);
    full_path += extra;
    const bool secure = (uc.nScheme == INTERNET_SCHEME_HTTPS);

    HINTERNET hS = WinHttpOpen(L"IridiScope", WINHTTP_ACCESS_TYPE_DEFAULT_PROXY,
                               WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
    if (!hS) return false;
    WinHttpSetTimeouts(hS, 8000, 8000, 8000, 8000);

    HINTERNET hC = WinHttpConnect(hS, host,
                                  secure ? INTERNET_DEFAULT_HTTPS_PORT
                                         : INTERNET_DEFAULT_HTTP_PORT,
                                  0);
    if (!hC) { WinHttpCloseHandle(hS); return false; }

    HINTERNET hR = WinHttpOpenRequest(hC, L"GET", full_path.c_str(), nullptr,
                                      WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES,
                                      secure ? WINHTTP_FLAG_SECURE : 0);
    if (!hR) { WinHttpCloseHandle(hC); WinHttpCloseHandle(hS); return false; }

    bool ok = WinHttpSendRequest(hR, WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA,
                                 0, 0, 0) &&
              WinHttpReceiveResponse(hR, nullptr);
    if (ok) {
        DWORD code = 0, len = sizeof(code);
        if (WinHttpQueryHeaders(hR, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
                                 WINHTTP_HEADER_NAME_BY_INDEX, &code, &len,
                                 WINHTTP_NO_HEADER_INDEX))
            status = (int)code;

        DWORD avail = 0;
        do {
            avail = 0;
            if (!WinHttpQueryDataAvailable(hR, &avail) || avail == 0) break;
            std::vector<char> buf(avail);
            DWORD read = 0;
            if (!WinHttpReadData(hR, buf.data(), avail, &read) || read == 0) break;
            body.append(buf.data(), read);
        } while (avail > 0);
    }
    WinHttpCloseHandle(hR);
    WinHttpCloseHandle(hC);
    WinHttpCloseHandle(hS);
    return ok;
}
#else
// macOS / Linux: libcurl (see util/http_get.cpp).
bool http_get(const std::string &url, std::string &body, int &status) {
    return http_get_portable(url, body, status);
}
#endif

} // namespace

AircraftLookup::~AircraftLookup() { stop(); }

void AircraftLookup::start() {
    if (running_.load()) return;
    running_.store(true);
    thread_ = std::thread(&AircraftLookup::worker, this);
}

void AircraftLookup::stop() {
    if (!running_.exchange(false)) return;
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
}

void AircraftLookup::request_aircraft(const std::string &registration, bool force) {
    if (!enabled_.load() || !running_.load()) return;
    std::string key = norm_key(registration);
    if (key.empty()) return;

    std::lock_guard<std::mutex> lk(mtx_);
    auto it = aircraft_.find(key);
    if (it != aircraft_.end()) {
        if (it->second.status == AircraftInfo::Pending) return;
        if (!force && it->second.status != AircraftInfo::None) return;
    }
    AircraftInfo pending;
    pending.status = AircraftInfo::Pending;
    aircraft_[key] = pending;
    queue_.push_back({Job::Aircraft, key});
    cv_.notify_one();
}

void AircraftLookup::request_route(const std::string &flight, bool force) {
    if (!enabled_.load() || !running_.load()) return;
    std::string key = norm_key(flight);
    if (key.empty()) return;

    std::lock_guard<std::mutex> lk(mtx_);
    auto it = routes_.find(key);
    if (it != routes_.end()) {
        if (it->second.status == AircraftInfo::Pending) return;
        if (!force && it->second.status != AircraftInfo::None) return;
    }
    AircraftInfo pending;
    pending.status = AircraftInfo::Pending;
    routes_[key] = pending;
    queue_.push_back({Job::Route, key});
    cv_.notify_one();
}

AircraftInfo AircraftLookup::get_aircraft(const std::string &registration) {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = aircraft_.find(norm_key(registration));
    if (it == aircraft_.end()) return AircraftInfo{};
    return it->second;
}

AircraftInfo AircraftLookup::get_route(const std::string &flight) {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = routes_.find(norm_key(flight));
    if (it == routes_.end()) return AircraftInfo{};
    return it->second;
}

void AircraftLookup::worker() {
    while (running_.load()) {
        Job job;
        {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait(lk, [this] { return !queue_.empty() || !running_.load(); });
            if (!running_.load()) break;
            job = std::move(queue_.front());
            queue_.pop_front();
        }

        if (job.kind == Job::Aircraft)
            do_aircraft(job.key);
        else
            do_route(job.key);

        // Be gentle with the free public API.
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    }
}

void AircraftLookup::do_aircraft(const std::string &reg) {
    AircraftInfo info;
    info.status = AircraftInfo::Error;
    info.registration = reg;

    std::string body;
    int status = 0;
    const std::string url = "https://api.adsbdb.com/v0/aircraft/" + url_encode(reg);
    bool ok = http_get(url, body, status);

    if (!ok) {
        info.error = "network error";
    } else if (status == 404) {
        info.status = AircraftInfo::NotFound;
    } else if (status != 200) {
        info.error = "HTTP " + std::to_string(status);
    } else {
        jsonmini::Value root;
        if (!jsonmini::parse(body, root)) {
            info.error = "bad response";
        } else {
            const jsonmini::Value *resp = root.find("response");
            const jsonmini::Value *ac = resp ? resp->find("aircraft") : nullptr;
            if (!ac) {
                info.status = AircraftInfo::NotFound;
            } else {
                info.status = AircraftInfo::Ok;
                info.registration = ac->s("registration");
                info.owner = ac->s("registered_owner");
                info.type = ac->s("type");
                info.icao_type = ac->s("icao_type");
                info.manufacturer = ac->s("manufacturer");
                info.mode_s = ac->s("mode_s");
                info.country = ac->s("registered_owner_country_name");
                info.operator_code = ac->s("registered_owner_operator_flag_code");
                info.photo_url = ac->s("url_photo");
                info.photo_thumb = ac->s("url_photo_thumbnail");
                if (info.registration.empty()) info.registration = reg;
            }
        }
    }

    std::lock_guard<std::mutex> lk(mtx_);
    aircraft_[reg] = std::move(info);
}

void AircraftLookup::do_route(const std::string &flight) {
    AircraftInfo info;
    info.status = AircraftInfo::Error;

    std::string body;
    int status = 0;
    const std::string url = "https://api.adsbdb.com/v0/callsign/" + url_encode(flight);
    bool ok = http_get(url, body, status);

    if (!ok) {
        info.error = "network error";
    } else if (status == 404) {
        info.status = AircraftInfo::NotFound;
    } else if (status != 200) {
        info.error = "HTTP " + std::to_string(status);
    } else {
        jsonmini::Value root;
        if (!jsonmini::parse(body, root)) {
            info.error = "bad response";
        } else {
            const jsonmini::Value *resp = root.find("response");
            const jsonmini::Value *fr = resp ? resp->find("flightroute") : nullptr;
            if (!fr) {
                info.status = AircraftInfo::NotFound;
            } else {
                info.status = AircraftInfo::Ok;
                const jsonmini::Value *airline = fr->find("airline");
                if (airline) info.airline = airline->s("name");
                const jsonmini::Value *origin = fr->find("origin");
                if (origin) info.origin = origin->s("iata_code");
                const jsonmini::Value *dest = fr->find("destination");
                if (dest) info.destination = dest->s("iata_code");
            }
        }
    }

    std::lock_guard<std::mutex> lk(mtx_);
    routes_[flight] = std::move(info);
}

} // namespace iridi
