#include "output/gsmtap.h"

#ifdef _WIN32

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

namespace iridi {

namespace {

std::once_flag g_wsa_once;
std::mutex g_mtx;
SOCKET g_sock = INVALID_SOCKET;
sockaddr_in g_addr{};
std::atomic<bool> g_enabled{false};
std::atomic<uint64_t> g_packets{0};

void close_socket() {
    if (g_sock != INVALID_SOCKET) {
        closesocket(g_sock);
        g_sock = INVALID_SOCKET;
    }
}

} // namespace

bool gsmtap_start(const std::string &host, int port, std::string &err) {
    std::lock_guard<std::mutex> lk(g_mtx);
    close_socket();
    g_enabled.store(false);
    if (port <= 0 || port > 65535) {
        err = "Invalid UDP port";
        return false;
    }
    std::call_once(g_wsa_once, [] {
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);
    });

    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    addrinfo *res = nullptr;
    char portstr[16];
    snprintf(portstr, sizeof(portstr), "%d", port);
    if (getaddrinfo(host.c_str(), portstr, &hints, &res) != 0 || !res) {
        err = "Cannot resolve " + host;
        return false;
    }
    std::memcpy(&g_addr, res->ai_addr, sizeof(sockaddr_in));
    freeaddrinfo(res);

    g_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_sock == INVALID_SOCKET) {
        err = "Cannot create UDP socket";
        return false;
    }
    g_packets.store(0);
    g_enabled.store(true);
    return true;
}

void gsmtap_stop() {
    std::lock_guard<std::mutex> lk(g_mtx);
    g_enabled.store(false);
    close_socket();
}

bool gsmtap_enabled() { return g_enabled.load(); }
uint64_t gsmtap_packets() { return g_packets.load(); }

void gsmtap_send(const std::vector<uint8_t> &lapdm, int ul, double freq_hz) {
    if (!g_enabled.load() || lapdm.empty()) return;

    uint8_t hdr[16];
    hdr[0] = 2;  // version
    hdr[1] = 4;  // header length in 32-bit words
    hdr[2] = 2;  // GSMTAP_TYPE_ABIS
    hdr[3] = 0;  // timeslot

    int chan = (int)((freq_hz - 1616e6) / (1e7 / (30.0 * 8.0)));
    if (chan < 0) chan += 0x2000;
    uint16_t arfcn = (uint16_t)((ul ? 0x4000 : 0) + (chan & 0x3fff));
    arfcn = htons(arfcn);
    std::memcpy(hdr + 4, &arfcn, 2);
    hdr[6] = 0; // signal dBm
    hdr[7] = 0; // SNR dB
    uint32_t fn = htonl((uint32_t)(int64_t)freq_hz);
    std::memcpy(hdr + 8, &fn, 4);
    hdr[12] = 1; // sub_type: BCCH
    hdr[13] = 0; // antenna
    hdr[14] = 0; // sub-slot
    hdr[15] = 0; // reserved

    std::vector<uint8_t> pkt;
    pkt.reserve(16 + lapdm.size());
    pkt.insert(pkt.end(), hdr, hdr + 16);
    pkt.insert(pkt.end(), lapdm.begin(), lapdm.end());

    std::lock_guard<std::mutex> lk(g_mtx);
    if (!g_enabled.load() || g_sock == INVALID_SOCKET) return;
    if (sendto(g_sock, (const char *)pkt.data(), (int)pkt.size(), 0,
               (const sockaddr *)&g_addr, sizeof(g_addr)) != SOCKET_ERROR)
        g_packets.fetch_add(1);
}

} // namespace iridi

#else // !_WIN32

namespace iridi {
bool gsmtap_start(const std::string &, int, std::string &err) {
    err = "GSMTAP is only implemented on Windows";
    return false;
}
void gsmtap_stop() {}
bool gsmtap_enabled() { return false; }
uint64_t gsmtap_packets() { return 0; }
void gsmtap_send(const std::vector<uint8_t> &, int, double) {}
} // namespace iridi

#endif // _WIN32
