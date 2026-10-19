// GSMTAP/LAPDm UDP output for Wireshark.
#ifndef IRIDISCOPE_GSMTAP_H
#define IRIDISCOPE_GSMTAP_H

#include <cstdint>
#include <string>
#include <vector>

namespace iridi {

bool gsmtap_start(const std::string &host, int port, std::string &err);
void gsmtap_stop();
bool gsmtap_enabled();
uint64_t gsmtap_packets();

// Send one reassembled IDA/LAPDm payload. No-op when disabled.
void gsmtap_send(const std::vector<uint8_t> &lapdm, int ul, double freq_hz);

} // namespace iridi

#endif // IRIDISCOPE_GSMTAP_H
