// Iridium L3 (Um) signaling decoder, ported from iridium-toolkit's
// reassembler/ida.py (ReassembleIDAPP).
#ifndef IRIDISCOPE_L3_H
#define IRIDISCOPE_L3_H

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace iridi {

struct L3Message {
    uint64_t timestamp_ns = 0;
    double frequency = 0;
    int ul = 0;
    std::string proto;   // CC / MM / SMS / SBD / ...
    std::string type;    // Setup / Location Updating Request / ...
    std::string summary; // decoded fields
    std::string raw_hex;
    bool ok = false;
};

// Decode one reassembled IDA payload (the bytes after IDA reassembly).
bool decode_l3(const std::vector<uint8_t> &data, double time_s, int ul, double freq,
               L3Message &out);

} // namespace iridi

#endif // IRIDISCOPE_L3_H
