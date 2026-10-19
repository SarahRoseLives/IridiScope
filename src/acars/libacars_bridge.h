// Optional libacars-2 bridge for ACARS / ARINC-622 decoding.
#ifndef IRIDISCOPE_LIBACARS_BRIDGE_H
#define IRIDISCOPE_LIBACARS_BRIDGE_H

#include <cstdint>
#include <string>
#include <vector>

namespace iridi {

struct AcarsDecoded {
    bool ok = false;
    bool crc_ok = false;
    bool err = false;
    bool final_block = true;
    std::string mode, reg, ack, label, sublabel, mfi, block_id;
    std::string msg_num, flight, text;
    std::string arinc622_text;
    std::string arinc622_json;
    std::string kind; // ADS-C / CPDLC / OHMA / MIAM
};

bool libacars_available();
void libacars_reset();
AcarsDecoded decode_acars_libacars(const std::vector<uint8_t> &raw, int ul, double time_s);

} // namespace iridi

#endif // IRIDISCOPE_LIBACARS_BRIDGE_H
