// Native C++ port of iridium-toolkit's bitsparser.py.
//
// Consumes demodulated frames (bitstream + metadata) and classifies them into
// Iridium message types (IRA/IBC/IDA/IP/VO/LCW/...). The bit ordering matches
// iridium-toolkit: the demodulated bits are stored as '0'/'1' and adjacent
// symbol pairs are swapped (symbol_reverse), exactly as bitsparser does for
// "RAW" input lines.
#ifndef IRIDISCOPE_IRIDIUM_PARSER_H
#define IRIDISCOPE_IRIDIUM_PARSER_H

#include <cstdint>
#include <string>
#include <vector>

namespace irid {

struct RawFrame {
    std::string bitstream;   // '0'/'1' per demodulated bit (RAW order)
    double   frequency = 0;  // Hz
    uint64_t timestamp_ns = 0;
    int      confidence = 0;
    float    level = 0;      // average amplitude
    float    noise = 0;      // dBFS/Hz
    float    snr = 0;        // = magnitude (dB)
    int      n_symbols = 0;
};

struct ParsedMessage {
    std::string type;        // "IRA","IBC","IDA","IIP","VOC","LW","SY","IME",...
    std::string pretty;      // human-readable summary
    double   frequency = 0;
    uint64_t timestamp_ns = 0;
    int      confidence = 0;
    int      uplink = 0;
    int      symbols = 0;

    // Ring alert
    bool has_ira = false;
    int ira_sat = 0, ira_beam = 0, ira_alt_km = 0, ira_pages = 0;
    double ira_lat = 0, ira_lon = 0;

    // Broadcast channel
    bool has_ibc = false;
    int ibc_sat = 0, ibc_beam = 0, ibc_type = -1;
    uint32_t ibc_iri_time = 0;
    int ibc_has_time = 0;

    // IDA / data
    bool has_ida = false;
    int ida_ctr = 0, ida_len = 0, ida_crc_ok = 0, ida_cont = 0;
    std::vector<uint8_t> ida_data;
    std::string ida_text;    // SBD ASCII preview

    // IMS / pager (MSG / MS3)
    bool has_ims = false;
    bool has_msg = false;
    int msg_ric = 0, msg_fmt = 0, msg_seq = 0, msg_part = 0, msg_parts = 0;
    int msg_checksum = 0;
    bool msg_csum_ok = false;
    std::vector<uint8_t> msg_bytes; // 7-bit characters (fmt 5)
    std::string msg_text;           // ASCII preview (fmt 5) or BCD digits (fmt 3)

    bool error = false;

    // Voice payload bytes (VOC/VOD/VO6), for the voice subsystem.
    std::vector<uint8_t> voice_payload;
};

// Classify a raw demodulated frame. Returns true if a type was assigned.
bool parse_frame(const RawFrame &in, ParsedMessage &out);

} // namespace irid

#endif // IRIDISCOPE_IRIDIUM_PARSER_H
