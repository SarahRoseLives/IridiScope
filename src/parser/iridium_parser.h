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
#include <atomic>
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
    int ira_rai = 0, ira_bc_sb = 0;
    bool ira_is_ground = false;
    double ira_lat = 0, ira_lon = 0;
    struct IraPage {
        uint32_t tmsi = 0;
        int msc_id = 0;
    };
    std::vector<IraPage> ira_page_list;

    // Broadcast channel
    bool has_ibc = false;
    int ibc_sat = 0, ibc_beam = 0, ibc_type = -1;
    int ibc_subtype = -1;
    int ibc_slot = 0, ibc_sv_blocking = 0;
    int ibc_aq_subband = 0, ibc_aq_channel = 0;
    std::string ibc_aq_classes;
    int ibc_max_uplink_pwr = 0;
    uint32_t ibc_iri_time = 0, ibc_tmsi_expiry = 0;
    int ibc_has_time = 0, ibc_has_tmsi_expiry = 0;
    struct IbAssignment {
        int type = 0, random_id = 0, timeslot = 0;
        int ul_sb = 0, dl_sb = 0, access = 0, dtoa = 0, dfoa = 0;
        bool empty = false;
        std::string raw;
    };
    std::vector<IbAssignment> ibc_assignments;

    // Time & Location (ITL) message
    bool has_itl = false;
    int itl_version = -1, itl_plane = 0, itl_ok = 0;
    std::string itl_sat, itl_msg;
    std::vector<int> itl_words;

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
    // Symbols of the 12-symbol unique word that were wrong but tolerated
    // (iridium-toolkit --uw-ec); 0 = exact match.
    int uw_errors = 0;

    // Voice payload bytes (VOC/VOD/VO6), for the voice subsystem.
    std::vector<uint8_t> voice_payload;
    std::string voice_subtype; // VOC/VOD/VOZ/VDA/VO6
    // LCW handoff response on a traffic channel: the call's next channel.
    bool has_handoff = false;
    int ho_slot = 0, ho_sband_up = 0, ho_sband_dn = 0, ho_access = 0;
    bool ho_denied = false;
};

// Classify a raw demodulated frame. Returns true if a type was assigned.
bool parse_frame(const RawFrame &in, ParsedMessage &out);

// Unique-word error tolerance (iridium-toolkit --uw-ec): accept the access
// code with up to 3 wrong DQPSK symbols. Off by default upstream.
extern std::atomic<bool> irid_uw_ec;

} // namespace irid

#endif // IRIDISCOPE_IRIDIUM_PARSER_H
