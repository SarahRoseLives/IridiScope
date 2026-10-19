// IDA multi-burst reassembly + SBD reassembly + ACARS decoding.
// Ported from iridium-toolkit (reassembler/ida.py, reassembler/sbd.py).
#ifndef IRIDISCOPE_ACARS_H
#define IRIDISCOPE_ACARS_H

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "parser/iridium_parser.h"
#include "messages/l3.h"

namespace iridi {

struct AcarsMessage {
    uint64_t timestamp_ns = 0;
    double frequency = 0;
    int ul = 0;
    std::string heading;   // SBD preheader summary
    std::string reg;       // aircraft registration
    std::string flight;
    std::string mode;
    std::string ack;
    std::string label;     // 2-char label
    std::string label_desc;
    std::string bid;
    std::string seqn;
    std::string sublabel;
    std::string mfi;
    std::string msg_num;
    std::string text;
    std::string raw_hex;   // SBD payload hex
    std::string errors;    // space-joined error tags
    std::string arinc622_text;
    std::string arinc622_json;
    std::string arinc622_kind;
    bool has_arinc622 = false;
    bool crc_ok = false;
    bool cont = false;
};

// GSM 03.40 SMS recovered from IDA CP-DATA (protocol 0x09/0x89, type 0x01).
struct SmsMessage {
    uint64_t timestamp_ns = 0;
    double frequency = 0;
    int ul = 0;
    std::string from;   // SMS-DELIVER originator
    std::string to;     // SMS-SUBMIT destination
    std::string text;
    std::string dcs;
    bool ok = false;
};

// Reassembled Iridium Short Burst Data (SBD) message.
struct SbdMessage {
    uint64_t timestamp_ns = 0;
    double frequency = 0;
    int ul = 0;
    std::string type;         // e.g. "7608", "0600"
    int length = 0;
    std::string preheader_hex;
    std::string payload_hex;
    std::string text;         // printable-ASCII preview of the payload
    std::string info;         // decoded preheader (IMEI/MOMSN/MTMSN/CRC/...)
    bool acars = false;       // payload is an ACARS message
    bool has_crc = false;     // SBD preheader carried a CRC
    bool crc_ok = false;

    // NAL10 position report (10-byte SBD payload).
    bool has_nal = false;
    double nal_lat = 0, nal_lon = 0;
    int nal_pdop = 0, nal_seconds = 0;
    bool nal_fix = false, nal_emergency = false, nal_emergency_ack = false,
         nal_motion = false;
    std::string nal_info;
};

// Decode a GSM 03.40 SMS TPDU (exposed for diagnostics).
bool decode_sms_tpdu(const std::vector<uint8_t> &tpdu, SmsMessage &out);

class AcarsAssembler {
public:
    AcarsAssembler() = default;

    void reset();
    void feed(const irid::ParsedMessage &pm); // thread-safe
    std::vector<AcarsMessage> messages() const;
    std::vector<SmsMessage> sms_messages() const;
    std::vector<SbdMessage> sbd_messages() const;
    std::vector<L3Message> l3_messages() const;
    uint64_t ida_assembled() const { return ida_ok_; }
    uint64_t sbd_packets() const { return sbd_cnt_; }
    uint64_t acars_count() const { return acars_cnt_; }
    uint64_t sms_count() const { return sms_cnt_; }
    uint64_t sbd_message_count() const { return sbd_msg_cnt_; }
    uint64_t l3_count() const { return l3_cnt_; }

private:
    struct SbdFrag {
        double freq = 0;
        std::vector<double> times;
        int ctr = 0;
        std::vector<uint8_t> data;
        bool cont = false;
        int ul = 0;
    };

    void emit_sbd_payload(const std::vector<uint8_t> &data, double time_s, int ul,
                          double frequency);
    void record_sbd(const std::string &typ, const std::vector<uint8_t> &prehdr,
                    const std::vector<uint8_t> &data, double time_s, int ul,
                    double frequency);
    void decode_acars(const std::vector<uint8_t> &data, double time_s, int ul,
                      const std::vector<uint8_t> &prehdr, const std::string &typ,
                      double frequency);

    mutable std::mutex mtx_;
    std::vector<SbdFrag> ida_buf_;
    struct MultiPkt {
        int msgno, msgcnt, ul;
        double time;
        std::vector<uint8_t> data;
        std::string typ;
        std::vector<uint8_t> prehdr;
    };
    std::vector<MultiPkt> multi_;
    std::deque<AcarsMessage> msgs_;
    std::deque<SmsMessage> sms_;
    std::deque<SbdMessage> sbd_msgs_;
    std::deque<L3Message> l3_;

    double otime_ = -1e9;
    std::vector<uint8_t> odata_;
    double ofreq_ = 0;
    uint64_t ida_ok_ = 0, ida_frag_ = 0, ida_broken_ = 0;
    uint64_t sbd_cnt_ = 0, acars_cnt_ = 0, sms_cnt_ = 0, sbd_msg_cnt_ = 0;
    uint64_t l3_cnt_ = 0;
};

} // namespace iridi

#endif // IRIDISCOPE_ACARS_H
