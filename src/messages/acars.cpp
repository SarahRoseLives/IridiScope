#include "messages/acars.h"
#include "acars/libacars_bridge.h"
#include "output/gsmtap.h"
#include "parser/fec.h"
#include "util/iritime.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>

namespace iridi {

// ---- helpers ---------------------------------------------------------------
static std::string hex(const std::vector<uint8_t> &v) {
    std::string s;
    char b[4];
    for (size_t i = 0; i < v.size(); i++) {
        if (i) s += ' ';
        snprintf(b, sizeof(b), "%02x", v[i]);
        s += b;
    }
    return s;
}
static std::string hexns(const std::vector<uint8_t> &v) {
    std::string s;
    char b[4];
    for (uint8_t c : v) { snprintf(b, sizeof(b), "%02x", c); s += b; }
    return s;
}
static std::string printable(const std::string &in) {
    std::string o;
    for (unsigned char c : in) o += (c >= 32 && c < 127) ? (char)c : '.';
    return o;
}
static std::string trim_right(const std::string &s) {
    size_t b = s.size();
    while (b > 0 && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\0')) --b;
    return s.substr(0, b);
}
static std::string label_desc(const std::string &label) {
    if (label.size() == 2 && label[0] == '_' && (unsigned char)label[1] == 0x7f)
        return "Demand mode";
    if (label == "H1") return "Message to/from terminal";
    if (label == "52") return "Ground UTC request";
    if (label == "C1") return "Uplink to cockpit printer 1";
    if (label == "C2") return "Uplink to cockpit printer 2";
    if (label == "C3") return "Uplink to cockpit printer 3";
    if (label == "Q0") return "Link Test";
    return "?";
}

// ---- GSM 03.40 / 04.11 SMS ------------------------------------------------
static std::string gsm7_char(int v) {
    static const char *tbl[128] = {
        "@", "\xC2\xA3", "$", "\xC2\xA5", "\xC3\xA8", "\xC3\xA9", "\xC3\xB9", "\xC3\xAC",
        "\xC3\xB2", "\xC3\x87", "\n", "\xC3\x98", "\xC3\xB8", "\r", "\xC3\x85", "\xC3\xA5",
        "\xCE\x94", "_", "\xCE\xA6", "\xCE\x93", "\xCE\x9B", "\xCE\xA9", "\xCE\xA0", "\xCE\xA8",
        "\xCE\xA3", "\xCE\x98", "\xCE\x9E", "", "\xC3\x86", "\xC3\xA6", "\xC3\x9F", "\xC3\x89",
        " ", "!", "\"", "#", "\xC2\xA4", "%", "&", "'", "(", ")", "*", "+", ",", "-", ".", "/",
        "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", ":", ";", "<", "=", ">", "?",
        "\xC2\xA1", "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O",
        "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z",
        "\xC3\x84", "\xC3\x96", "\xC3\x91", "\xC3\x9C", "\xC2\xA7",
        "\xC2\xBF", "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n", "o",
        "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z",
        "\xC3\xA4", "\xC3\xB6", "\xC3\xB1", "\xC3\xBC", "\xC3\xA0"};
    if (v < 0 || v > 127) return "?";
    return tbl[v];
}

static char bcd_char(int v) {
    if (v >= 0 && v <= 9) return (char)('0' + v);
    if (v == 10) return '*';
    if (v == 11) return '#';
    if (v == 12) return 'a';
    if (v == 13) return 'b';
    if (v == 14) return 'c';
    return 0; // filler
}

static std::string decode_bcd_number(const std::vector<uint8_t> &b, size_t off, int nbytes,
                                     int ndigits) {
    std::string out;
    for (int i = 0; i < nbytes && off + (size_t)i < b.size(); i++) {
        int lo = b[off + i] & 0x0F;
        int hi = (b[off + i] >> 4) & 0x0F;
        char c = bcd_char(lo);
        if (c && (int)out.size() < ndigits) out += c;
        c = bcd_char(hi);
        if (c && (int)out.size() < ndigits) out += c;
    }
    return out;
}

static std::string unpack_gsm7(const std::vector<uint8_t> &data, int septets) {
    std::string out;
    int bit = 0;
    bool esc = false;
    for (int i = 0; i < septets; i++) {
        int byte = bit / 8, shift = bit % 8;
        if (byte >= (int)data.size()) break;
        int v = data[byte] >> shift;
        if (shift > 1 && byte + 1 < (int)data.size()) v |= data[byte + 1] << (8 - shift);
        v &= 0x7F;
        bit += 7;
        if (v == 0x1B) { esc = true; continue; }
        if (esc) {
            esc = false;
            switch (v) {
            case 0x0A: out += '\f'; break;
            case 0x14: out += '^'; break;
            case 0x28: out += '{'; break;
            case 0x29: out += '}'; break;
            case 0x2F: out += '\\'; break;
            case 0x3C: out += '['; break;
            case 0x3D: out += '~'; break;
            case 0x3E: out += ']'; break;
            case 0x40: out += '|'; break;
            case 0x65: out += "\xE2\x82\xAC"; break;
            default: out += '?'; break;
            }
            continue;
        }
        out += gsm7_char(v);
    }
    return out;
}

static std::string ucs2_to_utf8(const std::vector<uint8_t> &d, int nbytes) {
    std::string out;
    int n = std::min<int>(nbytes, (int)d.size());
    for (int i = 0; i + 1 < n; i += 2) {
        unsigned cp = ((unsigned)d[i] << 8) | d[i + 1];
        if (cp < 0x80) {
            out += (char)cp;
        } else if (cp < 0x800) {
            out += (char)(0xC0 | (cp >> 6));
            out += (char)(0x80 | (cp & 0x3F));
        } else {
            out += (char)(0xE0 | (cp >> 12));
            out += (char)(0x80 | ((cp >> 6) & 0x3F));
            out += (char)(0x80 | (cp & 0x3F));
        }
    }
    return out;
}

bool decode_sms_tpdu(const std::vector<uint8_t> &t, SmsMessage &m) {
    if (t.size() < 2) return false;
    int mti = t[0] & 0x03;
    size_t i = 1;
    int dcs = 0, udl = 0;
    std::vector<uint8_t> ud;

    if (mti == 0) { // SMS-DELIVER
        if (i + 1 >= t.size()) return false;
        int oa_len = t[i++];
        int toa = t[i++];
        int oa_bytes = (oa_len + 1) / 2;
        m.from = decode_bcd_number(t, i, oa_bytes, oa_len);
        if ((toa & 0x70) == 0x10 && !m.from.empty()) m.from.insert(m.from.begin(), '+');
        i += oa_bytes;
        if (i + 2 + 7 + 1 > t.size()) return false;
        (void)t[i++]; // TP-PID
        dcs = t[i++];
        i += 7; // TP-SCTS
        udl = t[i++];
        ud.assign(t.begin() + i, t.end());
    } else if (mti == 1) { // SMS-SUBMIT
        if (i >= t.size()) return false;
        i++; // TP-MR
        if (i + 1 >= t.size()) return false;
        int da_len = t[i++];
        int toa = t[i++];
        int da_bytes = (da_len + 1) / 2;
        m.to = decode_bcd_number(t, i, da_bytes, da_len);
        if ((toa & 0x70) == 0x10 && !m.to.empty()) m.to.insert(m.to.begin(), '+');
        i += da_bytes;
        if (i + 2 > t.size()) return false;
        (void)t[i++]; // TP-PID
        dcs = t[i++];
        int vpf = (t[0] >> 3) & 0x03;
        if (vpf == 2 || vpf == 1) i += 1;
        else if (vpf == 3) i += 7;
        if (i >= t.size()) return false;
        udl = t[i++];
        ud.assign(t.begin() + i, t.end());
    } else {
        return false; // status report / command
    }

    int alpha;
    unsigned cgbits = (unsigned)(dcs >> 4);
    if ((cgbits & 0xC) == 0) {
        alpha = (dcs >> 2) & 0x03;
        if (alpha > 2) alpha = 0;
    } else if (cgbits == 0xC || cgbits == 0xD) {
        alpha = 0;
    } else if (cgbits == 0xE) {
        alpha = 2;
    } else if (cgbits == 0xF) {
        alpha = (dcs & 4) ? 1 : 0;
    } else {
        alpha = 0;
    }

    if (alpha == 0) {
        m.text = unpack_gsm7(ud, udl);
    } else if (alpha == 2) {
        m.text = ucs2_to_utf8(ud, udl);
    } else {
        int n = std::min<int>(udl, (int)ud.size());
        for (int k = 0; k < n; k++) {
            unsigned char c = ud[k];
            m.text += (c >= 32 && c < 127) ? (char)c : '.';
        }
    }
    char d[16];
    snprintf(d, sizeof(d), "0x%02x", dcs);
    m.dcs = d;
    m.ok = true;
    return true;
}

void AcarsAssembler::reset() {
    std::lock_guard<std::mutex> lk(mtx_);
    libacars_reset();
    ida_buf_.clear();
    multi_.clear();
    msgs_.clear();
    sms_.clear();
    sbd_msgs_.clear();
    l3_.clear();
    otime_ = -1e9; odata_.clear(); ofreq_ = 0;
    ida_ok_ = ida_frag_ = ida_broken_ = 0;
    sbd_cnt_ = acars_cnt_ = sms_cnt_ = sbd_msg_cnt_ = l3_cnt_ = 0;
}

std::vector<SmsMessage> AcarsAssembler::sms_messages() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return std::vector<SmsMessage>(sms_.begin(), sms_.end());
}

std::vector<SbdMessage> AcarsAssembler::sbd_messages() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return std::vector<SbdMessage>(sbd_msgs_.begin(), sbd_msgs_.end());
}

std::vector<L3Message> AcarsAssembler::l3_messages() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return std::vector<L3Message>(l3_.begin(), l3_.end());
}

std::vector<AcarsMessage> AcarsAssembler::messages() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return std::vector<AcarsMessage>(msgs_.begin(), msgs_.end());
}

// ---- ACARS decode ----------------------------------------------------------
void AcarsAssembler::decode_acars(const std::vector<uint8_t> &in, double time_s,
                                  int ul, const std::vector<uint8_t> &prehdr,
                                  const std::string &typ, double frequency) {
    if (in.empty() || in[0] != 1) return;
    if (in.size() <= 2) return;

    AcarsMessage m;
    m.timestamp_ns = (uint64_t)(time_s * 1e9);
    m.frequency = frequency;
    m.ul = ul;
    m.heading = typ + " <" + hexns(prehdr) + ">";

    std::vector<uint8_t> d(in.begin() + 1, in.end()); // skip 0x01

    uint8_t csum[2] = {0, 0};
    int csumn = 0;
    if (!d.empty() && d.back() == 0x7f && d.size() >= 3) {
        csum[0] = d[d.size() - 3];
        csum[1] = d[d.size() - 2];
        csumn = 2;
        d.resize(d.size() - 3);
    }

    if (!d.empty() && d[0] == 0x3 && d.size() >= 8)
        d.erase(d.begin(), d.begin() + 8); // header of unknown meaning

    std::string errors;
    if (csumn == 2) {
        std::vector<uint8_t> buf = d;
        buf.push_back(csum[0]);
        buf.push_back(csum[1]);
        if (irid::kermit_crc(buf.data(), buf.size()) == 0) m.crc_ok = true;
        else errors += "CRC_FAIL ";
    } else {
        errors += "CRC_MISSING ";
    }

    if (d.size() < 13) { errors += "TRUNCATED"; return; } // throw away

    bool ok = true;
    for (uint8_t c : d) if (__builtin_popcount(c) % 2 == 0) ok = false;
    for (auto &c : d) c &= 0x7f;
    if (!ok) errors += "PARITY_FAIL ";

    m.mode.assign(1, (char)d[0]);
    m.reg.assign((const char *)&d[1], 7);
    m.ack.assign(1, (char)d[8]);
    m.label.assign((const char *)&d[9], 2);
    m.bid.assign(1, (char)d[11]);

    std::vector<uint8_t> rest(d.begin() + 12, d.end());
    if (!rest.empty() && rest.back() == 0x03) rest.pop_back();
    else if (!rest.empty() && rest.back() == 0x17) { m.cont = true; rest.pop_back(); }
    else errors += "ETX ";

    if (!rest.empty() && rest[0] == 2) {
        if (ul) {
            if (rest.size() >= 11) {
                m.seqn.assign((const char *)&rest[1], 4);
                m.flight.assign((const char *)&rest[5], 6);
                m.text.assign((const char *)&rest[11], rest.size() - 11);
            }
        } else {
            m.text.assign((const char *)&rest[1], rest.size() - 1);
        }
    }

    while (!m.reg.empty() && m.reg[0] == '.') m.reg.erase(0, 1);

    m.errors = errors;
    m.raw_hex = hex(in);
    m.label_desc = label_desc(m.label);
    m.text = printable(m.text);
    m.reg = trim_right(printable(m.reg));
    m.flight = trim_right(m.flight);

    AcarsDecoded ld = decode_acars_libacars(in, ul, time_s);
    if (ld.ok) {
        m.crc_ok = ld.crc_ok;
        if (!ld.sublabel.empty()) m.sublabel = ld.sublabel;
        if (!ld.mfi.empty()) m.mfi = ld.mfi;
        if (!ld.msg_num.empty()) m.msg_num = ld.msg_num;
        if (m.reg.empty() && !ld.reg.empty()) {
            std::string r = ld.reg;
            while (!r.empty() && r[0] == '.') r.erase(0, 1);
            m.reg = trim_right(r);
        }
        if (m.flight.empty() && !ld.flight.empty()) m.flight = trim_right(ld.flight);
        if (m.text.empty() && !ld.text.empty()) m.text = printable(ld.text);
        m.arinc622_text = ld.arinc622_text;
        m.arinc622_json = ld.arinc622_json;
        m.arinc622_kind = ld.kind;
        m.has_arinc622 = !ld.kind.empty();
    }

    acars_cnt_++;
    msgs_.push_back(m);
    if (msgs_.size() > 2000) msgs_.pop_front();
}

// ---- SBD preheader / NAL10 -------------------------------------------------
static uint16_t be16v(const std::vector<uint8_t> &v, size_t i) {
    return (i + 1 < v.size()) ? (uint16_t)((v[i] << 8) | v[i + 1]) : 0;
}
static uint32_t be32v(const std::vector<uint8_t> &v, size_t i) {
    if (i + 3 >= v.size()) return 0;
    return ((uint32_t)v[i] << 24) | ((uint32_t)v[i + 1] << 16) |
           ((uint32_t)v[i + 2] << 8) | v[i + 3];
}
static std::string bcd_digits(const std::vector<uint8_t> &v, size_t start, size_t count) {
    static const char *d = "0123456789abcdef";
    std::string s;
    for (size_t i = start; i < start + count && i < v.size(); i++) {
        s += d[v[i] & 0xf];
        s += d[v[i] >> 4];
    }
    return s;
}
static uint16_t sbd_crc16(const uint8_t *d, size_t n) {
    uint16_t crc = 0;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408) : (uint16_t)(crc >> 1);
    }
    return crc ^ 0xffff;
}
static void xyz_from5(const std::vector<uint8_t> &v, size_t start, int skip,
                      int &x, int &y, int &z, double &lat, double &lon, double &alt) {
    uint32_t val = 0;
    for (size_t i = 0; i < 5 && start + i < v.size(); i++) val = (val << 8) | v[start + i];
    int sb = 4 - skip;
    x = (val >> (24 + sb)) & 0xfff; if (x > 0x800) x -= 0x1000;
    y = (val >> (12 + sb)) & 0xfff; if (y > 0x800) y -= 0x1000;
    z = (val >> sb) & 0xfff;        if (z > 0x800) z -= 0x1000;
    lat = atan2((double)z, std::sqrt((double)x * x + (double)y * y)) * 180.0 / M_PI;
    lon = atan2((double)y, (double)x) * 180.0 / M_PI;
    alt = std::sqrt((double)x * x + (double)y * y + (double)z * z) * 4.0;
}
static std::string hexv(std::initializer_list<uint8_t> l) {
    return hexns(std::vector<uint8_t>(l));
}

static void decode_sbd_preheader(const std::vector<uint8_t> &h,
                                 const std::vector<uint8_t> &data, SbdMessage &m) {
    std::string typ = m.type.substr(0, 4);
    char buf[640];
    if (typ == "0600" && h.size() >= 29) {
        if (h[0] == 0x20) {
            std::string sc = hexv({h[1], h[2], h[3]});
            if (sc == "13f010") sc = "ISC";
            else if (sc == "07f010") sc = "TSC";
            int prot = h[4];
            std::string bcd = bcd_digits(h, 5, 8);
            std::string imei = bcd.size() > 1 ? bcd.substr(1) : bcd;
            unsigned momsn = be16v(h, 13);
            int msgct = h[15];
            int addlen = 0;
            std::string extra, session, flags;
            if (prot == 0) {
                extra = hexv({h[16], h[17]});
                addlen = h[18];
            } else if (prot == 1) {
                extra = hexv({h[16]});
                addlen = h[17];
                flags = hexv({h[18]});
            } else if (prot == 2) {
                int st = h[16];
                if (st == 0x00) session = "STD";
                else if (st == 0x0c) session = "EXT";
                else if (st == 0x1c) session = "EXA";
                else if (st == 0x3c) session = "REG";
                else if (st == 0x2c) session = "ARG";
                else if (st == 0x20) session = "DET";
                else session = "???";
                addlen = h[17];
                flags = hexv({h[18]});
            } else {
                addlen = h[17];
            }
            uint16_t pktcrc = (uint16_t)(h[19] | (h[20] << 8));
            std::vector<uint8_t> cv = data;
            cv.insert(cv.end(), h.begin() + 5, h.begin() + 13);
            cv.insert(cv.end(), h.begin() + 13, h.begin() + 15);
            cv.insert(cv.end(), h.begin() + 25, h.begin() + 29);
            uint16_t crcval = sbd_crc16(cv.data(), cv.size());
            m.has_crc = true;
            m.crc_ok = (crcval == pktcrc);
            int16_t doppler = (int16_t)be16v(h, 21);
            unsigned delay = be16v(h, 23);
            std::string ts = fmt_iritime(be32v(h, 25));
            snprintf(buf, sizeof(buf),
                     "SBD %s v%d imei:%s MOMSN=%u msgct:%d %s len=%d crc:%s[%04x] ds:%+d0Hz pd:%05uus t:%s",
                     sc.c_str(), prot, imei.c_str(), momsn, msgct,
                     session.empty() ? extra.c_str() : session.c_str(),
                     addlen, m.crc_ok ? "OK" : "no", pktcrc,
                     (int)doppler, delay, ts.c_str());
            m.info = buf;
            if (!flags.empty()) m.info += " flags:" + flags;
            return;
        }
        if (h[0] == 0x10 || h[0] == 0x40 || h[0] == 0x70) {
            const char *kind = h[0] == 0x10 ? "REG" : (h[0] == 0x40 ? "SMS" : "CAL");
            std::string tmsi = hexv({h[4], h[5], h[6], h[7]});
            std::string lac = hexv({h[8], h[9]});
            std::string sca = hexv({h[10], h[11]});
            uint32_t ver = be32v(h, 12);
            int x, y, z; double lat, lon, alt;
            xyz_from5(h, 16, 0, x, y, z, lat, lon, alt);
            int16_t doppler = (int16_t)be16v(h, 21);
            unsigned delay = be16v(h, 23);
            std::string ts = fmt_iritime(be32v(h, 25));
            snprintf(buf, sizeof(buf),
                     "%s tmsi:%s lac:%s sca:%s v:%u xyz=(%+05d,%+05d,%+05d) ds:%+d0Hz pd:%05uus t:%s",
                     kind, tmsi.c_str(), lac.c_str(), sca.c_str(), ver, x, y, z,
                     (int)doppler, delay, ts.c_str());
            m.info = buf;
            return;
        }
    } else if (typ == "7608" && !h.empty()) {
        if (h[0] == 0x26 && h.size() >= 7) {
            snprintf(buf, sizeof(buf), "DL MTMSN=%u msgct:%d backlog=%d mid=%02x:%02x",
                     be16v(h, 1), h[3], h[4], h[5], h[6]);
            m.info = buf;
            return;
        }
        if (h[0] == 0x20 && h.size() >= 5) {
            snprintf(buf, sizeof(buf), "DL MTMSN=%u msgct:%d backlog=%d",
                     be16v(h, 1), h[3], h[4]);
            m.info = buf;
            return;
        }
    } else if (!h.empty() && h[0] == 0x50) {
        snprintf(buf, sizeof(buf), "ACK MTMSN=%u", be16v(h, 1));
        m.info = buf;
        return;
    }
    if (!h.empty()) m.info = "preheader " + hexns(h);
}

static bool decode_nal10(const std::vector<uint8_t> &data, SbdMessage &m) {
    if (data.size() != 10) return false;
    auto get = [&](int start, int amount) -> uint32_t {
        uint32_t v = 0;
        for (int i = 0; i < amount; i++) {
            int idx = start + i;
            if (data[idx / 8] & (1 << (idx % 8))) v += (1u << i);
        }
        return v;
    };
    uint32_t lat_raw = get(4, 25);
    uint32_t lon_raw = get(29, 25);
    uint32_t seconds = get(55, 17);
    uint32_t pdop_i = get(72, 4);
    uint32_t bits = get(76, 4);
    if (lat_raw > 18000000 || lon_raw > 36000000 || seconds >= 86400 || pdop_i >= 8)
        return false;
    static const int pdop_tbl[8] = {1, 2, 5, 10, 20, 40, 70, 100};
    m.has_nal = true;
    m.nal_lat = ((double)lat_raw - 9000000.0) / 100000.0;
    m.nal_lon = ((double)lon_raw - 18000000.0) / 100000.0;
    m.nal_seconds = (int)seconds;
    m.nal_pdop = pdop_tbl[pdop_i];
    m.nal_fix = (bits & 1) != 0;
    m.nal_emergency = (bits & 2) != 0;
    m.nal_emergency_ack = (bits & 4) != 0;
    m.nal_motion = (bits & 8) != 0;
    char buf[192];
    snprintf(buf, sizeof(buf),
             "lat=%.6f lon=%.6f pdop=%d emerg:%d/%d motion:%d fix:%d t=%02d:%02d:%02d",
             m.nal_lat, m.nal_lon, m.nal_pdop, m.nal_emergency ? 1 : 0,
             m.nal_emergency_ack ? 1 : 0, m.nal_motion ? 1 : 0, m.nal_fix ? 1 : 0,
             (int)(seconds / 3600), (int)((seconds / 60) % 60), (int)(seconds % 60));
    m.nal_info = buf;
    return true;
}

// ---- SBD reassembly --------------------------------------------------------
void AcarsAssembler::record_sbd(const std::string &typ, const std::vector<uint8_t> &prehdr,
                                const std::vector<uint8_t> &data, double time_s, int ul,
                                double frequency) {
    SbdMessage m;
    m.timestamp_ns = (uint64_t)(time_s * 1e9);
    m.frequency = frequency;
    m.ul = ul;
    m.type = typ;
    m.length = (int)data.size();
    m.preheader_hex = hexns(prehdr);
    m.payload_hex = hexns(data);
    if (!data.empty()) {
        std::string raw((const char *)data.data(), data.size());
        m.text = printable(raw);
    }
    m.acars = (!data.empty() && data[0] == 0x01);
    decode_sbd_preheader(prehdr, data, m);
    decode_nal10(data, m);
    sbd_msgs_.push_back(std::move(m));
    if (sbd_msgs_.size() > 1000) sbd_msgs_.pop_front();
    sbd_msg_cnt_++;
}

void AcarsAssembler::emit_sbd_payload(const std::vector<uint8_t> &pin, double time_s, int ul,
                                      double frequency) {
    gsmtap_send(pin, ul, frequency);

    // Decode the reassembled Um/L3 payload as well.
    L3Message l3;
    if (decode_l3(pin, time_s, ul, frequency, l3)) {
        l3_.push_back(std::move(l3));
        if (l3_.size() > 4000) l3_.pop_front();
        l3_cnt_++;
    }

    // SMS over GSM 04.11 CP-DATA: protocol discriminator 0x09/0x89, CP-DATA 0x01.
    if (pin.size() >= 2 && (pin[0] == 0x09 || pin[0] == 0x89) && pin[1] == 0x01) {
        std::vector<uint8_t> tpdu;
        if (pin.size() >= 4 && pin[2] == 0x01 && pin[3] > 0 &&
            (size_t)pin[3] <= pin.size() - 4) {
            size_t len = pin[3];
            tpdu.assign(pin.begin() + 4, pin.begin() + 4 + len);
        } else {
            tpdu.assign(pin.begin() + 2, pin.end());
        }
        SmsMessage m;
        m.timestamp_ns = (uint64_t)(time_s * 1e9);
        m.frequency = frequency;
        m.ul = ul;
        if (decode_sms_tpdu(tpdu, m)) {
            sms_cnt_++;
            sms_.push_back(m);
            if (sms_.size() > 500) sms_.pop_front();
        }
        return;
    }

    if (pin.size() < 5) return;
    bool is76 = pin[0] == 0x76 && pin[1] != 5;
    bool is06 = pin[0] == 0x06 && pin[1] == 0;
    if (!is76 && !is06) return;

    auto at = [](const std::vector<uint8_t> &v, int i) -> int {
        return (i >= 0 && i < (int)v.size()) ? v[i] : -1;
    };

    sbd_cnt_++;
    char typ[8];
    snprintf(typ, sizeof(typ), "%02x%02x", pin[0], pin[1]);
    std::vector<uint8_t> data(pin.begin() + 2, pin.end());
    std::vector<uint8_t> prehdr;
    int msgcnt = -1, msgno = 0;

    if (strcmp(typ, "0600") == 0) {
        if (at(data, 0) != 0x20) return;
        if (data.size() < 29) return;
        prehdr.assign(data.begin(), data.begin() + 29);
        data.erase(data.begin(), data.begin() + 29);
        msgcnt = prehdr[15];
        msgno = (msgcnt == 0) ? 0 : 1;
    } else {
        if (strcmp(typ, "7608") == 0) {
            if (at(data, 0) == 0x26 && data.size() >= 7) {
                prehdr.assign(data.begin(), data.begin() + 7);
                data.erase(data.begin(), data.begin() + 7);
            } else if (at(data, 0) == 0x20 && data.size() >= 5) {
                prehdr.assign(data.begin(), data.begin() + 5);
                data.erase(data.begin(), data.begin() + 5);
            } else if (data.size() >= 7) {
                prehdr.assign(data.begin(), data.begin() + 7);
                data.erase(data.begin(), data.begin() + 7);
            } else return;
            msgcnt = at(prehdr, 3);
        }
        if (ul && data.size() >= 3 && (data[0] == 0x50 || data[0] == 0x51))
            data.erase(data.begin(), data.begin() + 3);
        if (data.empty()) {
            msgno = 0;
        } else if (data.size() > 3 && data[0] == 0x10) {
            std::vector<uint8_t> hdr(data.begin(), data.begin() + 3);
            data.erase(data.begin(), data.begin() + 3);
            msgno = hdr[2];
            int hl = hdr[1];
            if ((int)data.size() < hl) return; // too short
            else if ((int)data.size() > hl) data.resize(hl);
        } else {
            msgno = 0;
        }
    }

    // expire stale multi packets
    for (size_t i = 0; i < multi_.size();) {
        if (multi_[i].time + 5.0 < time_s) multi_.erase(multi_.begin() + i);
        else ++i;
    }

    if (msgno == 0) {
        record_sbd(typ, prehdr, data, time_s, ul, frequency);
        decode_acars(data, time_s, ul, prehdr, typ, frequency);
        return;
    } else if (msgcnt == 1 && msgno == 1) {
        record_sbd(typ, prehdr, data, time_s, ul, frequency);
        decode_acars(data, time_s, ul, prehdr, typ, frequency);
        return;
    } else if (msgcnt > 1) {
        multi_.push_back({msgno, msgcnt, ul, time_s, data, typ, prehdr});
        return;
    } else if (msgno > 1) {
        for (size_t i = multi_.size(); i-- > 0;) {
            MultiPkt &mp = multi_[i];
            if (msgno == mp.msgno + 1 && mp.ul == ul) {
                mp.data.insert(mp.data.end(), data.begin(), data.end());
                mp.typ += typ;
                if (msgno < mp.msgcnt) {
                    mp.msgno++;
                } else {
                    record_sbd(mp.typ, mp.prehdr, mp.data, time_s, ul, frequency);
                    decode_acars(mp.data, time_s, ul, mp.prehdr, mp.typ, frequency);
                    multi_.erase(multi_.begin() + i);
                }
                return;
            }
        }
        return; // orphan
    }
}

// ---- IDA reassembly --------------------------------------------------------
void AcarsAssembler::feed(const irid::ParsedMessage &pm) {
    if (!pm.has_ida || !pm.ida_crc_ok || pm.ida_data.empty()) return;

    int nbytes = pm.ida_len > 0 ? std::min<int>(pm.ida_len, (int)pm.ida_data.size())
                                : (int)pm.ida_data.size();
    if (nbytes <= 0) return;
    std::vector<uint8_t> frag(pm.ida_data.begin(), pm.ida_data.begin() + nbytes);

    double t = pm.timestamp_ns / 1e9;
    double freq = pm.frequency;
    int ul = pm.uplink;
    int ctr = pm.ida_ctr;
    bool cont = pm.ida_cont != 0;

    std::lock_guard<std::mutex> lk(mtx_);

    // rudimentary de-dupe
    if ((otime_ - 1) <= t && t <= (otime_ + 1) && odata_ == frag &&
        (ofreq_ - 200) < freq && freq < (ofreq_ + 200))
        return;
    otime_ = t; odata_ = frag; ofreq_ = freq;

    for (size_t i = 0; i < ida_buf_.size(); i++) {
        SbdFrag &e = ida_buf_[i];
        if ((e.freq - 260) < freq && freq < (e.freq + 260) &&
            e.times.back() <= t && t <= (e.times.back() + 0.280) &&
            ((e.ctr + 1) % 8) == ctr && e.ul == ul) {
            std::vector<uint8_t> payload = e.data;
            payload.insert(payload.end(), frag.begin(), frag.end());
            e.times.push_back(t);
            e.ctr = ctr;
            if (cont) {
                e.data = payload;
                e.cont = true;
                ida_frag_++;
            } else {
                ida_ok_++;
                emit_sbd_payload(payload, t, ul, freq);
                ida_buf_.erase(ida_buf_.begin() + i);
            }
            // expire
            for (size_t j = 0; j < ida_buf_.size();) {
                if (ida_buf_[j].times.back() + 1.0 <= t) { ida_broken_++; ida_buf_.erase(ida_buf_.begin() + j); }
                else ++j;
            }
            return;
        }
    }

    if (ctr == 0 && !cont) {
        ida_ok_++;
        emit_sbd_payload(frag, t, ul, freq);
    } else if (ctr == 0 && cont) {
        ida_frag_++;
        ida_buf_.push_back({freq, {t}, ctr, frag, cont, ul});
    } else {
        ida_broken_++;
    }

    for (size_t j = 0; j < ida_buf_.size();) {
        if (ida_buf_[j].times.back() + 1.0 <= t) { ida_broken_++; ida_buf_.erase(ida_buf_.begin() + j); }
        else ++j;
    }
}

} // namespace iridi
