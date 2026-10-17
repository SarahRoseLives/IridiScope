#include "messages/acars.h"
#include "parser/fec.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

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

void AcarsAssembler::reset() {
    std::lock_guard<std::mutex> lk(mtx_);
    ida_buf_.clear();
    multi_.clear();
    msgs_.clear();
    otime_ = -1e9; odata_.clear(); ofreq_ = 0;
    ida_ok_ = ida_frag_ = ida_broken_ = 0;
    sbd_cnt_ = acars_cnt_ = 0;
}

std::vector<AcarsMessage> AcarsAssembler::messages() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return std::vector<AcarsMessage>(msgs_.begin(), msgs_.end());
}

// ---- ACARS decode ----------------------------------------------------------
void AcarsAssembler::decode_acars(const std::vector<uint8_t> &in, double time_s,
                                  int ul, const std::vector<uint8_t> &prehdr,
                                  const std::string &typ) {
    if (in.empty() || in[0] != 1) return;
    if (in.size() <= 2) return;

    AcarsMessage m;
    m.timestamp_ns = (uint64_t)(time_s * 1e9);
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

    acars_cnt_++;
    msgs_.push_back(m);
    if (msgs_.size() > 2000) msgs_.pop_front();
}

// ---- SBD reassembly --------------------------------------------------------
void AcarsAssembler::emit_sbd_payload(const std::vector<uint8_t> &pin, double time_s, int ul) {
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
        decode_acars(data, time_s, ul, prehdr, typ);
        return;
    } else if (msgcnt == 1 && msgno == 1) {
        decode_acars(data, time_s, ul, prehdr, typ);
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
                    decode_acars(mp.data, time_s, ul, mp.prehdr, mp.typ);
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
                emit_sbd_payload(payload, t, ul);
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
        emit_sbd_payload(frag, t, ul);
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
