#include "messages/l3.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iridi {
namespace {

using Bytes = std::vector<uint8_t>;

std::string to_hex(const Bytes &d, char sep = ':') {
    static const char *h = "0123456789abcdef";
    std::string s;
    for (size_t i = 0; i < d.size(); i++) {
        if (i && sep) s += sep;
        s += h[d[i] >> 4];
        s += h[d[i] & 0xf];
    }
    return s;
}

std::string ascii(const Bytes &d) {
    std::string s;
    for (uint8_t c : d) s += (c >= 32 && c < 127) ? (char)c : '.';
    return s;
}

std::string bits_str(uint32_t v, int n) {
    std::string s((size_t)n, '0');
    for (int i = 0; i < n; i++) s[n - 1 - i] = ((v >> i) & 1) ? '1' : '0';
    return s;
}

std::string bcd_num(const Bytes &d) {
    if (d.empty()) return "PARSE_FAIL";
    int ext = (d[0] >> 7) & 1;
    int numtype = (d[0] >> 4) & 7;
    int numplan = d[0] & 0xf;
    std::string txt;
    const uint8_t *num = nullptr;
    size_t n = 0;
    if (ext == 0) {
        if (d.size() < 2) return "PARSE_FAIL";
        int pi = (d[1] >> 5) & 3;
        if (pi == 2) {
            txt += "Number_not_available ";
            if (numplan == 0) numplan = 1;
        } else if (pi != 0) {
            char b[32]; snprintf(b, sizeof(b), "present=%d ", pi); txt += b;
        }
        num = d.data() + 2;
        n = d.size() - 2;
    } else {
        num = d.data() + 1;
        n = d.size() - 1;
    }
    static const char *h = "0123456789abcdef";
    std::string digits;
    for (size_t i = 0; i < n; i++) {
        digits += h[num[i] & 0xf];
        digits += h[num[i] >> 4];
    }
    if (!digits.empty() && digits.back() == 'f') digits.pop_back();
    for (char &c : digits) {
        if (c == 'a') c = '*';
        else if (c == 'b') c = '#';
        else if (c == 'c') c = 'a';
        else if (c == 'd') c = 'b';
        else if (c == 'e') c = 'c';
    }
    if (numtype == 1) txt += "+";
    else if (numtype != 0) {
        char b[32]; snprintf(b, sizeof(b), "type:%d ", numtype); txt += b;
    }
    if (numplan != 1) {
        char b[32]; snprintf(b, sizeof(b), "plan:%d ", numplan); txt += b;
    }
    txt += digits;
    return txt;
}

std::string lai_str(const Bytes &d) {
    if (d.size() < 5) return "PARSE_FAIL";
    char b[160];
    if ((d[1] >> 4) == 0xf)
        snprintf(b, sizeof(b), "MCC=%x%x%x/MNC=%x%x/LAC=%02x%02x",
                 d[0] & 0xf, d[0] >> 4, d[1] & 0xf, d[2] & 0xf, d[2] >> 4, d[3], d[4]);
    else
        snprintf(b, sizeof(b), "MCC=%x%x%x/MNC=%x%x%x/LAC=%02x%02x",
                 d[0] & 0xf, d[0] >> 4, d[1] & 0xf, d[2] & 0xf, d[2] >> 4,
                 d[1] >> 4, d[3], d[4]);
    return b;
}

std::string mi_str(const Bytes &d) {
    if (d.empty()) return "PARSE_FAIL";
    int dig = (d[0] >> 4) & 0xf;
    int odd = (d[0] >> 3) & 1;
    int typ = d[0] & 7;
    if (typ == 2 || typ == 1) { // IMEI / IMSI
        if (odd != 1) return "ERR_MI_PARSE_FAIL[" + to_hex(d) + "]";
        std::string s;
        char b[8]; snprintf(b, sizeof(b), "%x", dig); s += b;
        for (size_t i = 1; i < d.size(); i++) {
            s += "0123456789abcdef"[d[i] & 0xf];
            s += "0123456789abcdef"[d[i] >> 4];
        }
        return (typ == 2 ? "imei:" : "imsi:") + s;
    }
    if (typ == 4) { // TMSI
        if (odd == 0 && dig == 0xf)
            return "tmsi:" + to_hex(Bytes(d.begin() + 1, d.end()), 0);
    }
    return "ERR_PARSE_FAIL[" + to_hex(d) + "]";
}

std::string disc_str(const Bytes &d) {
    if (d.empty()) return "ERR:PARSE_FAIL";
    int ext = (d[0] >> 7) & 1;
    int coding = (d[0] >> 5) & 3;
    int loc = d[0] & 0xf;
    if (coding != 3 || ext != 1) return "ERR:PARSE_FAIL";
    std::string s;
    switch (loc) {
    case 0: s = "Loc:user "; break;
    case 2: s = "Net:local"; break;
    case 3: s = "Net:trans"; break;
    case 4: s = "Net:remot"; break;
    default: { char b[32]; snprintf(b, sizeof(b), "Loc:%3d ", loc); s = b; break; }
    }
    if (d.size() < 2) return "ERR:SHORT";
    int cause = d[1] & 0x7f;
    char b[64];
    switch (cause) {
    case 1: snprintf(b, sizeof(b), " Cause(01) Unassigned number"); break;
    case 16: snprintf(b, sizeof(b), " Cause(16) Normal call clearing"); break;
    case 17: snprintf(b, sizeof(b), " Cause(17) User busy"); break;
    case 21: snprintf(b, sizeof(b), " Cause(21) Rejected"); break;
    case 31: snprintf(b, sizeof(b), " Cause(31) Normal, unspecified"); break;
    case 34: snprintf(b, sizeof(b), " Cause(34) No channel available"); break;
    case 41: snprintf(b, sizeof(b), " Cause(41) Temporary failure"); break;
    case 57: snprintf(b, sizeof(b), " Cause(57) Bearer cap. not authorized"); break;
    case 127: snprintf(b, sizeof(b), " Cause(127) Interworking, unspecified"); break;
    default: snprintf(b, sizeof(b), " Cause: %d", cause); break;
    }
    s += b;
    if (d.size() > 2) s += " ERR:ADD_" + to_hex(Bytes(d.begin() + 2, d.end()));
    return s;
}

std::string bearer_str(const Bytes &d) {
    if (d.empty()) return "PARSE_FAIL";
    int rcr = (d[0] >> 5) & 3;
    int coding = (d[0] >> 4) & 1;
    int trans_mode = (d[0] >> 3) & 1;
    int trans_capa = d[0] & 7;
    static const char *rates[4] = {"rate:reserved", "full rate only",
                                   "half rate pref.", "full rate pref."};
    static const char *caps[4] = {"speech", "digital", "3.1 kHz audio", "fax"};
    std::string txt = rates[rcr];
    if (coding != 0) txt += ",coding:reserved";
    if (trans_mode != 0) txt += ",mode:packet";
    txt += ",";
    if (trans_capa < 4) txt += caps[trans_capa];
    else { char b[32]; snprintf(b, sizeof(b), "transfer:%x", trans_capa); txt += b; }
    if (d.size() > 1) txt += ":" + to_hex(Bytes(d.begin() + 1, d.end()));
    return txt;
}

std::string progress_str(const Bytes &d) {
    if (d.size() != 2) return "PARSE_FAIL[" + to_hex(d) + "]";
    int cs = (d[0] >> 5) & 3;
    int loc = d[0] & 0xf;
    int p = d[1] & 0x7f;
    std::string txt;
    char b[64];
    if (cs != 3) { snprintf(b, sizeof(b), "cs:%d ", cs); txt += b; }
    if (loc != 2) { snprintf(b, sizeof(b), "loc:%x", loc); txt += b; }
    switch (p) {
    case 3: txt += "Origination address is non-local"; break;
    case 2: txt += "Destination address is non-local"; break;
    case 8: txt += "In-band available"; break;
    case 32: txt += "Call is local"; break;
    default: snprintf(b, sizeof(b), "progress:%02x", p); txt += b; break;
    }
    return txt;
}

std::string facility_str(const Bytes &d) {
    std::string h = to_hex(d);
    if (h == "a1:0e:02:01:01:02:01:10:30:06:81:01:28:84:01:07")
        return "allCondForwardingSS(Provisioned+Registered+Active)";
    if (h == "a1:0e:02:01:01:02:01:10:30:06:81:01:21:84:01:07")
        return "cfu(Provisioned+Registered+Active)";
    return "facility[" + h + "]";
}

struct Xyz { int x = 0, y = 0, z = 0; double lat = 0, lon = 0, alt = 0; };

Xyz xyz_from(const Bytes &v, size_t start, int skip) {
    Xyz r;
    uint32_t val = 0;
    for (size_t i = 0; i < 5 && start + i < v.size(); i++)
        val = (val << 8) | v[start + i];
    int sb = 4 - skip;
    r.x = (val >> (24 + sb)) & 0xfff; if (r.x > 0x800) r.x -= 0x1000;
    r.y = (val >> (12 + sb)) & 0xfff; if (r.y > 0x800) r.y -= 0x1000;
    r.z = (val >> sb) & 0xfff;        if (r.z > 0x800) r.z -= 0x1000;
    r.lat = atan2((double)r.z, std::sqrt((double)r.x * r.x + (double)r.y * r.y)) * 180.0 / M_PI;
    r.lon = atan2((double)r.y, (double)r.x) * 180.0 / M_PI;
    r.alt = std::sqrt((double)r.x * r.x + (double)r.y * r.y + (double)r.z * r.z) * 4.0;
    return r;
}

std::string p_tv(int iei, const Bytes *data) {
    char b[256];
    auto first = [&]() -> int { return (data && !data->empty()) ? (*data)[0] : 0; };
    if (iei == 0x201) { // AccessDisposition + AccessDetailCode
        int ad = first() >> 4, adc = first() & 0xf;
        std::string rv = ad == 0 ? "R:ok   " : "R:NOT[" + to_hex(Bytes{ (uint8_t)ad }) + "]";
        if (adc == 2) rv += " (Location Update was not accepted)";
        else if (adc != 0) { snprintf(b, sizeof(b), " (unknown:%x)", adc); rv += b; }
        return rv;
    }
    if (iei == 0x140) {
        Xyz p = xyz_from(*data, 0, 4);
        snprintf(b, sizeof(b), "xyz=(%+05d,%+05d,%+05d)", p.x, p.y, p.z);
        return b;
    }
    if (iei == 0x150) {
        if ((first() & 0xf) == 0 && data->size() >= 3)
            snprintf(b, sizeof(b), "crc:%02x%02x", (*data)[2], (*data)[1]);
        else if (data->size() >= 3)
            snprintf(b, sizeof(b), "crc[%d]:%02x%02x", first() & 0xf, (*data)[2], (*data)[1]);
        else return "crc:short";
        return b;
    }
    if (iei == 0x1d0) return "unlock:" + (data ? to_hex(*data) : "");
    if (iei == 0x202) {
        snprintf(b, sizeof(b), "AD:%s ADC:%s",
                 bits_str(first() >> 4, 4).c_str(), bits_str(first() & 0xf, 4).c_str());
        return b;
    }
    if (iei == 0x203) {
        if (data->size() >= 3)
            snprintf(b, sizeof(b), "sat:%03d beam:%02d %x %s",
                     (*data)[0], (*data)[1], (*data)[2],
                     to_hex(Bytes(data->begin() + 3, data->end())).c_str());
        else return "sat:short";
        return b;
    }
    if (iei == 0x1e0) { snprintf(b, sizeof(b), "mre:%x", first() & 0xf); return b; }
    if (iei == 0x1a0) { snprintf(b, sizeof(b), "tss:%x", first() & 0xf); return b; }
    if (iei == 0x104) return "lac:" + to_hex(*data);
    if (iei == 0x161) return "sca:" + to_hex(*data);
    if (iei == 0x119) return "rrd:" + to_hex(*data);
    if (iei == 0x11a) return "rre:" + to_hex(*data);
    if (iei == 0x118) return "aep:" + to_hex(*data);
    if (iei == 0x11b) {
        Xyz p = xyz_from(*data, 0, 0);
        if (data->size() > 4 && ((*data)[4] & 0x4) == 0)
            snprintf(b, sizeof(b), "xyz=(%+05d,%+05d,%+05d)", p.x, p.y, p.z);
        else if (data->size() > 4)
            snprintf(b, sizeof(b), "xyz=(%+05d,%+05d,%+05d) [%d]",
                     p.x, p.y, p.z, (*data)[4] & 0xf);
        else return "xyz:short";
        return b;
    }
    if (iei == 0xd0) { snprintf(b, sizeof(b), "repeat indi.: %d", first()); return b; }
    if (iei == 0x80) {
        static const char *prio[8] = {"none", "4", "3", "2", "1", "0", "B", "A"};
        return std::string("prio:") + prio[first() & 7];
    }
    if (iei == 0xa1) return "Follow-on Proceed";
    if (iei == 0xa2) return "CTS permission";
    if (iei == 0x34) { snprintf(b, sizeof(b), "signal:%02x", first()); return b; }
    if (iei == 0x5c) return "src:[" + bcd_num(*data) + "]";
    if (iei == 0x5e) return "dest:[" + bcd_num(*data) + "]";
    if (iei == 0x1e) return "Progress[" + progress_str(*data) + "]";
    if (iei == 0x04) return "BC[" + bearer_str(*data) + "]";
    if (iei == 0x4c) return "num:[" + bcd_num(*data) + "]";
    if (iei == 0x08) return disc_str(*data);
    if (iei == 0x1c) return facility_str(*data);
    if (iei == 0x17) return mi_str(*data);
    if (iei == 0x13) return lai_str(*data);
    if (iei == 0x41) {
        if (first() == 0x28) return "";
        snprintf(b, sizeof(b), "CM1[%02x]", first());
        return b;
    }
    if (iei == 0x5d || iei == 0x6d || iei == 0x74 || iei == 0x75 ||
        iei == 0x7c || iei == 0x7d || iei == 0x7e || iei == 0x19 ||
        iei == 0x4d || iei == 0x15) {
        snprintf(b, sizeof(b), "%02x:[%s]", iei, to_hex(*data).c_str());
        return b;
    }
    if (!data) { snprintf(b, sizeof(b), "ERR:UK_T:%02x", iei); return b; }
    snprintf(b, sizeof(b), "ERR:UK_TV:%02x:[%s]", iei, to_hex(*data).c_str());
    return b;
}

struct TlvSpec {
    std::vector<std::pair<int, int>> type1, type2, type3, type4, start, type1i, type3i;
};

std::string p_opt_tlv(const Bytes &in, const TlvSpec &s, Bytes &rest) {
    Bytes d = in;
    std::string out;
    auto contains = [](const std::vector<std::pair<int, int>> &v, int k, int *len) {
        for (auto &p : v) if (p.first == k) { if (len) *len = p.second; return true; }
        return false;
    };
    auto append = [&](const std::string &x) {
        if (!x.empty()) { if (!out.empty()) out += " "; out += x; }
    };

    for (auto &sp : s.start) {
        int len = sp.second;
        if (len < 0) {
            if (d.empty()) { append("ERR:SHORT"); break; }
            len = d[0];
            d.erase(d.begin());
        }
        if ((int)d.size() < len) { append("ERR:SHORT:" + to_hex(d)); break; }
        Bytes v(d.begin(), d.begin() + len);
        d.erase(d.begin(), d.begin() + len);
        append(p_tv(sp.first, &v));
    }

    while (!d.empty()) {
        int iei = d[0] & 0xf0;
        int len = 0;
        if (contains(s.type1, iei, nullptr)) {
            Bytes v{ (uint8_t)(d[0] & 0x0f) };
            append(p_tv(iei, &v));
            d.erase(d.begin());
            continue;
        }
        if (contains(s.type1i, iei, &len)) {
            if ((int)d.size() < 1 + len) { append("ERR:SHORT(1i):" + to_hex(d)); break; }
            Bytes v(d.begin() + 1, d.begin() + 1 + len);
            append(p_tv(iei + 0x100, &v));
            d.erase(d.begin(), d.begin() + 1 + len);
            continue;
        }
        if (contains(s.type2, d[0], nullptr)) {
            append(p_tv(d[0], nullptr));
            d.erase(d.begin());
            continue;
        }
        if (contains(s.type3, d[0], &len)) {
            if ((int)d.size() < 1 + len) { append("ERR:SHORT:" + to_hex(d)); break; }
            Bytes v(d.begin() + 1, d.begin() + 1 + len);
            append(p_tv(d[0], &v));
            d.erase(d.begin(), d.begin() + 1 + len);
            continue;
        }
        if (contains(s.type3i, d[0], &len)) {
            if ((int)d.size() < 1 + len) { append("ERR:SHORT:" + to_hex(d)); break; }
            Bytes v(d.begin() + 1, d.begin() + 1 + len);
            append(p_tv(d[0] + 0x100, &v));
            d.erase(d.begin(), d.begin() + 1 + len);
            continue;
        }
        if (contains(s.type4, d[0], nullptr)) {
            if (d.size() < 2) { append("ERR:SHORT:" + to_hex(d)); break; }
            int l = d[1];
            if ((int)d.size() < 2 + l) { append("ERR:SHORT:" + to_hex(d)); break; }
            Bytes v(d.begin() + 2, d.begin() + 2 + l);
            append(p_tv(d[0], &v));
            d.erase(d.begin(), d.begin() + 2 + l);
            continue;
        }
        break;
    }
    rest = d;
    return out;
}

std::string p_l3_body(const Bytes &body, const std::string &typ) {
    Bytes rest;
    TlvSpec s;
    char b[128];

    if (typ == "7605") {
        s.start = {{0x201, 1}};
        s.type1i = {{0x40, 4}, {0x50, 2}, {0xd0, 9}};
        return p_opt_tlv(body, s, rest);
    }
    if (typ == "0605") {
        s.start = {{0x202, 1}, {0x203, 20}};
        s.type1i = {{0xe0, 0}, {0xa0, 0}};
        s.type3i = {{0x04, 2}, {0x61, 2}, {0x19, 1}, {0x1a, 1},
                    {0x18, 1}, {0x1b, 5}, {0xa0, 1}};
        return p_opt_tlv(body, s, rest);
    }
    if (typ == "0305") {
        s.type4 = {{0x04, 0}, {0x1c, 0}, {0x1e, 0}, {0x34, 0}, {0x5c, 0},
                   {0x5d, 0}, {0x5e, 0}, {0x6d, 0}, {0x74, 0}, {0x75, 0},
                   {0x7c, 0}, {0x7d, 0}, {0x7e, 0}, {0x19, 0}};
        s.type1 = {{0xd0, 0}, {0x80, 0}};
        s.type3 = {{0x34, 1}};
        return p_opt_tlv(body, s, rest);
    }
    if (typ == "0301") {
        s.type4 = {{0x1c, 0}, {0x1e, 0}, {0x7e, 0}};
        return p_opt_tlv(body, s, rest);
    }
    if (typ == "0307") {
        s.type4 = {{0x1c, 0}, {0x1e, 0}, {0x4c, 0}, {0x4d, 0}, {0x7e, 0}};
        return p_opt_tlv(body, s, rest);
    }
    if (typ == "0308") {
        s.type4 = {{0x04, 0}, {0x08, 0}, {0x15, 0}};
        s.type1 = {{0xd0, 0}};
        return p_opt_tlv(body, s, rest);
    }
    if (typ == "032d" || typ == "032a") {
        s.type4 = {{0x08, 0}, {0x1c, 0}, {0x7e, 0}};
        return p_opt_tlv(body, s, rest);
    }
    if (typ == "0325") {
        Bytes d{0x08};
        d.insert(d.end(), body.begin(), body.end());
        s.type4 = {{0x08, 0}, {0x1c, 0}, {0x1e, 0}, {0x7e, 0}, {0x7b, 0}};
        return p_opt_tlv(d, s, rest);
    }
    if (typ == "0302") {
        s.type4 = {{0x04, 0}, {0x1c, 0}, {0x1e, 0}};
        s.type1 = {{0xd0, 0}, {0x80, 0}};
        return p_opt_tlv(body, s, rest);
    }
    if (typ == "0303") {
        Bytes d{0x1e};
        d.insert(d.end(), body.begin(), body.end());
        s.type4 = {{0x1e, 0}, {0x7e, 0}};
        return p_opt_tlv(d, s, rest);
    }
    if (typ == "0502") {
        s.type4 = {{0x17, 0}};
        s.type2 = {{0xa1, 0}, {0xa2, 0}};
        s.start = {{0x13, 5}};
        return p_opt_tlv(body, s, rest);
    }
    if (typ == "0508") {
        if (body.empty()) return "";
        std::string out;
        int key = body[0] >> 4;
        snprintf(b, sizeof(b), "key=%s ", key == 7 ? "none" : std::to_string(key).c_str());
        out = b;
        s.start = {{0x13, 5}, {0x41, 1}, {0x17, -1}};
        out += p_opt_tlv(Bytes(body.begin() + 1, body.end()), s, rest);
        return out;
    }
    if (typ == "051a") {
        s.start = {{0x13, 5}, {0x17, -1}};
        return p_opt_tlv(body, s, rest);
    }
    if (typ == "0504") {
        if (!body.empty() && body[0] == 2) return "02(IMSI unknown in HLR)";
        return to_hex(body);
    }
    if (typ == "0518") {
        if (!body.empty() && body[0] == 2) return "02(IMEI)";
        if (!body.empty() && body[0] == 1) return "01(IMSI)";
        return to_hex(body);
    }
    if (typ == "0519") {
        s.start = {{0x17, -1}};
        return p_opt_tlv(body, s, rest);
    }
    if (typ == "0524") {
        if (body.empty()) return "";
        std::string out;
        int key = body[0] >> 4;
        snprintf(b, sizeof(b), "key=%s ", key == 7 ? "none" : std::to_string(key).c_str());
        out = b;
        int svc = body[0] & 0xf;
        if (svc == 4) out += "[SMS] ";
        else if (svc == 1) out += "[MO call/pkt mode] ";
        else { snprintf(b, sizeof(b), "[SVC:%d] ", svc); out += b; }
        Bytes rest2(body.begin() + 1, body.end());
        if (!rest2.empty()) {
            int cmlen = rest2[0];
            if (cmlen > 0 && (int)rest2.size() >= 1 + cmlen) {
                out += "CM2:" + to_hex(Bytes(rest2.begin() + 1, rest2.begin() + 1 + cmlen)) + " ";
                rest2.erase(rest2.begin(), rest2.begin() + 1 + cmlen);
            }
        }
        s.type1 = {{0x80, 0}};
        s.start = {{0x17, -1}};
        out += p_opt_tlv(rest2, s, rest);
        return out;
    }
    if (typ == "0901")
        return "CP-DATA " + to_hex(body);
    if (typ == "0904")
        return "CP-ACK";
    if (typ == "0910")
        return "CP-ERROR " + to_hex(body);
    if (typ == "0600")
        return "access/registration " + to_hex(body);
    if (typ == "7608" || typ == "7609" || typ == "760a" ||
        typ == "760c" || typ == "760d" || typ == "760e")
        return "SBD " + to_hex(body);
    return to_hex(body);
}

} // namespace

bool decode_l3(const std::vector<uint8_t> &data, double time_s, int ul, double freq,
               L3Message &out) {
    if (data.size() < 2) return false;
    int tmaj = data[0];
    int tmin = (data[0] << 8) | data[1];
    int typ = tmin;
    if (tmaj == 0x83 || tmaj == 0x89)
        typ = ((data[0] & 0x7f) << 8) | data[1];

    if ((typ == 0x063a || typ == 0x083a || typ == 0x7608 ||
         typ == 0x7609 || typ == 0x760c) && data.size() <= 2)
        return false;

    static const std::map<int, const char *> majmap = {
        {0x03, "CC"}, {0x83, "CC(dest)"}, {0x05, "MM"}, {0x06, "06"},
        {0x08, "08"}, {0x09, "SMS"}, {0x89, "SMS(dest)"}, {0x76, "SBD"},
    };
    static const std::map<int, const char *> minmap = {
        {0x0301, "Alerting"}, {0x0302, "Call Proceeding"}, {0x0303, "Progress"},
        {0x0305, "Setup"}, {0x0307, "Connect"}, {0x0308, "Call Confirmed"},
        {0x030f, "Connect Acknowledge"}, {0x0325, "Disconnect"},
        {0x032a, "Release Complete"}, {0x032d, "Release"},
        {0x0502, "Location Updating Accept"}, {0x0504, "Location Updating Reject"},
        {0x0508, "Location Updating Request"}, {0x0512, "Authentication Request"},
        {0x0514, "Authentication Response"}, {0x0518, "Identity request"},
        {0x0519, "Identity response"}, {0x051a, "TMSI Reallocation Command"},
        {0x051b, "TMSI Reallocation Complete"}, {0x0521, "CM Service Accept"},
        {0x0524, "CM Service Request"}, {0x0901, "CP-DATA"}, {0x0904, "CP-ACK"},
        {0x0910, "CP-ERROR"}, {0x0600, "Access Request Message"},
        {0x0605, "Access decision"}, {0x7605, "Access decision"},
        {0x7608, "downlink #1"}, {0x7609, "downlink #2"}, {0x760a, "downlink #3+"},
        {0x760c, "uplink initial"}, {0x760d, "uplink #2"}, {0x760e, "uplink #3"},
    };

    out = L3Message{};
    out.timestamp_ns = (uint64_t)(time_s * 1e9);
    out.frequency = freq;
    out.ul = ul;

    auto mi = majmap.find(tmaj);
    out.proto = (mi != majmap.end()) ? mi->second : "?";
    auto ti = minmap.find(typ);
    out.type = (ti != minmap.end()) ? ti->second : "?";

    Bytes body(data.begin() + 2, data.end());
    char tkey[8];
    snprintf(tkey, sizeof(tkey), "%04x", typ);
    out.summary = p_l3_body(body, tkey);
    if (out.summary.empty()) out.summary = to_hex(body);
    out.raw_hex = to_hex(data, ' ');
    out.ok = true;
    return true;
}

} // namespace iridi
