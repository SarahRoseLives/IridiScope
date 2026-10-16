#include "parser/iridium_parser.h"
#include "parser/fec.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace irid {

int irid_debug = 0; // set via --frame to trace classification

// ---- constants (bitsparser.py) --------------------------------------------
static const Bits iridium_access = "001100000011000011110011";
static const Bits uplink_access  = "110011000011110011111100";
static const Bits next_access_dl = "110011110011111111111100";
static const Bits next_access_ul = "001111000000000011111111";
static const Bits header_messaging = "00110011111100110011001111110011";
static const Bits header_time_location = "11" + Bits(94, '0');
static const int messaging_bch_poly  = 1897;
static const int ringalert_bch_poly  = 1207;
static const int acch_bch_poly       = 3545;
static const int hdr_poly            = 29;

static double threshold_simplex() {
    return (1626104e3 - 36000.0 - 1000.0) * (1.0 - 100e-6);
}
static double threshold_duplex() {
    return (1625979e3 + 36000.0 + 1000.0) * (1.0 + 100e-6);
}

// Python-<str>[pos:]/[pos:pos+n] clamp instead of throwing.
static Bits safe_sub(const Bits &s, size_t pos) {
    return pos >= s.size() ? Bits() : s.substr(pos);
}
static Bits safe_sub(const Bits &s, size_t pos, size_t n) {
    if (pos >= s.size()) return Bits();
    return s.substr(pos, std::min(n, s.size() - pos));
}

static int popcount_bits(const Bits &s) {
    int c = 0; for (char ch : s) if (ch == '1') c++; return c;
}

static std::string to_hex(const Bits &bits) {
    // ".".join("%02x" % int("0"+x,2) for x in slice(bits,8))
    std::string out;
    auto chunks = slice(bits, 8);
    for (size_t i = 0; i < chunks.size(); i++) {
        int v = bits_to_int(chunks[i]);
        char b[4]; snprintf(b, sizeof(b), "%02x", v & 0xff);
        if (i) out += '.';
        out += b;
    }
    return out;
}

static std::string to_ascii_dots(const std::vector<uint8_t> &data) {
    std::string s;
    for (uint8_t c : data) s += (c >= 32 && c < 127) ? (char)c : '.';
    return s;
}

// ---- frame working state ---------------------------------------------------
namespace {
struct Frame {
    // metadata
    double frequency = 0;
    uint64_t timestamp_ns = 0;
    int confidence = 0;
    float level = 0, noise = 0, snr = 0;
    int symbols = 0;

    std::string bitstream_raw;
    int uplink = 0;
    bool next = false;
    std::string msgtype;
    std::string header;
    Bits descrambled_bits;              // concatenated for MS/AQ/TL
    std::vector<Bits> descrambled;      // blocks (RA/BC/DA)
    Bits descramble_extra;
    int fill = 0;
    int fixederrs = 0;
    int ec_lcw = -1;

    // LCW
    int ft = 0;
    Bits lcw1, lcw2, lcw3;
    std::string lcw_header;
    bool lcw_ok = false;
};
} // namespace

// ---- ECC (RA/BC/MS) --------------------------------------------------------
static bool ecc_decode(Frame &f, int poly) {
    // remove FILL pattern for MS/RA
    if (f.msgtype == "MS" || f.msgtype == "RA") {
        const Bits pat1 = "10100010011100111011111101101101";
        const Bits pat2 = "01010100010001011100001011100110";
        while (f.descrambled.size() >= 2) {
            const Bits &a = f.descrambled[f.descrambled.size() - 2];
            const Bits &b = f.descrambled[f.descrambled.size() - 1];
            if ((a + b) == (pat1 + pat2) ||
                (bitdiff(a, pat1) <= 2 && bitdiff(b, pat2) <= 2)) {
                f.fill++;
                f.descrambled.pop_back();
                f.descrambled.pop_back();
            } else break;
        }
        if (f.fill > 0) f.descramble_extra.clear();
    }

    Bits bitstream_bch;
    for (const Bits &block : f.descrambled) {
        if (block.size() != 32) {
            if (irid_debug) fprintf(stderr, "[ecc] bad block len=%zu\n", block.size());
            return false;
        }
        Bits parity = block.substr(31, 1);
        Bits b31 = block.substr(0, 31);
        Bits data, bch;
        int errs = bch_repair(poly, b31, data, bch);
        if (irid_debug)
            fprintf(stderr, "[ecc] poly=%d ndiv=%d errs=%d\n", poly,
                    ndivide(poly, b31), errs);
        if (errs < 0) { f.descramble_extra.clear(); return false; }
        if (((popcount_bits(data + bch + parity)) % 2) == 1) {
            if (errs > 0) { f.descramble_extra.clear(); return false; }
        }
        if (errs > 0) f.fixederrs++;
        bitstream_bch += data;
    }
    f.descrambled.clear();
    f.descrambled.push_back(bitstream_bch);
    return !bitstream_bch.empty();
}

static bool lcw_ecc_decode(Frame &f, int poly) {
    Bits bitstream_bch;
    for (const Bits &block : f.descrambled) {
        if (block.size() != 31) return false;
        Bits data, bch;
        int errs = bch_repair(poly, block, data, bch);
        if (errs < 0) { f.descramble_extra.clear(); return false; }
        if (errs > 0) f.fixederrs++;
        bitstream_bch += data;
    }
    f.descrambled.clear();
    f.descrambled.push_back(bitstream_bch);
    return !bitstream_bch.empty();
}

// ---- LCW pretty ------------------------------------------------------------
static void build_lcw_header(Frame &f) {
    // f.lcw2 (13 bits), f.lcw3 (21 bits)
    int lcw_ft = bits_to_int(f.lcw2, 0, 2);
    int lcw_code = bits_to_int(f.lcw2, 2, 13);
    const Bits &l3 = f.lcw3;
    char buf[160];
    if (lcw_ft == 0) {
        const char *code;
        char extra[80] = "";
        if (lcw_code == 6) code = "geoloc";
        else if (lcw_code == 15) code = "<silent>";
        else if (lcw_code == 12) { code = "maint[1]";
            snprintf(extra, sizeof(extra), "[lqi:%d,power:%d]", bits_to_int(l3,19,21), bits_to_int(l3,16,19)); }
        else if (lcw_code == 0) { code = "sync";
            snprintf(extra, sizeof(extra), "[status:%d,dtoa:%d,dfoa:%d]", bits_to_int(l3,1,2), bits_to_int(l3,3,13), bits_to_int(l3,13,21)); }
        else if (lcw_code == 3) { code = "maint[2]";
            snprintf(extra, sizeof(extra), "[lqi:%d,power:%d,f_dtoa:%d,f_dfoa:%d]", bits_to_int(l3,1,3), bits_to_int(l3,3,6), bits_to_int(l3,6,13), bits_to_int(l3,13,20)); }
        else if (lcw_code == 1) { code = "switch";
            snprintf(extra, sizeof(extra), "[dtoa:%d,dfoa:%d]", bits_to_int(l3,3,13), bits_to_int(l3,13,21)); }
        else { static char rs[24]; snprintf(rs, sizeof(rs), "rsrvd(%d)", lcw_code); code = rs; }
        snprintf(buf, sizeof(buf), "LCW(%d,T:maint,C:%s%s)", f.ft, code, extra);
    } else if (lcw_ft == 2) {
        const char *code; char extra[120] = "";
        if (lcw_code == 12) code = "handoff_cand";
        else if (lcw_code == 3) { code = "handoff_resp";
            snprintf(extra, sizeof(extra), "[cand:%c,slot:%d,sband_up:%d,sband_dn:%d]",
                     bits_to_int(l3,2,3) ? 'S' : 'P', 1 + bits_to_int(l3,6,8), bits_to_int(l3,8,13), bits_to_int(l3,13,18)); }
        else if (lcw_code == 15) code = "<silent>";
        else { static char rs[24]; snprintf(rs, sizeof(rs), "rsrvd(%d)", lcw_code); code = rs; }
        snprintf(buf, sizeof(buf), "LCW(%d,T:hndof,C:%s%s)", f.ft, code, extra);
    } else if (lcw_ft == 1) {
        snprintf(buf, sizeof(buf), "LCW(%d,T:acchl,C:%d)", f.ft, lcw_code);
    } else {
        snprintf(buf, sizeof(buf), "LCW(%d,T:rsrvd,C:%d)", f.ft, lcw_code);
    }
    f.lcw_header = buf;
}

// ---- bodies ----------------------------------------------------------------

static void body_ira(Frame &f, ParsedMessage &out) {
    const Bits &b = f.descrambled.empty() ? Bits() : f.descrambled[0];
    if (b.size() < 63) { out.error = true; return; }
    int sat  = bits_to_int(b, 0, 7);
    int beam = bits_to_int(b, 7, 13);
    int px = bits_to_int(b, 14, 25) - bits_to_int(b, 13, 14) * (1 << 11);
    int py = bits_to_int(b, 26, 37) - bits_to_int(b, 25, 26) * (1 << 11);
    int pz = bits_to_int(b, 38, 49) - bits_to_int(b, 37, 38) * (1 << 11);
    double lat = std::atan2((double)pz, std::sqrt((double)px * px + (double)py * py)) * 180.0 / M_PI;
    double lon = std::atan2((double)py, (double)px) * 180.0 / M_PI;
    double alt = std::sqrt((double)px * px + (double)py * py + (double)pz * pz) * 4.0;

    out.has_ira = true;
    out.ira_sat = sat; out.ira_beam = beam;
    out.ira_lat = lat; out.ira_lon = lon;
    out.ira_alt_km = (int)(alt - 6378 + 23);
    out.type = "IRA";
    char buf[192];
    snprintf(buf, sizeof(buf), "IRA sat:%03d beam:%02d pos=(%+.2f/%+.2f) alt=%03d",
             sat, beam, lat, lon, out.ira_alt_km);
    out.pretty = buf;
}

static void body_ibc(Frame &f, ParsedMessage &out) {
    const Bits &b = f.descrambled.empty() ? Bits() : f.descrambled[0];
    std::vector<Bits> blocks; Bits extra;
    slice_extra(b, 42, blocks, extra);
    if (blocks.size() > 4) blocks.resize(4);
    out.type = "IBC";
    int bc_type = bits_to_int(f.header);
    out.ibc_type = bc_type;
    if (bc_type == 0 && !blocks.empty()) {
        const Bits &d = blocks[0];
        int sat = bits_to_int(d, 0, 7);
        int beam = bits_to_int(d, 7, 13);
        out.has_ibc = true; out.ibc_sat = sat; out.ibc_beam = beam;
        char buf[160];
        snprintf(buf, sizeof(buf), "IBC sat:%03d cell:%02d slot:%d sv_blkn:%d",
                 sat, beam, bits_to_int(d, 14, 15), bits_to_int(d, 15, 16));
        out.pretty = buf;
        if (blocks.size() > 1) {
            const Bits &t = blocks[1];
            int type = bits_to_int(t, 0, 6);
            if (type == 1) {
                out.ibc_iri_time = (uint32_t)bits_to_int(t, 10, 42);
                out.ibc_has_time = 1;
            }
        }
    } else {
        out.pretty = "IBC";
    }
}

static void body_ida(Frame &f, ParsedMessage &out) {
    const Bits &bch = f.descrambled.empty() ? Bits() : f.descrambled[0];
    out.type = "IDA";
    out.has_ida = true;
    if ((int)bch.size() >= 20) {
        out.ida_ctr = bits_to_int(bch, 5, 8);
        out.ida_len = bits_to_int(bch, 11, 16);
        out.ida_cont = bits_to_int(bch, 3, 4);
    }
    if ((int)bch.size() >= 9 * 20 + 16) {
        out.ida_len = bits_to_int(bch, 11, 16);
        Bits da_crc = bch.substr(9 * 20, 16);
        (void)da_crc;
        Bits crcstream = bch.substr(0, 20) + Bits(12, '0') + bch.substr(20, bch.size() - 20 - 4);
        std::vector<uint8_t> bytes;
        for (size_t i = 0; i + 8 <= crcstream.size(); i += 8)
            bytes.push_back((uint8_t)bits_to_int(crcstream.substr(i, 8)));
        uint16_t crc = crc16_ccitt_false(bytes.data(), bytes.size());
        out.ida_crc_ok = (crc == 0);
        // payload bytes: bch[20 : 9*20]
        Bits sbd = bch.substr(20, 9 * 20 - 20);
        for (size_t i = 0; i + 8 <= sbd.size(); i += 8)
            out.ida_data.push_back((uint8_t)bits_to_int(sbd.substr(i, 8)));
        out.ida_text = to_ascii_dots(out.ida_data);
    }
    char buf[256];
    std::string hex = to_hex(safe_sub(bch, 20, 160));
    snprintf(buf, sizeof(buf), "IDA len=%02d ctr=%d crc=%s [%s] %s",
             out.ida_len, out.ida_ctr, out.ida_crc_ok ? "OK" : "no",
             hex.c_str(), out.ida_text.c_str());
    out.pretty = buf;
}

static void body_lcw(Frame &f, ParsedMessage &out) {
    // f.descrambled_bits holds the 312-bit data after LCW; ft set.
    const Bits &data = f.descrambled_bits;
    build_lcw_header(f);
    if (f.ft == 0) {
        // VO
        out.type = "VOC";
        auto chunks = slice(data, 8);
        for (auto &c : chunks) out.voice_payload.push_back((uint8_t)bits_to_int(c));
        out.pretty = "VOC " + f.lcw_header + " [" + to_hex(data.substr(0, std::min<size_t>(data.size(), 39 * 8))) + "]";
    } else if (f.ft == 1) {
        // IP
        std::vector<uint8_t> payload_r;
        auto chunks = slice(data, 8);
        for (auto &c : chunks) { Bits r = c; std::reverse(r.begin(), r.end()); payload_r.push_back((uint8_t)bits_to_int(r)); }
        uint32_t crc = crc24_iip(payload_r.data(), payload_r.size());
        if (crc == 0 && payload_r.size() >= 4) {
            out.type = "IIP";
            int ip_hdr = payload_r[0], ip_seq = payload_r[1], ip_ack = payload_r[2], ip_cs = payload_r[3];
            char buf[192];
            snprintf(buf, sizeof(buf), "IIP type:%02x seq=%03d ack=%03d cs=%03d/%s",
                     ip_hdr, ip_seq, ip_ack, ip_cs, ((ip_hdr + ip_seq + ip_ack) % 255 == ip_cs) ? "OK" : "no");
            out.pretty = buf;
        } else {
            out.type = "IIU";
            out.pretty = "IIU " + f.lcw_header;
        }
    } else if (f.ft == 2) {
        body_ida(f, out); // DA path: f.descrambled holds blocks
    } else if (f.ft == 7) {
        out.type = "SY";
        out.pretty = "SY " + f.lcw_header;
    } else if (f.ft == 3) {
        out.type = "IU3";
        out.pretty = "IU3 " + f.lcw_header;
    } else if (f.ft == 6) {
        out.type = "IU6";
        out.pretty = "IU6 " + f.lcw_header;
    } else {
        out.type = "IRI";
        out.pretty = "IRI " + f.lcw_header;
    }
}

// ---- classification --------------------------------------------------------

bool parse_frame(const RawFrame &in, ParsedMessage &out) {
    out = ParsedMessage{};
    out.frequency = in.frequency;
    out.timestamp_ns = in.timestamp_ns;
    out.confidence = in.confidence;
    out.symbols = in.n_symbols;

    Frame f;
    f.frequency = in.frequency;
    f.timestamp_ns = in.timestamp_ns;
    f.confidence = in.confidence;
    f.level = in.level; f.noise = in.noise; f.snr = in.snr;
    f.symbols = in.n_symbols;
    f.bitstream_raw = symbol_reverse(in.bitstream);

    const double f_simplex = threshold_simplex();
    const double f_duplex = threshold_duplex();

    // access code / direction
    const Bits &bs = f.bitstream_raw;
    auto starts = [&](const Bits &a) { return bs.size() >= a.size() && bs.compare(0, a.size(), a) == 0; };
    if (starts(iridium_access)) f.uplink = 0;
    else if (starts(uplink_access)) f.uplink = 1;
    else if (starts(next_access_dl)) { f.uplink = 0; f.next = true; }
    else if (starts(next_access_ul)) { f.uplink = 1; f.next = true; }
    else {
        if (irid_debug) fprintf(stderr, "[cls] no access code; bs[0..24]=%s len=%zu\n",
                                bs.substr(0, 24).c_str(), bs.size());
        out.error = true; out.type = "ERR"; out.pretty = "Access code missing"; return false;
    }

    if (f.next) { out.type = "NX"; out.pretty = "NX (next-gen)"; return true; }

    Bits data = f.uplink ? bs.substr(uplink_access.size()) : bs.substr(iridium_access.size());
    out.uplink = f.uplink;

    // detect type (order matches bitsparser: MS, TL, BC, LW, RA, AQ)
    if ((int)data.size() >= 32 && data.compare(0, 32, header_messaging) == 0 &&
        f.frequency > f_simplex && !f.uplink)
        f.msgtype = "MS";
    else if ((int)data.size() >= 96 && data.compare(0, 96, header_time_location) == 0 &&
             f.frequency > f_simplex && !f.uplink)
        f.msgtype = "TL";

    if (f.msgtype.empty() && (int)data.size() > 70 && ndivide(hdr_poly, data.substr(0, 6)) == 0) {
        Bits o_bc1, o_bc2;
        de_interleave(data.substr(6, 64), o_bc1, o_bc2);
        if (irid_debug) fprintf(stderr, "[BC?] hdr=%s bc1ndiv=%d bc2ndiv=%d\n",
                                data.substr(0, 6).c_str(),
                                ndivide(ringalert_bch_poly, o_bc1.substr(0, 31)),
                                ndivide(ringalert_bch_poly, o_bc2.substr(0, 31)));
        if ((int)o_bc1.size() >= 31 && (int)o_bc2.size() >= 31 &&
            ndivide(ringalert_bch_poly, o_bc1.substr(0, 31)) == 0 &&
            ndivide(ringalert_bch_poly, o_bc2.substr(0, 31)) == 0)
            f.msgtype = "BC";
    } else if (irid_debug) {
        fprintf(stderr, "[BC?] skip: size=%zu hdrndiv=%d hdr=%s\n", data.size(),
                (int)data.size() >= 6 ? ndivide(hdr_poly, data.substr(0, 6)) : -1,
                data.size() >= 6 ? data.substr(0, 6).c_str() : "?");
    }
    if (f.msgtype.empty() && (int)data.size() > 64) {
        Bits l1, l2, l3;
        de_interleave_lcw(data.substr(0, 46), l1, l2, l3);
        if (ndivide(29, l1) == 0 && ndivide(41, l3) == 0) {
            Bits d, bch;
            int e2 = bch_repair(465, l2 + "0", d, bch);
            if (e2 != 0) e2 = bch_repair(465, l2 + "1", d, bch);
            if (e2 == 0) f.msgtype = "LW";
        }
    }
    if (f.msgtype.empty() && (f.frequency > f_simplex) && !f.uplink) {
        if ((int)data.size() >= 96) {
            Bits ra1, ra2, ra3;
            de_interleave3(data.substr(0, 96), ra1, ra2, ra3);
            if (ndivide(ringalert_bch_poly, ra1.substr(0, 31)) == 0 &&
                ndivide(ringalert_bch_poly, ra2.substr(0, 31)) == 0 &&
                ndivide(ringalert_bch_poly, ra3.substr(0, 31)) == 0)
                f.msgtype = "RA";
        }
    }
    if (f.msgtype.empty() && f.uplink && (int)data.size() >= 52 && (int)data.size() < 100)
        f.msgtype = "AQ";

    if (irid_debug) fprintf(stderr, "[cls] uplink=%d len=%zu msgtype=%s\n",
                            f.uplink, data.size(), f.msgtype.c_str());
    if (f.msgtype.empty()) {
        out.error = true; out.type = "ERR"; out.pretty = "unknown Iridium message type";
        return false;
    }

    // descramble per type
    if (f.msgtype == "RA") {
        Bits ra1, ra2, ra3;
        de_interleave3(data.substr(0, 96), ra1, ra2, ra3);
        f.descrambled.clear();
        f.descrambled.push_back(ra1); f.descrambled.push_back(ra2); f.descrambled.push_back(ra3);
        std::vector<Bits> blocks; Bits extra;
        slice_extra(data.substr(96), 64, blocks, extra);
        for (auto &x : blocks) { Bits a, b; de_interleave(x, a, b); f.descrambled.push_back(a); f.descrambled.push_back(b); }
        f.descramble_extra = extra;
        if (irid_debug) fprintf(stderr, "[RA] desc blocks=%zu ecc=%d\n", f.descrambled.size(), 0);
        if (ecc_decode(f, ringalert_bch_poly)) body_ira(f, out);
        else { out.type = "IME"; out.pretty = "IME RA"; out.error = true; }
    } else if (f.msgtype == "BC") {
        f.header = data.substr(0, 6);
        Bits d, bch;
        bch_repair1(hdr_poly, f.header, d, bch);
        f.header = d; // bc_type bits
        std::vector<Bits> blocks; Bits extra;
        slice_extra(data.substr(6, 131 * 2 - 6), 64, blocks, extra);
        f.descrambled.clear();
        for (auto &x : blocks) { Bits a, b; de_interleave(x, a, b); f.descrambled.push_back(a); f.descrambled.push_back(b); }
        f.descramble_extra = extra + safe_sub(data, 131 * 2);
        if (ecc_decode(f, ringalert_bch_poly)) body_ibc(f, out);
        else { out.type = "IME"; out.pretty = "IME BC"; out.error = true; }
    } else if (f.msgtype == "MS") {
        f.header = data.substr(0, 32);
        std::vector<Bits> blocks; Bits extra;
        slice_extra(data.substr(32), 64, blocks, extra);
        f.descrambled.clear();
        for (auto &x : blocks) { Bits a, b; de_interleave(x, a, b); f.descrambled.push_back(a); f.descrambled.push_back(b); }
        f.descramble_extra = extra;
        if (ecc_decode(f, messaging_bch_poly)) { out.type = "IMS"; out.pretty = "IMS"; }
        else { out.type = "IME"; out.pretty = "IME MS"; out.error = true; }
    } else if (f.msgtype == "TL") {
        out.type = "ITL";
        out.pretty = "ITL";
    } else if (f.msgtype == "LW") {
        Bits l1, l2, l3;
        de_interleave_lcw(data.substr(0, 46), l1, l2, l3);
        Bits d, bch;
        bch_repair(29, l1, f.lcw1, bch);
        int e2 = bch_repair(465, l2 + "0", f.lcw2, bch);
        Bits lcw2b;
        int e2b = bch_repair(465, l2 + "1", lcw2b, bch);
        if ((e2b >= 0 && e2b <= e2) || e2 < 0) { e2 = e2b; f.lcw2 = lcw2b; }
        bch_repair(41, l3, f.lcw3, bch);
        f.lcw_ok = (e2 >= 0);
        f.ft = bits_to_int(f.lcw1);
        Bits rest = data.substr(46);
        f.descramble_extra = rest.size() > 312 ? rest.substr(312) : Bits();
        f.descrambled_bits = rest.substr(0, std::min<size_t>(rest.size(), 312));

        if (f.ft == 2) {
            // DA: build descrambled blocks from descrambled_bits
            std::vector<Bits> blocks; Bits end;
            slice_extra(f.descrambled_bits, 124, blocks, end);
            f.descrambled.clear();
            for (auto &x : blocks) {
                Bits b1, b2; de_interleave(x, b1, b2);
                Bits both = b1 + b2;
                if (both.size() >= 124) {
                    Bits c1 = both.substr(0, 31), c2 = both.substr(31, 31),
                         c3 = both.substr(62, 31), c4 = both.substr(93, 31);
                    f.descrambled.push_back(c4); f.descrambled.push_back(c2);
                    f.descrambled.push_back(c3); f.descrambled.push_back(c1);
                }
            }
            if (end.size() % 2 == 1) end = end.substr(0, end.size() - 1);
            Bits b1, b2; de_interleave(end, b1, b2);
            if (b2.size() > 1) f.descrambled.push_back(b2.substr(1));
            if (b1.size() > 1) f.descrambled.push_back(b1.substr(1));
            if (lcw_ecc_decode(f, acch_bch_poly)) body_ida(f, out);
            else { out.type = "IME"; out.pretty = "IME DA"; out.error = true; }
        } else {
            body_lcw(f, out);
        }
    } else {
        out.type = f.msgtype; out.pretty = f.msgtype;
    }
    return true;
}

} // namespace irid
