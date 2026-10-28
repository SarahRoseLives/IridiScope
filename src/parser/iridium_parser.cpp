#include "parser/iridium_parser.h"
#include "parser/fec.h"
#include "parser/itl_tables.h"
#include "util/iritime.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace irid {

int irid_debug = 0; // set via --frame to trace classification
std::atomic<bool> irid_uw_ec{true};

// ---- constants (bitsparser.py) --------------------------------------------
static const Bits iridium_access = "001100000011000011110011";
static const Bits uplink_access  = "110011000011110011111100";
static const Bits next_access_dl = "110011110011111111111100";
static const Bits next_access_ul = "001111000000000011111111";
static const Bits header_messaging = "00110011111100110011001111110011";

// Unique words as DQPSK symbols (iridium-toolkit bitsparser.py).
static const int UW_DOWNLINK[12] = {0,2,2,2,2,0,0,0,2,0,0,2};
static const int UW_UPLINK[12]   = {2,2,0,0,0,2,0,0,2,0,2,2};

// Bits back to differentially decoded DQPSK symbols (bitsparser.py de_dqpsk).
static void de_dqpsk(const Bits &bits, int n_symbols, int *out) {
    static const int imap[4] = {0, 1, 3, 2};
    for (int i = 0; i < n_symbols; i++)
        out[i] = imap[(bits[2 * i] == '1' ? 2 : 0) + (bits[2 * i + 1] == '1' ? 1 : 0)];
    for (int i = 1; i < n_symbols; i++) out[i] = (out[i - 1] + out[i]) % 4;
}
static int symdiff(const int *a, const int *b, int n) {
    int d = 0;
    for (int i = 0; i < n; i++) d += a[i] != b[i];
    return d;
}
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
    // handoff_resp fields (iridium-toolkit bitsparser.py layout)
    bool ho = false;
    int ho_slot = 0, ho_sband_up = 0, ho_sband_dn = 0, ho_access = 0;
    bool ho_denied = false;
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
            f.ho = true;
            f.ho_denied = bits_to_int(l3,3,4) != 0;
            f.ho_slot = 1 + bits_to_int(l3,6,8);
            f.ho_sband_up = bits_to_int(l3,8,13);
            f.ho_sband_dn = bits_to_int(l3,13,18);
            f.ho_access = 1 + bits_to_int(l3,18,21);
            snprintf(extra, sizeof(extra), "[cand:%c,denied:%d,slot:%d,sband_up:%d,sband_dn:%d,access:%d]",
                     bits_to_int(l3,2,3) ? 'S' : 'P', f.ho_denied ? 1 : 0, f.ho_slot,
                     f.ho_sband_up, f.ho_sband_dn, f.ho_access); }
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
    out.ira_rai = bits_to_int(b, 49, 56);
    out.ira_bc_sb = bits_to_int(b, 58, 63);
    out.ira_is_ground = out.ira_alt_km < 100;
    out.type = "IRA";

    // Paging list: 42-bit PAGE entries after the 63-bit header. An all-ones
    // page terminates the list (bitsparser.py IridiumRAMessage).
    if (b.size() > 63) {
        Bits msg = b.substr(63);
        for (size_t off = 0; off + 42 <= msg.size(); off += 42) {
            Bits page = msg.substr(off, 42);
            if (page.find('0') == std::string::npos) break; // END marker
            ParsedMessage::IraPage p;
            p.tmsi = (uint32_t)bits_to_int(page, 0, 32);
            p.msc_id = bits_to_int(page, 34, 39);
            out.ira_page_list.push_back(p);
        }
    }
    out.ira_pages = (int)out.ira_page_list.size();

    char buf[192];
    snprintf(buf, sizeof(buf), "IRA sat:%03d beam:%02d pos=(%+.2f/%+.2f) alt=%03d RAI:%02d bc_sb:%02d",
             sat, beam, lat, lon, out.ira_alt_km, out.ira_rai, out.ira_bc_sb);
    out.pretty = buf;
    if (!out.ira_page_list.empty()) {
        char pb[64];
        snprintf(pb, sizeof(pb), " P%02d:", out.ira_pages);
        out.pretty += pb;
        for (const auto &p : out.ira_page_list) {
            snprintf(pb, sizeof(pb), " PAGE(tmsi:%08x msc_id:%02d)",
                     (unsigned)p.tmsi, p.msc_id);
            out.pretty += pb;
        }
    }
}

static void body_ibc(Frame &f, ParsedMessage &out) {
    const Bits &b = f.descrambled.empty() ? Bits() : f.descrambled[0];
    std::vector<Bits> blocks; Bits extra;
    slice_extra(b, 42, blocks, extra);
    if (blocks.size() > 4) blocks.resize(4);
    out.type = "IBC";
    int bc_type = bits_to_int(f.header);
    out.ibc_type = bc_type;
    out.pretty = "IBC";
    if (bc_type != 0 || blocks.empty()) return;

    const Bits &d = blocks[0];
    out.has_ibc = true;
    out.ibc_sat = bits_to_int(d, 0, 7);
    out.ibc_beam = bits_to_int(d, 7, 13);
    out.ibc_slot = bits_to_int(d, 14, 15);
    out.ibc_sv_blocking = bits_to_int(d, 15, 16);
    out.ibc_aq_classes = d.substr(16, 16);
    out.ibc_aq_subband = bits_to_int(d, 32, 37);
    out.ibc_aq_channel = bits_to_int(d, 37, 40);

    char buf[512];
    snprintf(buf, sizeof(buf),
             "IBC bc:%d sat:%03d cell:%02d slot:%d sv_blkn:%d aq_cl:%s aq_sb:%02d aq_ch:%d",
             bc_type, out.ibc_sat, out.ibc_beam, out.ibc_slot, out.ibc_sv_blocking,
             out.ibc_aq_classes.c_str(), out.ibc_aq_subband, out.ibc_aq_channel);
    std::string pretty = buf;

    if (blocks.size() > 1) {
        const Bits &t = blocks[1];
        int type = bits_to_int(t, 0, 6);
        out.ibc_subtype = type;
        if (type == 0) {
            out.ibc_max_uplink_pwr = bits_to_int(t, 36, 42);
            snprintf(buf, sizeof(buf), " max_uplink_pwr:%02d", out.ibc_max_uplink_pwr);
            pretty += buf;
        } else if (type == 1) {
            out.ibc_iri_time = (uint32_t)bits_to_int(t, 10, 42);
            out.ibc_has_time = 1;
            pretty += " time:" + iridi::fmt_iritime(out.ibc_iri_time);
        } else if (type == 2) {
            out.ibc_tmsi_expiry = (uint32_t)bits_to_int(t, 10, 42);
            out.ibc_has_tmsi_expiry = 1;
            pretty += " tmsi_expiry:" + iridi::fmt_iritime(out.ibc_tmsi_expiry);
        } else {
            snprintf(buf, sizeof(buf), " st:%02d %s", type, t.c_str());
            pretty += buf;
        }
    }

    for (size_t bi = 2; bi < blocks.size(); bi++) {
        const Bits &a = blocks[bi];
        if ((int)a.size() < 42) break;
        ParsedMessage::IbAssignment as;
        as.type = bits_to_int(a, 0, 3);
        if (a == Bits(42, '1') || a == "111000000000000000000000000000000000000000") {
            as.empty = true;
            pretty += " []";
            out.ibc_assignments.push_back(as);
            continue;
        }
        if (as.type == 0) {
            as.random_id = bits_to_int(a, 3, 11);
            as.timeslot = 1 + bits_to_int(a, 11, 13);
            as.ul_sb = bits_to_int(a, 13, 18);
            as.dl_sb = bits_to_int(a, 18, 23);
            as.access = 1 + bits_to_int(a, 23, 26);
            as.dtoa = bits_to_int(a, 26, 34);
            if (as.dtoa > 128) as.dtoa -= 256;
            as.dfoa = bits_to_int(a, 34, 40);
            snprintf(buf, sizeof(buf),
                     " [0 Rid:%03d ts:%d ul_sb:%02d dl_sb:%02d access:%d dtoa:%+04d dfoa:%02d %s]",
                     as.random_id, as.timeslot, as.ul_sb, as.dl_sb, as.access,
                     as.dtoa, as.dfoa, a.substr(40, 2).c_str());
        } else {
            as.raw = a.substr(3, 39);
            snprintf(buf, sizeof(buf), " [%d %s]", as.type, as.raw.c_str());
        }
        pretty += buf;
        out.ibc_assignments.push_back(as);
    }

    out.pretty = pretty;
}

static std::string itl_hex_bytes(const Bits &bits, size_t start, size_t nbits) {
    static const char *h = "0123456789abcdef";
    std::string out;
    for (size_t i = start; i + 8 <= start + nbits && i + 8 <= bits.size(); i += 8) {
        int v = bits_to_int(bits.substr(i, 8));
        out += h[(v >> 4) & 0xf];
        out += h[v & 0xf];
    }
    return out;
}

static std::string itl_two(int v) {
    char b[8];
    snprintf(b, sizeof(b), "%02d", v);
    return b;
}

static std::string itl_three(int v) {
    char b[8];
    snprintf(b, sizeof(b), "%03d", v);
    return b;
}

static void itl_map_sat(int num, int version, std::string &sat, std::string &msg) {
    if (version == 2) {
        if (num == 77) { sat = "---"; msg = "M08"; }
        else if (num < 66) {
            sat = "S" + itl_two(num % 11 + 1);
            msg = "M" + itl_two(num / 11 + 1);
        } else if (num >= 82 && num <= 84) {
            sat = "R" + itl_two(((num - 82) % 3) + 1);
            msg = "N01";
        } else if (num >= 85 && num <= 95) {
            sat = "S" + itl_two(num - 84);
            msg = "N02";
        } else if (num >= 96 && num <= 107) {
            sat = "R" + itl_two(((num - 96) % 3) + 1);
            msg = "N" + itl_two((num - 96) / 3 + 3);
        } else if (num == 108) { sat = "---"; msg = "SSS"; }
        else if (num == 111) { sat = "---"; msg = "N08"; }
        else { sat = "---"; msg = itl_three(num); }
    } else {
        if (num < 88) {
            sat = "S" + itl_two(num % 11 + 1);
            msg = "M" + itl_two(num / 11 + 1);
        } else {
            sat = "?";
            msg = itl_three(num);
        }
    }
}

static void body_itl(Frame &f, ParsedMessage &out) {
    out.type = "ITL";
    out.has_itl = true;
    const Bits &data = f.descrambled_bits;
    if (data.size() < 768) {
        out.pretty = "ITL (short)";
        return;
    }

    auto symbols = de_dqpsk(data);
    Bits ibits, qbits;
    split_qpsk(symbols, ibits, qbits);
    std::string i0 = itl_hex_bytes(ibits, 0, 128);
    std::string i1 = itl_hex_bytes(ibits, 128, 256);
    std::string q[4];
    for (int k = 0; k < 4; k++) q[k] = itl_hex_bytes(qbits, (size_t)k * 96, 96);

    int version = -1;
    for (int v = 0; v < 3; v++)
        if (i0 == kItlPrsHdr[v]) { version = v; break; }
    out.itl_version = version;
    if (version != 1 && version != 2) {
        out.pretty = "ITL V" + std::to_string(version) + " (raw)";
        return;
    }

    int plane = 0;
    if (version == 2) {
        for (int p = 0; p < 6; p++)
            if (i1 == kItlPrsPlanes[p]) { plane = p + 1; break; }
    } else {
        for (int p = 0; p < 6; p++)
            if (i1 == kItlPrsPlanesV1[p]) { plane = p + 1; break; }
    }
    if (!plane) {
        out.pretty = "ITL V" + std::to_string(version) + " (plane?)";
        return;
    }
    out.itl_plane = plane;

    int vals[4] = {0, 0, 0, 0};
    for (int k = 0; k < 4; k++) {
        int found = -1;
        if (version == 2) {
            for (int j = 0; j < 512; j++)
                if (q[k] == kItlPrsList[j]) { found = j % 128; break; }
        } else {
            for (int j = 0; j < 128; j++)
                if (q[k] == kItlPrsListV1[j]) { found = j; break; }
        }
        if (found < 0) {
            out.pretty = "ITL V" + std::to_string(version) + " (PRS?)";
            return;
        }
        vals[k] = found;
    }

    std::string sat, msg;
    itl_map_sat(vals[0], version, sat, msg);
    out.itl_sat = sat;
    out.itl_msg = msg;
    out.itl_words.assign(vals + 1, vals + 4);
    out.itl_ok = 1;

    char buf[256];
    snprintf(buf, sizeof(buf), "ITL V%d OK P%d %s %s %03d %03d %03d",
             version, plane, sat.c_str(), msg.c_str(), vals[1], vals[2], vals[3]);
    out.pretty = buf;
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

static void body_ims(Frame &f, ParsedMessage &out) {
    out.type = "IMS";
    out.has_ims = true;
    const Bits &bits = f.descrambled.empty() ? Bits() : f.descrambled[0];
    std::vector<Bits> blocks = slice(bits, 21);
    if (irid_debug) fprintf(stderr, "[ims] enter bits=%zu blocks=%zu\n", bits.size(), blocks.size());
    if (blocks.empty()) { out.pretty = "IMS"; return; }

    const Bits &h = blocks[0];
    if ((int)h.size() < 21) { out.pretty = "IMS"; return; }
    int ms_type = h[0] - '0';
    int bch_blocks = bits_to_int(h, 15, 19);
    bool groupA = (ms_type == 1);
    if (irid_debug)
        fprintf(stderr, "[ims] ms_type=%d bch_blocks=%d h=%s\n", ms_type, bch_blocks, h.c_str());
    if (bch_blocks < 2) { out.pretty = "IMS (short)"; return; }

    size_t want = (size_t)bch_blocks * 2;
    if (blocks.size() > want) blocks.resize(want);

    blocks.erase(blocks.begin()); // header block
    if (groupA) {
        if (blocks.size() < 2) { out.pretty = "IMS"; return; }
        size_t pop = blocks.size() >= 4 ? 4 : 2; // block-header message
        blocks.erase(blocks.begin(), blocks.begin() + pop);
    }

    // Up to two all-ones trailer blocks.
    int trailers = 0;
    while (!blocks.empty() && blocks.back().size() == 21 && blocks.back()[0] == '1') {
        blocks.pop_back();
        if (++trailers >= 2) break;
    }
    if (blocks.empty()) { out.pretty = "IMS"; return; }

    // Body bit string: drop the leading "odd" bit of every 21-bit block.
    std::string rest;
    for (const Bits &b : blocks)
        if (b.size() >= 21) rest += b.substr(1);
    if (irid_debug)
        fprintf(stderr, "[ims] after trailers blocks=%zu rest=%zu trailers=%d\n", blocks.size(),
                rest.size(), trailers);
    if (rest.size() <= 27) { out.pretty = "IMS"; return; }

    Bits ricbits = rest.substr(0, 22);
    std::reverse(ricbits.begin(), ricbits.end());
    int ric = bits_to_int(ricbits);
    int fmt = bits_to_int(rest, 22, 27);
    rest = rest.substr(27);
    if (irid_debug)
        fprintf(stderr, "[ims] ric=%d fmt=%d rest_after=%zu\n", ric, fmt, rest.size());
    if (rest.size() <= 16) { out.pretty = "IMS"; return; }
    int seq = bits_to_int(rest, 0, 6);
    rest = rest.substr(16); // msg_data

    out.has_msg = true;
    out.msg_ric = ric;
    out.msg_fmt = fmt;
    out.msg_seq = seq;
    if (irid_debug)
        fprintf(stderr, "[ims] ric=%d fmt=%d seq=%d rest=%zu\n", ric, fmt, seq, rest.size());

    char buf[192];
    if (fmt == 5) {
        if (rest.size() < 5 + 8) { out.pretty = "MSG (short)"; return; }
        int len_bit = rest[4] - '0';
        rest = rest.substr(5);
        int part = 0, parts = 0;
        if (len_bit == 1) {
            if (rest.size() < 4) return;
            int lfl = bits_to_int(rest, 0, 4);
            if (lfl < 1 || lfl > 2) return;
            if (rest.size() < (size_t)(4 + 2 * lfl)) return;
            part = bits_to_int(rest, 4, 4 + lfl);
            parts = bits_to_int(rest, 4 + lfl, 4 + 2 * lfl);
            rest = rest.substr(4 + 2 * lfl);
        }
        if (rest.size() < 8) return;
        int csum = bits_to_int(rest, 1, 8);
        rest = rest.substr(8);
        auto chars = slice(rest, 7);
        std::string txt;
        std::vector<uint8_t> bytes;
        for (const Bits &c : chars) {
            if ((int)c.size() < 7) break;
            int v = bits_to_int(c);
            bytes.push_back((uint8_t)v);
            if (v == 3) { /* ETX */ }
            else if (v >= 32 && v < 127) txt += (char)v;
            else { char t[8]; snprintf(t, sizeof(t), "[%d]", v); txt += t; }
        }
        out.msg_part = part;
        out.msg_parts = parts;
        out.msg_checksum = csum;
        out.msg_bytes = std::move(bytes);
        out.msg_text = txt;
        int s = 0;
        for (uint8_t b : out.msg_bytes) s = (s + b) % 128;
        out.msg_csum_ok = (((~s) & 127) == csum);
        snprintf(buf, sizeof(buf), "MSG ric:%07d fmt:%02d seq:%02d %d/%d csum:%s",
                 ric, fmt, seq, part, parts, out.msg_csum_ok ? "OK" : "no");
        out.pretty = buf;
    } else if (fmt == 3) {
        if (rest.size() < 1) return;
        rest = rest.substr(1);
        auto groups = slice(rest, 4);
        std::string digits;
        for (const Bits &g : groups) {
            if ((int)g.size() < 4) break;
            digits += "0123456789abcdef"[bits_to_int(g) & 0xF];
        }
        out.msg_part = 0;
        out.msg_parts = 0;
        out.msg_text = digits;
        out.msg_csum_ok = true;
        snprintf(buf, sizeof(buf), "MS3 ric:%07d fmt:%02d seq:%02d BCD:%s",
                 ric, fmt, seq, digits.c_str());
        out.pretty = buf;
    } else {
        snprintf(buf, sizeof(buf), "MSG ric:%07d fmt:%02d seq:%02d", ric, fmt, seq);
        out.pretty = buf;
    }
}

static void body_lcw(Frame &f, ParsedMessage &out) {
    // f.descrambled_bits holds the 312-bit data after LCW; ft set.
    const Bits &data = f.descrambled_bits;
    build_lcw_header(f);
    if (f.ho) {
        out.has_handoff = true;
        out.ho_slot = f.ho_slot; out.ho_sband_up = f.ho_sband_up;
        out.ho_sband_dn = f.ho_sband_dn; out.ho_access = f.ho_access;
        out.ho_denied = f.ho_denied;
    }
    if (f.ft == 0) {
        // VO: classify VDA / VO6 / VOD / VOZ / VOC (bitsparser.py IridiumVOMessage).
        auto chunks = slice(data, 8);
        std::vector<uint8_t> pf, pr;
        pf.reserve(chunks.size());
        pr.reserve(chunks.size());
        for (auto &c : chunks) {
            if ((int)c.size() < 8) break;
            uint8_t b = (uint8_t)bits_to_int(c);
            pf.push_back(b);
            uint8_t r = 0;
            for (int k = 0; k < 8; k++) r = (uint8_t)((r << 1) | ((b >> k) & 1));
            pr.push_back(r);
        }
        std::vector<uint8_t> p6;
        auto six = slice(data.substr(0, std::min<size_t>(data.size(), 312)), 6);
        for (auto &c : six)
            if ((int)c.size() == 6) p6.push_back((uint8_t)bits_to_int(c));

        std::string subtype = "VOC";
        if (!pr.empty() && crc24_iip(pr.data(), pr.size()) == 0) {
            subtype = "VDA";
        } else {
            std::vector<uint8_t> m6, c6;
            if (rs6_fix(p6, m6, c6)) {
                subtype = "VO6";
            } else {
                std::vector<uint8_t> md, cd;
                if (rs_fix(pf, md, cd)) {
                    subtype = "VOD";
                    out.voice_payload = md;
                    out.voice_payload.insert(out.voice_payload.end(), cd.begin(), cd.end());
                } else {
                    bool zeros = pf.size() >= 4;
                    if (zeros) {
                        for (size_t i = pf.size() - 4; i + 1 < pf.size(); i++)
                            if (pf[i] != 0) { zeros = false; break; }
                    }
                    int sum = 0;
                    for (uint8_t b : pf) sum = (sum + b) & 0xff;
                    if (zeros && sum == 0) {
                        subtype = "VOZ";
                        out.voice_payload = pf;
                    } else {
                        subtype = "VOC";
                        out.voice_payload = pf;
                    }
                }
            }
        }
        out.type = subtype;
        out.voice_subtype = subtype;
        out.pretty = subtype + " " + f.lcw_header;
        if (subtype == "VOC" || subtype == "VOZ" || subtype == "VOD")
            out.pretty += " [" + to_hex(data.substr(0, std::min<size_t>(data.size(), 39 * 8))) + "]";
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
        // Tolerate up to 3 wrong symbols in the unique word (iridium-toolkit
        // --uw-ec). A random burst matches within 3 of 12 quaternary symbols
        // with probability ~0.04%, and later ECC rejects most of those.
        bool recovered = false;
        if (irid_uw_ec.load(std::memory_order_relaxed) && bs.size() >= iridium_access.size()) {
            int sym[12];
            de_dqpsk(bs, 12, sym);
            const int ddl = symdiff(sym, UW_DOWNLINK, 12), dul = symdiff(sym, UW_UPLINK, 12);
            if (ddl < 4) { f.uplink = 0; out.uw_errors = ddl; recovered = true; }
            else if (dul < 4) { f.uplink = 1; out.uw_errors = dul; recovered = true; }
        }
        if (!recovered) {
            if (irid_debug) fprintf(stderr, "[cls] no access code; bs[0..24]=%s len=%zu\n",
                                    bs.substr(0, 24).c_str(), bs.size());
            out.error = true; out.type = "ERR"; out.pretty = "Access code missing"; return false;
        }
    }

    if (f.next) { out.type = "NX"; out.pretty = "NX (next-gen)"; return true; }

    Bits data = f.uplink ? bs.substr(uplink_access.size()) : bs.substr(iridium_access.size());
    out.uplink = f.uplink;

    // detect type (order matches bitsparser: MS, TL, BC, LW, RA, AQ)
    if ((int)data.size() >= 32 && data.compare(0, 32, header_messaging) == 0 &&
        f.frequency > f_simplex && !f.uplink)
        f.msgtype = "MS";
    else if ((int)data.size() >= 96 + 768 &&
             bitdiff(data.substr(0, 96), header_time_location) < 4 &&
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
        if (ecc_decode(f, messaging_bch_poly)) body_ims(f, out);
        else { out.type = "IME"; out.pretty = "IME MS"; out.error = true; }
    } else if (f.msgtype == "TL") {
        f.descrambled_bits = data.substr(96, 768);
        body_itl(f, out);
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
