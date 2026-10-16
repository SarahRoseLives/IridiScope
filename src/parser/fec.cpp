#include "parser/fec.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <unordered_map>

namespace irid {

// ---- bit helpers -----------------------------------------------------------

int bitdiff(const Bits &a, const Bits &b) {
    int n = (int)std::min(a.size(), b.size());
    int d = 0;
    for (int i = 0; i < n; i++) if (a[i] != b[i]) d++;
    return d;
}

Bits symbol_reverse(const Bits &b) {
    Bits r = b;
    for (size_t x = 0; x + 1 < r.size(); x += 2) std::swap(r[x], r[x + 1]);
    return r;
}

// Python's de_interleave builds a list of 2-char "symbols" (group[z+1]+group[z])
// and then concatenates whole symbols. Here ns = number of symbols and symbol i
// is g[2i+1], g[2i].
static inline void append_symbol(Bits &dst, const Bits &g, int i) {
    dst += g[2 * i + 1];
    dst += g[2 * i];
}

void de_interleave(const Bits &g, Bits &odd, Bits &even) {
    int ns = (int)g.size() / 2;
    even.clear();
    odd.clear();
    for (int x = ns - 2; x >= 0; x -= 2) append_symbol(even, g, x);
    for (int x = ns - 1; x >= 0; x -= 2) append_symbol(odd, g, x);
}

void de_interleave3(const Bits &g, Bits &first, Bits &second, Bits &third) {
    int ns = (int)g.size() / 2;
    first.clear(); second.clear(); third.clear();
    for (int x = ns - 3; x >= 0; x -= 3) append_symbol(third, g, x);
    for (int x = ns - 2; x >= 0; x -= 3) append_symbol(second, g, x);
    for (int x = ns - 1; x >= 0; x -= 3) append_symbol(first, g, x);
}

void de_interleave_lcw(const Bits &bits, Bits &l1, Bits &l2, Bits &l3) {
    static const int tbl[46] = {
        40, 39, 36, 35, 32, 31, 28, 27, 24, 23, 20, 19, 16, 15, 12, 11, 8, 7, 4, 3,
        41, 38, 37, 34, 33, 30, 29, 26, 25, 22, 21, 18, 17, 14, 13, 10, 9, 6, 5, 2,
        1, 46, 45, 44, 43, 42
    };
    Bits lcw;
    for (int i = 0; i < 46; i++) {
        int idx = tbl[i] - 1;
        if (idx >= 0 && idx < (int)bits.size()) lcw += bits[idx];
    }
    l1 = lcw.substr(0, 7);
    l2 = lcw.substr(7, 13);
    l3 = lcw.substr(20);
}

std::vector<Bits> slice(const Bits &s, int n) {
    std::vector<Bits> out;
    for (size_t x = 0; x < s.size(); x += n) out.push_back(s.substr(x, n));
    return out;
}

void slice_extra(const Bits &s, int n, std::vector<Bits> &blocks, Bits &extra) {
    blocks.clear();
    for (size_t x = 0; x <= s.size(); x += n) blocks.push_back(s.substr(x, n));
    if (!blocks.empty()) { extra = blocks.back(); blocks.pop_back(); }
    else extra.clear();
}

int bits_to_int(const Bits &b) {
    int v = 0;
    for (char c : b) v = (v << 1) | (c == '1' ? 1 : 0);
    return v;
}

int bits_to_int(const Bits &b, int from, int to) {
    if (from < 0) from = 0;
    if (to > (int)b.size()) to = (int)b.size();
    int v = 0;
    for (int i = from; i < to; i++) v = (v << 1) | (b[i] == '1' ? 1 : 0);
    return v;
}

// ---- GF(2) division --------------------------------------------------------

int nndivide(int poly, int num) {
    if (num == 0) return 0;
    int nbits = 0; for (int t = num; t; t >>= 1) nbits++;
    int pbits = 0; for (int t = poly; t; t >>= 1) pbits++;
    int bits = nbits - pbits;
    int pow = 1 << (nbits - 1);
    while (bits >= 0) {
        if (num >= pow) num ^= (poly << bits);
        pow >>= 1;
        bits -= 1;
    }
    return num;
}

int ndivide(int poly, const Bits &bits) { return nndivide(poly, bits_to_int(bits)); }

// ---- BCH syndromes ---------------------------------------------------------

namespace {
struct SyndromeTable {
    std::vector<int>     ecnt;  // -2 = absent (uncorrectable), >=0 error count
    std::vector<uint32_t> eloc;
};

std::unordered_map<int, SyndromeTable> &syndromes() {
    static std::unordered_map<int, SyndromeTable> s;
    return s;
}

void mk_syn(int poly, int nbits, int synbits, int errors) {
    SyndromeTable t;
    t.ecnt.assign(1 << synbits, -2);
    t.eloc.assign(1 << synbits, 0);
    for (int n1 = 0; n1 < nbits; n1++) {
        uint32_t val = (uint32_t)(1u << n1);
        int r = nndivide(poly, (int)val);
        t.ecnt[r] = 1; t.eloc[r] = val;
    }
    if (errors >= 2) {
        for (int n1 = 0; n1 < nbits; n1++)
            for (int n2 = n1 + 1; n2 < nbits; n2++) {
                uint32_t val = (uint32_t)((1u << n1) | (1u << n2));
                int r = nndivide(poly, (int)val);
                if (t.ecnt[r] == -2) { t.ecnt[r] = 2; t.eloc[r] = val; }
            }
    }
    syndromes()[poly] = std::move(t);
}

void init_bch() {
    // Thread-safe one-time initialization (C++11 magic static).
    static const bool inited = [] {
        mk_syn(29,   7,  4, 1);
        mk_syn(465,  14, 8, 2);
        mk_syn(41,   26, 5, 1);
        mk_syn(1897, 31, 10, 2);
        mk_syn(1207, 31, 10, 2);
        mk_syn(3545, 31, 11, 2);
        return true;
    }();
    (void)inited;
}

int poly_bits(int poly) { int c = 0; while (poly) { c++; poly >>= 1; } return c; }

Bits int_to_bits(uint32_t v, int len) {
    Bits b(len, '0');
    for (int i = 0; i < len; i++) b[len - 1 - i] = (v >> i) & 1 ? '1' : '0';
    return b;
}
} // namespace

int bch_repair(int poly, const Bits &bits, Bits &data, Bits &bch) {
    init_bch();
    int num = bits_to_int(bits);
    int r = nndivide(poly, num);
    int errs;
    uint32_t repaired;
    if (r == 0) { errs = 0; repaired = (uint32_t)num; }
    else {
        auto it = syndromes().find(poly);
        if (it == syndromes().end() || r >= (int)it->second.ecnt.size() || it->second.ecnt[r] == -2) {
            data = bits; bch.clear();
            return -1;
        }
        errs = it->second.ecnt[r];
        repaired = it->second.eloc[r] ^ (uint32_t)num;
    }
    Bits rep = int_to_bits(repaired, (int)bits.size());
    int k = poly_bits(poly) - 1;
    data = rep.substr(0, rep.size() - k);
    bch = rep.substr(rep.size() - k);
    return errs;
}

int bch_repair1(int poly, const Bits &bits, Bits &data, Bits &bch) {
    int num = bits_to_int(bits);
    int r = nndivide(poly, num);
    int errs;
    uint32_t repaired;
    if (r == 0) { errs = 0; repaired = (uint32_t)num; }
    else {
        errs = -1; repaired = (uint32_t)num;
        for (int b1 = 0; b1 < (int)bits.size(); b1++) {
            int num1 = num ^ (1 << b1);
            if (nndivide(poly, num1) == 0) { errs = 1; repaired = (uint32_t)num1; break; }
        }
    }
    Bits rep = int_to_bits(repaired, (int)bits.size());
    int k = poly_bits(poly) - 1;
    data = rep.substr(0, rep.size() - k);
    bch = rep.substr(rep.size() - k);
    return errs;
}

// ---- CRC -------------------------------------------------------------------

namespace {
uint32_t reflect(uint32_t v, int bits) {
    uint32_t r = 0;
    for (int i = 0; i < bits; i++) { r = (r << 1) | (v & 1); v >>= 1; }
    return r;
}

uint64_t crc_generic(int width, uint64_t poly, uint64_t init, bool refin,
                     bool refout, uint64_t xorout, const uint8_t *data, size_t n) {
    uint64_t mask = (width == 64) ? ~0ull : ((1ull << width) - 1);
    uint64_t top = 1ull << (width - 1);
    uint64_t crc = init & mask;
    if (refin) {
        uint64_t rpoly = reflect((uint32_t)poly, width);
        for (size_t i = 0; i < n; i++) {
            crc ^= data[i];
            for (int b = 0; b < 8; b++)
                crc = (crc & 1) ? ((crc >> 1) ^ rpoly) : (crc >> 1);
        }
    } else {
        for (size_t i = 0; i < n; i++) {
            crc ^= ((uint64_t)data[i]) << (width - 8);
            for (int b = 0; b < 8; b++)
                crc = (crc & top) ? ((crc << 1) ^ poly) : (crc << 1);
        }
    }
    crc &= mask;
    if (refout != refin) crc = reflect((uint32_t)crc, width);
    return crc ^ xorout;
}
} // namespace

uint16_t crc16_ccitt_false(const uint8_t *data, size_t n) {
    return (uint16_t)crc_generic(16, 0x1021, 0xFFFF, false, false, 0x0000, data, n);
}

uint32_t crc24_iip(const uint8_t *data, size_t n) {
    return (uint32_t)crc_generic(24, 0x1BBA1B5, 0xffffff ^ 0x0c91b6, true, true,
                                 0x0c91b6, data, n);
}

uint16_t iaq_crc16(const uint8_t *data, size_t n) {
    return (uint16_t)crc_generic(16, 0x15101, 0, false, false, 0, data, n);
}

uint16_t kermit_crc(const uint8_t *data, size_t n) {
    // CRC-16/KERMIT: poly 0x1021, init 0, refin/refout true, xorout 0.
    return (uint16_t)crc_generic(16, 0x1021, 0, true, true, 0, data, n);
}

// ---- Reed-Solomon ----------------------------------------------------------
// TODO: full error/erasure decoding. Until implemented, rs_fix/rs6_fix report
// "not correctable" so affected subtypes fall back (VO->VOC, IP->IIU). This
// does not affect IRA/IBC/IDA/LCW parsing.
bool rs_fix(const std::vector<uint8_t> &, std::vector<uint8_t> &, std::vector<uint8_t> &) {
    return false;
}
bool rs6_fix(const std::vector<uint8_t> &, std::vector<uint8_t> &, std::vector<uint8_t> &) {
    return false;
}

} // namespace irid
