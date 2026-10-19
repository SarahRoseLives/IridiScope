#include "parser/fec.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <unordered_map>
#include <utility>

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

std::vector<int> de_dqpsk(const Bits &bits) {
    static const int imap[4] = {0, 1, 3, 2};
    std::vector<int> symbols;
    symbols.reserve(bits.size() / 2);
    for (size_t i = 0; i + 1 < bits.size(); i += 2)
        symbols.push_back(imap[(bits[i] - '0') * 2 + (bits[i + 1] - '0')]);
    for (size_t c = 1; c < symbols.size(); c++)
        symbols[c] = (symbols[c - 1] + symbols[c]) % 4;
    return symbols;
}

void split_qpsk(const std::vector<int> &symbols, Bits &i_bits, Bits &q_bits) {
    i_bits.clear();
    q_bits.clear();
    i_bits.reserve(symbols.size());
    q_bits.reserve(symbols.size());
    for (int s : symbols) {
        switch (s) {
        case 0: i_bits += '0'; q_bits += '0'; break;
        case 1: i_bits += '1'; q_bits += '0'; break;
        case 2: i_bits += '1'; q_bits += '1'; break;
        default: i_bits += '0'; q_bits += '1'; break;
        }
    }
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
namespace {

struct RsGf {
    int charac = 255;
    std::vector<int> ex, lg;

    void init(int c_exp, int prim) {
        charac = (1 << c_exp) - 1;
        ex.assign((size_t)charac * 2, 0);
        lg.assign((size_t)charac + 1, 0);
        int x = 1;
        for (int i = 0; i < charac; i++) {
            ex[i] = x;
            lg[x] = i;
            x <<= 1;
            if (x & (1 << c_exp)) x ^= prim;
        }
        for (int i = charac; i < charac * 2; i++) ex[i] = ex[i - charac];
    }
    int mul(int a, int b) const {
        if (!a || !b) return 0;
        return ex[lg[a] + lg[b]];
    }
    int div(int a, int b) const {
        if (!a) return 0;
        return ex[(lg[a] - lg[b] + charac) % charac];
    }
    int pow(int a, int n) const {
        if (!a) return 0;
        if (n == 0) return 1;
        int e = (lg[a] * n) % charac;
        if (e < 0) e += charac;
        return ex[e];
    }
    int inv(int a) const { return ex[charac - lg[a]]; }
};

using Poly = std::vector<int>;

Poly pscale(const Poly &p, int x, const RsGf &g) {
    Poly r(p.size());
    for (size_t i = 0; i < p.size(); i++) r[i] = g.mul(p[i], x);
    return r;
}

Poly padd(const Poly &p, const Poly &q) {
    Poly r(std::max(p.size(), q.size()), 0);
    for (size_t i = 0; i < p.size(); i++) r[r.size() - p.size() + i] ^= p[i];
    for (size_t i = 0; i < q.size(); i++) r[r.size() - q.size() + i] ^= q[i];
    return r;
}

Poly pmul(const Poly &p, const Poly &q, const RsGf &g) {
    if (p.empty() || q.empty()) return {};
    Poly r(p.size() + q.size() - 1, 0);
    for (size_t j = 0; j < q.size(); j++) {
        if (!q[j]) continue;
        for (size_t i = 0; i < p.size(); i++)
            if (p[i]) r[i + j] ^= g.mul(p[i], q[j]);
    }
    return r;
}

std::pair<Poly, Poly> pdiv(const Poly &dividend, const Poly &divisor, const RsGf &g) {
    Poly out = dividend;
    for (size_t i = 0; i + divisor.size() <= out.size(); i++) {
        int coef = out[i];
        if (!coef) continue;
        for (size_t j = 1; j < divisor.size(); j++)
            if (divisor[j]) out[i + j] ^= g.mul(divisor[j], coef);
    }
    size_t sep = out.size() - (divisor.size() - 1);
    Poly q(out.begin(), out.begin() + sep);
    Poly r(out.begin() + sep, out.end());
    return {q, r};
}

int peval(const Poly &p, int x, const RsGf &g) {
    int y = p.empty() ? 0 : p[0];
    for (size_t i = 1; i < p.size(); i++) y = g.mul(y, x) ^ p[i];
    return y;
}

Poly reverse_poly(const Poly &p) { return Poly(p.rbegin(), p.rend()); }

Poly calc_syndromes(const Poly &msg, int nsym, int fcr, const RsGf &g) {
    Poly synd((size_t)nsym + 1, 0);
    for (int i = 0; i < nsym; i++)
        synd[i + 1] = peval(msg, g.pow(2, i + fcr), g);
    return synd;
}

Poly find_errata_locator(const std::vector<int> &e_pos, const RsGf &g) {
    Poly loc{1};
    for (int i : e_pos) {
        Poly t{g.pow(2, i), 1};
        loc = pmul(loc, t, g);
    }
    return loc;
}

Poly find_error_evaluator(const Poly &synd, const Poly &err_loc, int nsym,
                          const RsGf &g) {
    Poly prod = pmul(synd, err_loc, g);
    Poly divisor((size_t)nsym + 2, 0);
    divisor[0] = 1;
    return pdiv(prod, divisor, g).second;
}

std::vector<int> find_errors(const Poly &err_loc, int nmess, const RsGf &g) {
    int errs = (int)err_loc.size() - 1;
    std::vector<int> pos;
    for (int i = 0; i < nmess; i++)
        if (peval(err_loc, g.pow(2, i), g) == 0) pos.push_back(nmess - 1 - i);
    if ((int)pos.size() != errs) return {};
    return pos;
}

std::vector<int> forney_syndromes(const Poly &synd, const std::vector<int> &pos,
                                  int nmess, const RsGf &g) {
    std::vector<int> fsynd(synd.begin() + 1, synd.end());
    for (int p : pos) {
        int x = g.pow(2, nmess - 1 - p);
        for (size_t j = 0; j + 1 < fsynd.size(); j++)
            fsynd[j] = g.mul(fsynd[j], x) ^ fsynd[j + 1];
    }
    return fsynd;
}

Poly find_error_locator(const std::vector<int> &synd, int nsym, int erase_count,
                        const RsGf &g) {
    Poly err_loc{1}, old_loc{1};
    int synd_shift = 0;
    if ((int)synd.size() > nsym) synd_shift = (int)synd.size() - nsym;
    for (int i = 0; i < nsym - erase_count; i++) {
        int K = i + synd_shift;
        int delta = synd[K];
        for (size_t j = 1; j < err_loc.size(); j++) {
            int idx = K - (int)j;
            if (idx < 0) return {};
            delta ^= g.mul(err_loc[err_loc.size() - 1 - j], synd[idx]);
        }
        old_loc.push_back(0);
        if (delta) {
            if (old_loc.size() > err_loc.size()) {
                Poly new_loc = pscale(old_loc, delta, g);
                old_loc = pscale(err_loc, g.inv(delta), g);
                err_loc = new_loc;
            }
            err_loc = padd(err_loc, pscale(old_loc, delta, g));
        }
    }
    size_t first = 0;
    while (first < err_loc.size() && err_loc[first] == 0) first++;
    if (first) err_loc.erase(err_loc.begin(), err_loc.begin() + first);
    int errs = (int)err_loc.size() - 1;
    if ((errs - erase_count) * 2 + erase_count > nsym) return {};
    return err_loc;
}

Poly correct_errata(const Poly &msg_in, const Poly &synd,
                    const std::vector<int> &err_pos, int fcr, const RsGf &g) {
    Poly msg = msg_in;
    std::vector<int> coef_pos;
    for (int p : err_pos) coef_pos.push_back((int)msg.size() - 1 - p);
    Poly err_loc = find_errata_locator(coef_pos, g);
    Poly err_eval = find_error_evaluator(reverse_poly(synd), err_loc,
                                         (int)err_loc.size() - 1, g);
    err_eval = reverse_poly(err_eval);
    std::vector<int> X;
    for (int cp : coef_pos) X.push_back(g.pow(2, cp));

    Poly E(msg.size(), 0);
    for (size_t i = 0; i < X.size(); i++) {
        int Xi_inv = g.inv(X[i]);
        int denom = 1;
        for (size_t j = 0; j < X.size(); j++) {
            if (j == i) continue;
            denom = g.mul(denom, 1 ^ g.mul(Xi_inv, X[j]));
        }
        int y = peval(reverse_poly(err_eval), Xi_inv, g);
        y = g.mul(g.pow(X[i], 1 - fcr), y);
        E[err_pos[i]] = g.div(y, denom);
    }
    return padd(msg, E);
}

bool correct_msg(const std::vector<uint8_t> &in, int nsym, int fcr,
                 const std::vector<int> &erase_pos, const RsGf &g,
                 std::vector<uint8_t> &msg, std::vector<uint8_t> &ecc) {
    Poly msg_out(in.begin(), in.end());
    for (int p : erase_pos)
        if (p >= 0 && p < (int)msg_out.size()) msg_out[p] = 0;
    Poly synd = calc_syndromes(msg_out, nsym, fcr, g);
    bool any = false;
    for (int s : synd) if (s) { any = true; break; }
    if (!any) {
        size_t k = msg_out.size() - nsym;
        msg.assign(msg_out.begin(), msg_out.begin() + k);
        ecc.assign(msg_out.begin() + k, msg_out.end());
        return true;
    }
    std::vector<int> fsynd = forney_syndromes(synd, erase_pos, (int)msg_out.size(), g);
    Poly err_loc = find_error_locator(fsynd, nsym, (int)erase_pos.size(), g);
    if (err_loc.empty()) return false;
    std::vector<int> err_pos = find_errors(reverse_poly(err_loc), (int)msg_out.size(), g);
    if (err_pos.empty() && err_loc.size() > 1) return false;
    std::vector<int> all = erase_pos;
    all.insert(all.end(), err_pos.begin(), err_pos.end());
    Poly corrected = correct_errata(msg_out, synd, all, fcr, g);
    Poly check = calc_syndromes(corrected, nsym, fcr, g);
    for (int s : check) if (s) return false;
    size_t k = corrected.size() - nsym;
    msg.assign(corrected.begin(), corrected.begin() + k);
    ecc.assign(corrected.begin() + k, corrected.end());
    return true;
}

} // namespace

bool rs_fix(const std::vector<uint8_t> &in, std::vector<uint8_t> &msg,
            std::vector<uint8_t> &crs) {
    if (in.size() != 39) return false;
    RsGf g;
    g.init(8, 0x11d);
    std::vector<uint8_t> data = in;
    data.resize(47, 0);
    std::vector<int> erasures;
    for (int i = 39; i < 47; i++) erasures.push_back(i);
    std::vector<uint8_t> ecc;
    if (!correct_msg(data, 16, 0, erasures, g, msg, ecc)) return false;
    crs.assign(ecc.begin(), ecc.begin() + 8);
    return true;
}

bool rs6_fix(const std::vector<uint8_t> &in, std::vector<uint8_t> &msg,
             std::vector<uint8_t> &crs) {
    if (in.size() != 52) return false;
    RsGf g;
    g.init(6, 0x43);
    std::vector<uint8_t> ecc;
    if (!correct_msg(in, 10, 54, {}, g, msg, ecc)) return false;
    crs.assign(ecc.begin(), ecc.end());
    return true;
}

} // namespace irid
