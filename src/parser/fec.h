// Bit, BCH, CRC and Reed-Solomon helpers for the native Iridium parser.
// Ported from iridium-toolkit (bitsparser.py / bch.py / rs.py / rs6.py / util.py).
#ifndef IRIDISCOPE_FEC_H
#define IRIDISCOPE_FEC_H

#include <cstdint>
#include <string>
#include <vector>

namespace irid {

using Bits = std::string; // '0'/'1' characters

// ---- bit helpers -----------------------------------------------------------
int  bitdiff(const Bits &a, const Bits &b);
Bits symbol_reverse(const Bits &b);
void de_interleave(const Bits &g, Bits &odd, Bits &even);
void de_interleave3(const Bits &g, Bits &first, Bits &second, Bits &third);
void de_interleave_lcw(const Bits &bits, Bits &l1, Bits &l2, Bits &l3);
std::vector<Bits> slice(const Bits &s, int n);
void slice_extra(const Bits &s, int n, std::vector<Bits> &blocks, Bits &extra);
int  bits_to_int(const Bits &b);
int  bits_to_int(const Bits &b, int from, int to); // int(b[from:to], 2)

// DQPSK -> BPSK symbols (bitsparser.py de_dqpsk/split_qpsk).
std::vector<int> de_dqpsk(const Bits &bits);
void split_qpsk(const std::vector<int> &symbols, Bits &i_bits, Bits &q_bits);

// ---- GF(2) division / BCH --------------------------------------------------
int  nndivide(int poly, int num);
int  ndivide(int poly, const Bits &bits);

// BCH repair. Returns number of corrected errors (>=0) or -1 if uncorrectable.
// Splits the repaired value into data (leading) and bch (trailing parity) bits,
// matching bch.bch_repair(). bch_repair1 limits to a single-bit search.
int  bch_repair(int poly, const Bits &bits, Bits &data, Bits &bch);
int  bch_repair1(int poly, const Bits &bits, Bits &data, Bits &bch);

// ---- CRC -------------------------------------------------------------------
uint16_t crc16_ccitt_false(const uint8_t *data, size_t n);
uint32_t crc24_iip(const uint8_t *data, size_t n);
uint16_t iaq_crc16(const uint8_t *data, size_t n);
uint16_t kermit_crc(const uint8_t *data, size_t n); // ACARS CRC

// ---- Reed-Solomon ----------------------------------------------------------
// rs.rs_fix : GF(2^8), prim 0x11d, nsym=8, elen=8, fcr=0  (IIQ/LCW3/VOD)
// rs6.rs_fix: GF(2^6), prim 0x43,  nsym=10, elen=0, fcr=54 (VO6/LCW3)
// On success sets msg (corrected codeword minus ecc) and crs (correction
// values). Mirrors the Python return (ok, msg, crs).
bool rs_fix(const std::vector<uint8_t> &in, std::vector<uint8_t> &msg,
            std::vector<uint8_t> &crs);
bool rs6_fix(const std::vector<uint8_t> &in, std::vector<uint8_t> &msg,
             std::vector<uint8_t> &crs);

} // namespace irid

#endif // IRIDISCOPE_FEC_H
