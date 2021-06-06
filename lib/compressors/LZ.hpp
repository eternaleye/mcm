/*	MCM file compressor

  Copyright (C) 2013, Google Inc.
  Authors: Mathieu Chartier

  LICENSE

    This file is part of the MCM file compressor.

    MCM is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MCM is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MCM.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _LZ_HPP_
#define _LZ_HPP_

#include <cstddef>           // for size_t
#include <cstdint>           // for uint8_t, uint32_t, uint64_t

#include "Compressor.hpp"    // for MemoryCompressor
#include "MatchFinder.hpp"   // for Match
#include "Stream.hpp"        // for Stream (ptr only)
#include "Util.hpp"          // for RoundUp

template <const size_t kMatchBits, const size_t kDistBits>
class SimpleEncoder {
  static const size_t kTotalBits = 1u + kMatchBits + kDistBits;
  static const size_t kRoundedBits = RoundUp(kTotalBits, 8u);
  static const size_t kDistMask = (1u << kDistBits) - 1;
  static_assert(kRoundedBits % 8 == 0, "must be aligned");
public:
  template <typename SOut>
  void encodeMatch(SOut& sout, const Match& match) {
    size_t w = 1u;
    w = (w << kMatchBits) + match.Length();
    w = (w << kDistBits) + match.Pos();
    w <<= kRoundedBits - kTotalBits;
    if (kRoundedBits >= 24) sout.put((w >> 24) & 0xFF);
    if (kRoundedBits >= 16) sout.put((w >> 16) & 0xFF);
    if (kRoundedBits >= 8) sout.put((w >> 8) & 0xFF);
    if (kRoundedBits >= 0) sout.put((w >> 0) & 0xFF);
  }
  // Non match is always before the match.
  template <typename SIn>
  void decodeMatch(SIn& in, Match* out_match, size_t* out_non_match_len, uint8_t* out_non_match) {
    uint8_t b = in.get();
    const auto match_flag = b & 0x80;
    if (match_flag) {
      *out_non_match_len = 0;
      b ^= match_flag;
      size_t w = b;
      if (kRoundedBits >= 8) w = (w << 8) | in.get();
      if (kRoundedBits >= 16) w = (w << 8) | in.get();
      if (kRoundedBits >= 24) w = (w << 8) | in.get();
      *out_match = Match(w >> kDistBits, w & kDistMask);
    } else {
      // Read non match len.
      *out_non_match_len = b;
      for (size_t i = 0; i < b; ++i) {
        out_non_match[i] = in.get();
      }
    }
  }
  template <typename SOut, typename MF>
  void encodeNonMatch(SOut& sout, MF& match_finder) {
    // sout. match_finder.nonMatchLength();
  }
};

// Very fast lz that always copies 16 bytes.
template <class MatchFinder>
class LZ16 : public MemoryCompressor {
  static constexpr size_t kMinMatch = 5;
  static constexpr size_t kMaxMatch = 15;
  static constexpr size_t kMaxNonMatch = 15;
public:
  virtual size_t getMaxExpansion(size_t in_size) {
    return in_size * 2;
  }
  virtual size_t compress(uint8_t* in, uint8_t* out, size_t count);
  virtual void decompress(uint8_t* in, uint8_t* out, size_t count);
};

#endif
