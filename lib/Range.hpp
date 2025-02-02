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

#ifndef _RANGE_HPP_
#define _RANGE_HPP_

#include <cassert>   // for assert
#include <cstdint>   // for uint32_t, uint8_t, uint64_t

// From 7zip, added single bit functions
class Range7 {
private:
  static constexpr uint32_t TopBits = 24, TopValue = 1 << TopBits, Top = 0xFFFFFFFF;

  uint32_t Range = Top, Code = 0, _cacheSize = 1;
  uint64_t Low = 0;
  uint8_t _cache = 0;

  template <typename TOut>
  inline void shiftLow(TOut& sout) { //Emit the top byte 
    if (static_cast<uint32_t>(Low) < static_cast<uint32_t>(0xFF << TopBits) || (Low >> 32) != 0) {
      uint8_t temp = _cache;
      do {
        sout.put(uint8_t(temp + uint8_t(Low >> 32)));
        temp = 0xFF;
      } while (--_cacheSize);
      _cache = uint8_t(uint32_t(Low >> 24));
    }
    ++_cacheSize;
    Low = static_cast<uint32_t>(Low << 8);
  }
public:
  template <typename TOut>
  inline void IncreaseRange(TOut& out) {
    while (Range < TopValue) {
      Range <<= 8;
      shiftLow(out);
    }
  }

  inline uint32_t getDecodedBit(uint32_t p, uint32_t shift) {
    const uint32_t mid = (Range >> shift) * p;
    uint32_t bit = Code < mid;
    if (bit) {
      Range = mid;
    } else {
      Code -= mid;
      Range -= mid;
    }
    return bit;
  }

  template <typename TOut>
  inline void encode(TOut& out, uint32_t bit, uint32_t p, uint32_t shift) {
    assert(p < (1U << shift));
    assert(p != 0U);
    const uint32_t mid = (Range >> shift) * p;
    if (bit) {
      Range = mid;
    } else {
      Low += mid;
      Range -= mid;
    }
    IncreaseRange(out);
  }

  template <typename TIn>
  inline uint32_t decode(TIn& in, uint32_t p, uint32_t shift) {
    assert(p < (1U << shift));
    assert(p != 0U);
    auto ret = getDecodedBit(p, shift);
    Normalize(in);
    return ret;
  }

  template <typename TOut>
  inline void encodeBit(TOut& out, uint32_t bit) {
    Range >>= 1;
    if (bit) {
      Low += Range;
    }
    while (Range < TopValue) {
      Range <<= 8;
      shiftLow(out);
    }
  }

  template <typename TIn>
  inline uint32_t decodeBit(TIn& in) {
    uint32_t top = Range;
    Range >>= 1;
    if (Code >= Range) {
      Code -= Range;
      Normalize(in);
      return 1;
    } else {
      Normalize(in);
      return 0;
    }
  }

  template <typename TOut>
  void EncodeBits(TOut& Out, uint32_t value, int numBits) {
    for (numBits--; numBits >= 0; numBits--) {
      Range >>= 1;
      Low += Range & (0 - ((value >> numBits) & 1));
      while (Range < TopValue) {
        Range <<= 8;
        shiftLow(Out);
      }
    }
  }

  template <typename TIn>
  uint32_t DecodeDirectBits(TIn& In, int numTotalBits) {
    uint32_t range = Range, code = Code, result = 0;
    for (int i = numTotalBits; i != 0; i--) {
      range >>= 1;
      uint32_t t = (code - range) >> 31;
      code -= range & (t - 1);
      result = (result << 1) | (1 - t);
      if (range < TopValue) {
        code = (code << 8) | (In.get() & 0xFF);
        range <<= 8;
      }
    }
    Range = range;
    Code = code;
    return result;
  }

  template <typename TIn>
  uint32_t DecodeDirectBit(TIn& sin) {
    Range >>= 1;
    uint32_t t = (Code - Range) >> 31;
    Code -= Range & (t - 1);
    Normalize(sin);
    return t ^ 1;
  }

  template <typename TOut>
  inline void Encode(TOut& Out, uint32_t start, uint32_t size, uint32_t total) {
    assert(size > 0);
    assert(start < total);
    assert(start + size <= total);
    Range /= total;
    Low += start * Range;
    Range *= size;
    while (Range < TopValue) {
      Range <<= 8;
      shiftLow(Out);
    }
  }

  template <typename TIn>
  inline void Decode(TIn& In, uint32_t start, uint32_t size) {
    Code -= start * Range;
    Range *= size;
    Normalize(In);
  }

  template <typename TOut>
  inline void encodeDirect(TOut& Out, uint32_t start, uint32_t total) {
    assert(start < total);
    Range /= total;
    Low += start * Range;
    while (Range < TopValue) {
      Range <<= 8;
      shiftLow(Out);
    }
  }

  template <typename TIn>
  inline uint32_t decodeByte(TIn& In) {
    Range >>= 8;
    uint32_t start = Code / Range;
    Code -= start * Range;
    Normalize(In);
    return start;
  }

  template <typename TIn>
  inline uint32_t decodeDirect(TIn& In, uint32_t Total) {
    uint32_t start = GetThreshold(Total);
    Code -= start * Range;
    Normalize(In);
    return start;
  }

  template <typename TOut>
  void flush(TOut& Out) {
    for (uint32_t i = 0; i < 5; i++)
      shiftLow(Out);
  }

  template <typename TIn>
  void initDecoder(TIn& In) {
    *this = Range7();
    Code = 0;
    Range = Top;
    for (uint32_t i = 0; i < 5; i++)
      Code = (Code << 8) | (In.get() & 0xFF);
  }

  inline uint32_t GetThreshold(uint32_t Total) {
    return Code / (Range /= Total);
  }

  template <typename TIn>
  inline void Normalize(TIn& In) {
    while (Range < TopValue) {
      Code = (Code << 8) | (In.get() & 0xFF);
      Range <<= 8;
    }
  }
};

#endif
