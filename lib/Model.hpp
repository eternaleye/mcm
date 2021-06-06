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

#ifndef _MODEL_HPP_
#define _MODEL_HPP_

#include <cassert>  // for assert
#include <cstddef>  // for size_t
#include <cstdint>  // for uint32_t, int32_t

template <const uint64_t n>
struct _bitSize { static const uint64_t value = 1 + _bitSize<n / 2>::value; };

template <>
struct _bitSize<0> { static const uint64_t value = 0; };

// Bit probability model (should be rather fast).
template <typename T, const uint32_t _shift, const uint32_t _learn_rate = 5, const uint32_t _bits = 15>
class safeBitModel {
protected:
  static const uint32_t pmax = (1 << _bits) - 1;
public:
  static const uint32_t shift = _shift;
  static const uint32_t learn_rate = _learn_rate;
  static const uint32_t max = 1 << shift;

  inline void update(T bit) {
    int round = 1 << (_learn_rate - 1);
    p += ((static_cast<int>(bit) << _bits) - static_cast<int>(p) + round) >> _learn_rate;
  }

  inline uint32_t getP() const {
    uint32_t ret = p >> (_bits - shift);
    ret += ret == 0;
    return ret;
  }

private:
  T p = pmax / 2;
};

template <const uint32_t _shift, const uint32_t _learn_rate = 5, const uint32_t _bits = 15>
class bitLearnModel {
  static const uint32_t kCountBits = 8;
  static const uint32_t kCountMask = (1 << kCountBits) - 1;
  static const uint32_t kInitialCount = 2;
  // Count is in low kCountBits.
  uint32_t p;
public:
  static const uint32_t shift = _shift;
  static const uint32_t learn_rate = _learn_rate;
  static const uint32_t max = 1 << shift;

  inline void init(int new_p = 1 << (_shift - 1)) {
    setP(new_p);
  }

  inline bitLearnModel() {
    init();
  }

  inline void update(uint32_t bit) {
    const size_t count = p & kCountMask;
    // 255 / 32 = 9
    const size_t learn_rate = 2 + (count >> 5);
    const int m[2] = { kCountMask, (1u << 31) - 1 };
    p = p + (((m[bit] - static_cast<int>(p)) >> learn_rate) & ~kCountMask);
    p += count < kCountMask;
  }

  inline uint32_t getCount() {
    return p & kCountMask;
  }

  inline void setP(uint32_t new_p, uint32_t count = kInitialCount << 5) {
    p = new_p << (31 - shift) | count;
  }

  inline uint32_t getP() const {
    int ret = p >> (31 - shift);
    return ret;
  }
};

// Bit probability model (should be rather fast).
template <typename T, const uint32_t _shift, const uint32_t _learn_rate = 5, const uint32_t _bits = 15>
class fastBitModel {
protected:
  T p;
  static const T pmax = (1 << _bits) - 1;
public:
  static const uint32_t shift = _shift;
  static const uint32_t learn_rate = _learn_rate;
  static const uint32_t max = 1 << shift;

  inline void init(int new_p = 1u << (_shift - 1)) {
    p = new_p << (_bits - shift);
  }
  inline fastBitModel() {
    init();
  }
  inline void update(T bit) {
    update(bit, learn_rate);
  }
  inline void update(T bit, int32_t learn_rate, int32_t round = 0) {
    p += ((static_cast<int>(bit) << _bits) - static_cast<int>(p) + round) >> learn_rate;
  }
  inline void setP(uint32_t new_p) {
    p = new_p << (_bits - shift);
  }
  inline uint32_t getP() const {
    return p >> (_bits - shift);
  }
};

#endif
