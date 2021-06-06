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

#ifndef _UTIL_HPP_
#define _UTIL_HPP_

#include <algorithm>    // for min
#include <fstream>      // for operator>>
#include <iostream>     // for operator<<, basic_ostream, endl, ostream, cerr
#include <mutex>        // for mutex
#include <sstream>      // for ostringstream
#include <string>       // for string, char_traits, operator<<
#include <utility>      // for pair
#include <vector>       // for vector

#include <cassert>      // for assert
#include <climits>      // for CHAR_BIT
#include <cstddef>      // for size_t
#include <cstdint>      // for uint32_t, uint8_t, uint64_t, int64_t

#include <xmmintrin.h>  // for __m128, _mm_loadu_ps, _mm_storeu_ps

#define OVERRIDE

#ifdef WIN32
#define ALWAYS_INLINE __forceinline
#define NO_INLINE __declspec(noinline)
#else
#define ALWAYS_INLINE inline __attribute__((always_inline))
#define NO_INLINE __attribute__((noinline))
#endif

// TODO: Implement these.
#define LIKELY(x) x
#define UNLIKELY(x) x

#ifdef _DEBUG
static const bool kIsDebugBuild = true;
#else
static const bool kIsDebugBuild = false;
#endif

#ifdef _MSC_VER
#define ASSUME(x) __assume(x)
#else
#define ASSUME(x) __builtin_assume(x)
#endif

typedef uint32_t hash_t;

static const uint64_t KB = 1024;
static const uint64_t MB = KB * KB;
static const uint64_t GB = KB * MB;
static const uint32_t kCacheLineSize = 64; // Sandy bridge.
static const uint32_t kPageSize = 4 * KB;
static const uint32_t kBitsPerByte = 8;

// Used by CM.hpp, MatchModel.hpp 
ALWAYS_INLINE void Prefetch(const void* ptr) {
#ifdef WIN32
  _mm_prefetch((char*)ptr, _MM_HINT_T0);
#else
  __builtin_prefetch(ptr);
#endif
}

// Used by IsWordChar, MakeLowerCase, WordCounter.hpp
ALWAYS_INLINE static bool IsUpperCase(int c) {
  return c >= 'A' && c <= 'Z';
}

// Used by IsWordChar, MakeUpperCase, WordCounter.hpp
ALWAYS_INLINE static bool IsLowerCase(int c) {
  return c >= 'a' && c <= 'z';
}

// Used by Detector.hpp, Dict.hpp
ALWAYS_INLINE static bool IsWordChar(int c) {
  return IsLowerCase(c) || IsUpperCase(c) || c >= 128;
}

// Used by Dict.hpp, WordCounter.hpp
ALWAYS_INLINE static int MakeUpperCase(int c) {
  if (IsLowerCase(c)) {
    c = c - 'a' + 'A';
  }
  return c;
}

// Used by Dict.hpp, WordModel.hpp
ALWAYS_INLINE static int MakeLowerCase(int c) {
  if (IsUpperCase(c)) {
    c = c - 'A' + 'a';
  }
  return c;
}

// Used by WordModel.hpp, MatchFinder.hpp, CM.hpp, LZ.hpp 
// https://blog.regehr.org/archives/1063
ALWAYS_INLINE uint32_t rotate_left(uint32_t h, uint32_t bits) {
  assert(bits < sizeof(h) * char_bit);
  return (h << bits) | (h >> (-bits & (sizeof(h) * CHAR_BIT - 1)));
}

// Used by Compressor.cpp, LZ-inl.hpp
ALWAYS_INLINE void copy16bytes(uint8_t* __restrict out, const uint8_t* __restrict in) {
  _mm_storeu_ps(reinterpret_cast<float*>(out), _mm_loadu_ps(reinterpret_cast<const float*>(in)));
}

// Used by many
#define check(c) while (!(c)) { std::cerr << "check failed " << #c << std::endl; *reinterpret_cast<int*>(1234) = 4321;}
#define dcheck(c) assert(c)

// Used by many
std::string prettySize(uint64_t size);
std::string formatNumber(uint64_t n);
std::string errstr(int err);
uint64_t computeRate(uint64_t size, uint64_t delta_time);

// Used by many
// TODO<C++17>: Replace with std::clamp
static inline int Clamp(int a, int min, int max) {
  if (a < min) a = min;
  if (a > max) a = max;
  return a;
}

// Used by RoundUp
static inline const size_t RoundDown(size_t n, size_t r) {
  return n - n % r;
}

// Used by LZ.hpp, WordCounter.hpp
static inline const size_t RoundUp(size_t n, size_t r) {
  return RoundDown(n + r - 1, r);
}

// Used by Tests.cpp
void RunUtilTests();

// Used by Archive.cpp, Util.cpp
// TODO<C++17>: Use std::filesystem
bool IsAbsolutePath(const std::string& path);

// Used by Archive.cpp, bin/mcm.cpp
template <typename T>
std::vector<T> ReadCSI(const std::string& file) {
  std::ifstream fin(file.c_str());
  std::vector<T> ret;
  for (;;) {
    T temp;
    if (!(fin >> temp)) break;
    char separator;
    fin >> separator;
    if (separator != ',') break;
    ret.push_back(temp);
  }
  return ret;
}

// Used by JPEG.hpp, Wav16.hpp
static inline constexpr uint32_t MakeWord(uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
  return (a << 24) | (b << 16) | (c << 8) | (d << 0);
}

// Used by CyclicBuffer.hpp, Memory.hpp, Wav16.hpp
enum Endian {
  kEndianLittle,
  kEndianBig,
};

// Used by JPEG.hpp, Wav16.hpp, Detector.hpp
struct OffsetBlock {
  size_t offset;
  size_t len;
};

// Used by many
template <uint32_t kAlphabetSize = 0x100>
class FrequencyCounter {
  uint64_t frequencies_[kAlphabetSize] = {};
public:
  ALWAYS_INLINE void Add(uint32_t index, uint64_t count = 1) {
    frequencies_[index] += count;
  }

  template <typename T>
  ALWAYS_INLINE void AddRegion(const T* in, size_t count) {
    for (size_t i = 0; i < count; ++i) {
      Add(in[i], 1);
    }
  }

  ALWAYS_INLINE void Remove(uint32_t index, uint64_t count = 1) {
    dcheck(frequencies_[index] >= count);
    frequencies_[index] -= count;
  }

  uint64_t Sum() const {
    uint64_t ret = 0;
    for (uint64_t c : frequencies_) {
      ret += c;
    }
    return ret;
  }

  void Normalize(uint32_t target) {
    check(target != 0U);
    uint64_t total = 0;
    for (auto f : frequencies_) {
      total += f;
    }
    const auto factor = static_cast<double>(target) / static_cast<double>(total);
    for (auto& f : frequencies_) {
      auto new_val = static_cast<uint32_t>(double(f) * factor);
      total += new_val - f;
      f = new_val;
    }
    // Fudge the probabilities until we match.
    int64_t delta = static_cast<int64_t>(target) - total;
    while (delta) {
      for (auto& f : frequencies_) {
        if (f) {
          if (delta > 0) {
            ++f;
            delta--;
          } else {
            // Don't ever go back down to 0 since we can't necessarily represent that.
            if (f > 1) {
              --f;
              delta++;
            }
          }
        }
      }
    }
  }

  const uint64_t* GetFrequencies() const {
    return frequencies_;
  }
};

#endif
