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

#ifndef _COMPRESS_HPP_
#define _COMPRESS_HPP_

#include <vector>                 // for vector

#include <cstddef>                // for size_t
#include <cstdint>                // for uint8_t, uint64_t, uint32_t

#include <libmcm/Compressor.hpp>  // for Compressor
#include <libmcm/Stream.hpp>      // for Stream (ptr only)

#include "Stream.hpp"             // for WriteMemoryStream, ReadMemoryStream
#include "Util.hpp"               // for MB

enum class CompressorType : int {
  kTypeStore,
  kTypeWav16,
  kTypeCMTurbo,
  kTypeCMFast,
  kTypeCMMid,
  kTypeCMHigh,
  kTypeCMMax,
  kTypeCMSimple,
  kTypeDMC,
};

// In memory compressor.
class MemoryCompressor : public Compressor {
  static const size_t kBufferSize = 32 * MB;
public:
  virtual size_t getMaxExpansion(size_t s) = 0;
  virtual size_t compress(uint8_t* in, uint8_t* out, size_t count) = 0;
  virtual void decompress(uint8_t* in, uint8_t* out, size_t count) = 0;
  // Initial implementations of compress and decompress from memory.
  virtual void compress(InStream* in, OutStream* out, uint64_t max_count = 0xFFFFFFFFFFFFFFFF);
  // Decompress n bytes, the calls must line up.
  virtual void decompress(InStream* in, OutStream* out, uint64_t max_count = 0xFFFFFFFFFFFFFFFF);
};

class Store : public Compressor {
public:
  Store();
  virtual void compress(InStream* in, OutStream* out, uint64_t count);
  virtual void decompress(InStream* in, OutStream* out, uint64_t count);
private:
  uint8_t transform_[256];
  uint8_t reverse_[256];
};

#endif
