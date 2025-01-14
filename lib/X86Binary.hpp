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

#ifndef _X86_BINARY_HPP_
#define _X86_BINARY_HPP_

#include <algorithm>   // for copy
#include <iostream>    // for operator<<, endl, basic_ostream<>::__ostream_type

#include <cstddef>     // for size_t
#include <cstdint>     // for uint8_t, uint32_t, int32_t

#include "Filter.hpp"  // for ByteBufferFilter, ByteStreamFilter
#include "Util.hpp"    // for KB, check

class Stream;

// Encoding format:
// E8/E9 FF/00 XX XX XX
// E8/E8 B2 <char> -> E8/E9 <char>
// E8/E9 <other> -> E8/E9 <other>
class X86AdvancedFilter : public ByteStreamFilter<16 * KB, 20 * KB> {
  static const uint8_t kXORByte = 162;
  static const uint8_t kMarkerByte = 99;
public:
  X86AdvancedFilter(Stream* stream)
    : ByteStreamFilter(stream), offset_(17), last_offset_(17)
    , opt_var_(0), transform_count_(0), false_positives_(0) {

  }
  virtual void forwardFilter(uint8_t* out, size_t* out_count, uint8_t* in, size_t* in_count) {
    process<true>(out, out_count, in, in_count);
  }
  virtual void reverseFilter(uint8_t* out, size_t* out_count, uint8_t* in, size_t* in_count) {
    process<false>(out, out_count, in, in_count);
  }
  static uint32_t getMaxExpansion() {
    return 1;
  }
  void dumpInfo() const {
    std::cout << std::endl << "E8E9: " << transform_count_ << " / " << false_positives_ << std::endl;
  }
  void setOpt(uint64_t s) {
    opt_var_ = s;
  }

private:
  template <bool encode>
  void process(uint8_t* out, size_t* out_count, uint8_t* in, size_t* in_count) {
    const size_t in_c = *in_count;
    const size_t out_c = *out_count;
    uint8_t*start_out = out, *out_limit = out + out_c;
    uint8_t *start_in = in, *in_limit = in + in_c;
    if (in_c <= 6) {
      check(out_c >= in_c);
      std::copy(in, in + in_c, out);
      in += in_c;
      out += in_c;
    } else {
      for (;in < in_limit - 6 && out < out_limit - 6; ++in) {
        *out++ = *in;
        if ((*in & 0xFE) == 0xE8 || ((*in & 0xF0) == 0x80 && in > start_in && in[-1] == 0x0F)) {
          const size_t cur_offset = offset_ + (encode ? (in - start_in) : (out - 1 - start_out));
          if (false && (in[4] == 0 || in[4] == 0xFF) && cur_offset - last_offset_ > 4 * KB) {
            last_offset_ = cur_offset;
            *out++ = in[1];
            *out++ = in[2];
            *out++ = in[3];
            *out++ = in[4];
            in += 4;
            continue;
          }
          if (encode) {
            uint8_t sign_byte = in[4];
            if (sign_byte == 0xFF || sign_byte == 0x00) {
              int32_t delta =
                static_cast<uint32_t>(in[1]) +
                (static_cast<uint32_t>(in[2]) << 8) +
                (static_cast<uint32_t>(in[3]) << 16) +
                (static_cast<uint32_t>(sign_byte) << 24);
              // Don't change 0 deltas.
              if (delta > 0 || (delta < 0 && -delta < static_cast<int32_t>(cur_offset))) {
                if (cur_offset - last_offset_ > 3 * KB * 32) {
                  offset_ = 0;
                }
                uint32_t addr = delta + static_cast<uint32_t>(cur_offset);
                *out++ = sign_byte;
                *out++ = (addr >> 16) ^ kXORByte;
                *out++ = (addr >> 8) ^ kXORByte;
                *out++ = (addr >> 0) ^ kXORByte;
                *out++ = kMarkerByte;  // Marker byte for CM models (signals end of jump).
                in += 4;
                ++transform_count_;
                last_offset_ = cur_offset;
                continue;
              }
            }
            // Forbidden chars / non match.
            if (in[1] == 0xFF || in[1] == 0x00 || in[1] == 0xB2) {
              ++false_positives_;
              *out++ = 0xB2;
              *out++ = in[1];
              in++;
            }
          } else {
            auto sign_byte = in[1];
            if (sign_byte == 0xFF || sign_byte == 0x00) {
              if (cur_offset - last_offset_ > 3 * KB * 32) {
                offset_ = 0;
              }
              uint32_t delta =
                static_cast<uint32_t>(in[4] ^ kXORByte) +
                (static_cast<uint32_t>(in[3] ^ kXORByte) << 8) +
                (static_cast<uint32_t>(in[2] ^ kXORByte) << 16);
              uint32_t addr = delta - static_cast<uint32_t>(cur_offset);
              *out++ = (addr >> 0);
              *out++ = (addr >> 8);
              *out++ = (addr >> 16);
              *out++ = sign_byte;
              last_offset_ = cur_offset;
              in += 5;
            } else if (in[1] == 0xB2) {
              *out++ = in[2];
              in += 2;
            }
          }
        }
      }
    }
    *out_count = out - start_out;
    *in_count = in - start_in;
    offset_ += encode ? *in_count : *out_count;
  }

  size_t offset_;
  size_t last_offset_;
  uint64_t opt_var_;

  size_t transform_count_;
  size_t false_positives_;
};

#endif
