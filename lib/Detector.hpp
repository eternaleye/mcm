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

#ifndef _DETECTOR_HPP_
#define _DETECTOR_HPP_

#include <algorithm>              // for min
#include <array>                  // for array
#include <deque>                  // for deque
#include <iostream>               // for operator<<, basic_ostream, endl, basic_o...
#include <memory>                 // for allocator_traits<>::value_type
#include <string>                 // for operator<<, char_traits, string
#include <utility>                // for pair
#include <vector>                 // for vector

#include <cassert>                // for assert
#include <cctype>                 // for isdigit, isspace
#include <cstdint>                // for uint8_t, uint64_t, uint32_t
#include <cstdio>                 // for size_t, EOF

#include "compressors/Wav16.hpp"  // for Wav16

#include "CyclicBuffer.hpp"       // for CyclicDeque, Window
#include "Dict.hpp"               // for Dict, Dict::Builder
#include "Stream.hpp"             // for BufferedStreamWriter, Stream
#include "UTF8.hpp"               // for UTF8Decoder
#include "Util.hpp"               // for formatNumber, OffsetBlock

// Detects blocks and data type from input data
class Detector {
public:
  // Pre-detected.
  enum Profile {
    kProfileText,
    kProfileBinary,
    kProfileWave16,
    kProfileSimple,
    kProfileSkip,  // SKip this block, hopefully due to dedupe, or maybe zero pad.
    kProfileEOF,
    kProfileCount,
    // Not a real profile, tells CM to use streaming detection.
    kProfileDetect,
  };

  static std::string profileToString(Profile profile) {
    switch (profile) {
    case Detector::kProfileBinary: return "binary";
    case Detector::kProfileText: return "text";
    case Detector::kProfileWave16: return "wav16";
    }
    return "unknown";
  }

  class DetectedBlock {
  public:
    DetectedBlock(Profile profile = kProfileBinary, uint32_t length = 0)
      : profile_(profile), length_(length) {
    }
    DetectedBlock(const DetectedBlock& other) {
      *this = other;
    }
    DetectedBlock& operator=(const DetectedBlock& other) {
      profile_ = other.profile_;
      length_ = other.length_;
      return *this;
    }

    static size_t calculateLengthBytes(size_t length) {
      if (length & 0xFF000000) return 4;
      if (length & 0xFF0000) return 3;
      if (length & 0xFF00) return 2;
      return 1;
    }
    static size_t getSizeFromHeaderByte(uint8_t b) {
      return 1 + getLengthBytes(b);
    }
    static size_t getLengthBytes(uint8_t b) {
      return (b >> kLengthBytesShift) + 1;
    }
    size_t write(uint8_t* ptr) {
      const auto* orig_ptr = ptr;
      size_t enc_len = length_ - 1;
      const auto length_bytes = calculateLengthBytes(enc_len);
      *(ptr++) = static_cast<uint8_t>(profile_) | static_cast<uint8_t>((length_bytes - 1) << kLengthBytesShift);
      for (size_t i = 0; i < length_bytes; ++i) {
        *(ptr++) = static_cast<uint8_t>(enc_len >> (i * 8));
      }
      return ptr - orig_ptr;
    }
    size_t read(const uint8_t* ptr) {
      const auto* orig_ptr = ptr;
      auto c = *(ptr++);
      profile_ = static_cast<Profile>(c & kDataProfileMask);
      auto length_bytes = getLengthBytes(c);
      length_ = 0;
      for (size_t i = 0; i < length_bytes; ++i) {
        length_ |= static_cast<uint32_t>(*(ptr++)) << (i * 8);
      }
      ++length_;
      return ptr - orig_ptr;
    }
    Profile profile() const {
      return profile_;
    }
    uint64_t length() const {
      return length_;
    }
    void setLength(uint64_t length) {
      length_ = length;
    }
    void extend(uint64_t len) {
      length_ += len;
    }
    // Remove one character from length.
    void pop(uint64_t count = 1) {
      assert(length_ >= count);
      length_ -= count;
    }

  private:
    static const size_t kLengthBytesShift = 6;
    static const size_t kDataProfileMask = (1u << kLengthBytesShift) - 1;
    Profile profile_;
    uint64_t length_;
  };
};

// Detects blocks and data type from input data
class InDetector {
  bool is_forbidden[256]; // Chars which don't appear in text often.
  bool is_word_or_ascii_art[256];
  uint8_t is_space[256];

  // MZ pattern, todo replace with better detection.
  typedef std::vector<uint8_t> Pattern;
  Pattern exe_pattern;

  // Lookahed.
  using BufferType = CyclicDeque<uint8_t>;
  BufferType buffer_;

  // Out buffer, only used to store headers (for now).
  std::array<uint8_t, 16 * KB> out_buffer_;
  size_t out_buffer_pos_, out_buffer_size_;

  // Read / write stream.
  InStream* stream_;

  // Opt var
  uint64_t opt_var_;
public:

  // std::vector<DetectedBlock> detected_blocks_;
  Detector::DetectedBlock current_block_;

  // Detected but not already read.
  Detector::DetectedBlock detected_block_;

  // Saved detected blocks.
  std::deque<Detector::DetectedBlock> saved_blocks_;

  // Statistics
  uint64_t num_blocks_[Detector::kProfileCount];
  uint64_t num_bytes_[Detector::kProfileCount];
  uint64_t overhead_bytes_;
  uint64_t small_len_;

  // Spaces.
  size_t no_spaces_;

  // Last things.
  uint32_t last_word_;
public:

  InDetector(InStream* stream) : stream_(stream), opt_var_(0), last_word_(0) {
  }

  void setOptVar(uint64_t var) {
    opt_var_ = var;
  }

  void init() {
    overhead_bytes_ = 0;
    small_len_ = 0;
    for (size_t i = 0; i < Detector::kProfileCount; ++i) {
      num_blocks_[i] = num_bytes_[i] = 0;
    }
    out_buffer_pos_ = out_buffer_size_ = 0;
    for (auto& b : is_forbidden) b = false;

    const uint8_t forbidden_arr[] = {
      0, 1, 2, 3, 4,
      5, 6, 7, 8, 11,
      12, 14, 15, 16, 17,
      19, 20, 21, 22, 23,
      24, 25, 26, 27, 28,
      29, 30, 31
    };
    for (auto c : forbidden_arr) is_forbidden[c] = true;
    for (size_t i = 0; i < 256; ++i) {
      is_space[i] = isspace(i) ? 1u : 0u;
      is_word_or_ascii_art[i] = IsWordOrAsciiArtChar(i);
    }
    no_spaces_ = 0;

    buffer_.Resize(256 * KB);
    // Exe pattern
    uint8_t p[] = { 0x4D, 0x5A, 0x90, 0x00, 0x03, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0xFF, 0xFF, };
    exe_pattern.clear();
    for (auto& c : p) exe_pattern.push_back(c);
  }

  void RefillRead() {
    const size_t kBufferSize = 8 * KB;
    uint8_t buffer[kBufferSize];
    for (;;) {
      const size_t remain = buffer_.Remain();
      const size_t n = stream_->read(buffer, std::min(kBufferSize, remain));
      if (n == 0 || remain == 0) break;
      buffer_.PushBackCount(buffer, n);
    }
  }

  inline bool empty() const {
    return size() == 0;
  }

  inline size_t size() const {
    return buffer_.Size();
  }

  Detector::Profile detect() {
    if (current_block_.length() > 0) {
      return current_block_.profile();
    }
    if (current_block_.profile() == Detector::kProfileEOF) {
      return Detector::kProfileEOF;
    }
    return Detector::kProfileBinary;
  }

  inline uint32_t at(uint32_t index) const {
    assert(index < buffer_.Size());
    return buffer_[index];
  }

  int get(Detector::Profile& profile) {
    // Profile can't extend past the end of the buffer.
    if (false && current_block_.length() == 0) {
      current_block_ = detectBlock();
    }
    if (current_block_.length() > 0) {
      profile = current_block_.profile();
      return readChar();
    }
    // Still have some header to read?
    if (out_buffer_pos_ < out_buffer_size_) {
      if (++out_buffer_pos_ == out_buffer_size_) {
        current_block_ = detected_block_;
      }
      overhead_bytes_ += out_buffer_size_;
      profile = Detector::kProfileBinary;
      return out_buffer_[out_buffer_pos_ - 1];
    }
    if (current_block_.profile() == Detector::kProfileEOF) {
      profile = Detector::kProfileEOF;
      return EOF;
    }
    detected_block_ = detectBlock();
    ++num_blocks_[detected_block_.profile()];
    num_bytes_[detected_block_.profile()] += detected_block_.length();
    if (detected_block_.length() < 64) ++small_len_;
    out_buffer_size_ = detected_block_.write(&out_buffer_[0]);
    profile = Detector::kProfileBinary;
    out_buffer_pos_ = 1;
    return out_buffer_[0];
  }

  // Read char without detection.
  uint8_t readChar() {
    current_block_.pop();
    return popChar();
  }
  int popChar() {
    if (buffer_.Empty()) {
      RefillRead();
      if (buffer_.Empty()) {
        return EOF;
      }
    }
    auto ret = buffer_.Front();
    buffer_.PopFront();
    return ret;
  }
  size_t read(uint8_t* out, size_t count) {
    const auto n = std::min(count, buffer_.Size());
    for (size_t i = 0; i < n; ++i) {
      out[i] = buffer_[i];
    }
    buffer_.PopFront(n);
    current_block_.pop(n);
    return n;
  }

  void dumpInfo() {
    std::cout << "Detector overhead " << formatNumber(overhead_bytes_) << " small=" << small_len_ << std::endl;
    for (size_t i = 0; i < Detector::kProfileCount; ++i) {
      std::cout << Detector::profileToString(static_cast<Detector::Profile>(i)) << "("
        << formatNumber(num_blocks_[i]) << ") : " << formatNumber(num_bytes_[i]) << std::endl;
    }
  }

  static bool IsWordOrAsciiArtChar(uint8_t c) {
    return IsWordChar(c) || c == '|' || c == '_' || c == '-';
  }

  Detector::DetectedBlock detectBlock() {
    if (!saved_blocks_.empty()) {
      auto ret = saved_blocks_.front();
      saved_blocks_.pop_front();
      return ret;
    }
    RefillRead();
    const size_t buffer_size = buffer_.Size();
    if (buffer_size == 0) {
      return Detector::DetectedBlock(Detector::kProfileEOF, 0);
    }

    size_t binary_len = 0;
    while (binary_len < buffer_size) {
      UTF8Decoder<true> decoder;
      size_t text_len = 0;
      size_t space_count = 0;
      size_t word_chars = 0;
      size_t word_len = 0;
      size_t number_len = 0;
      int text_score = 0;
      while (binary_len + text_len < buffer_size) {
        const size_t pos = binary_len + text_len;
        Window<BufferType> window(buffer_, static_cast<uint32_t>(pos));
        OffsetBlock b;
        if (Wav16::Detect(last_word_, window, &b)) {
          saved_blocks_.push_back(Detector::DetectedBlock(Detector::kProfileWave16, b.len));
          return Detector::DetectedBlock(Detector::kProfileBinary, b.offset);
        }
        const uint8_t c = buffer_[pos];
        last_word_ = (last_word_ << 8) | c;
        decoder.update(c);
        if (decoder.err() || is_forbidden[c]) {
          break; // Error state?
        }
        ++text_len;
        const uint8_t last_c = (last_word_ >> 8) & 0xFF;
        text_score += is_space[c];
        if (last_c != c) {
          if (is_word_or_ascii_art[c]) {
            ++word_len;
            text_score += is_space[last_c] * 10;
          } else if (word_len != 0) {
            if (word_len >= 3 && word_len < 32) {
              text_score += word_len * 3;
            }
            text_score += is_space[c] * 10;
            word_len = 0;
          }
          if (c >= '0' && c <= '9') {
            ++number_len;
          } else {
            number_len = 0;
          }
          text_score += number_len >= 1 && number_len <= 12;
          space_count += is_space[c];
        } else {
          if (!is_space[c] && !isdigit(c)) {
            // Only expect adjacent spaces really.
            --text_score;
          }
        }
      }
      if (text_len > 64) {
        uint8_t buf[512];
        char* bptr = reinterpret_cast<char*>(buf);
        for (size_t i = 0; i < sizeof(buf) && i < text_len; ++i) {
          buf[i] = buffer_[binary_len + i];
        }
        if (space_count * 100 > text_len && text_score > static_cast<int>(text_len)) {
          if (binary_len == 0) {
            return Detector::DetectedBlock(Detector::kProfileText, static_cast<uint32_t>(text_len));
          } else {
            break;
          }
        }
      } 
      binary_len += text_len;
      if (binary_len >= buffer_size) {
        break;
      }
      ++binary_len;
    }
    return Detector::DetectedBlock(Detector::kProfileBinary, static_cast<uint32_t>(binary_len));
  }
};

// Detects blocks and data type from input data
class OutDetector {
  bool is_forbidden[256]; // Chars which don't appear in text often.
  bool is_word_or_ascii_art[256];
  uint8_t is_space[256];

  // MZ pattern, todo replace with better detection.
  typedef std::vector<uint8_t> Pattern;
  Pattern exe_pattern;

  // Lookahed.
  using BufferType = CyclicDeque<uint8_t>;
  BufferType buffer_;

  // Out buffer, only used to store headers (for now).
  std::array<uint8_t, 16 * KB> out_buffer_;
  size_t out_buffer_pos_, out_buffer_size_;

  // Read / write stream.
  OutStream* stream_;

  // Opt var
  uint64_t opt_var_;
public:

  // std::vector<DetectedBlock> detected_blocks_;
  Detector::DetectedBlock current_block_;

  // Detected but not already read.
  Detector::DetectedBlock detected_block_;

  // Saved detected blocks.
  std::deque<Detector::DetectedBlock> saved_blocks_;

  // Statistics
  uint64_t num_blocks_[Detector::kProfileCount];
  uint64_t num_bytes_[Detector::kProfileCount];
  uint64_t overhead_bytes_;
  uint64_t small_len_;

  // Spaces.
  size_t no_spaces_;

  // Last things.
  uint32_t last_word_;
public:

  OutDetector(OutStream* stream) : stream_(stream), opt_var_(0), last_word_(0) {
  }

  void setOptVar(uint64_t var) {
    opt_var_ = var;
  }

  void init() {
    overhead_bytes_ = 0;
    small_len_ = 0;
    for (size_t i = 0; i < Detector::kProfileCount; ++i) {
      num_blocks_[i] = num_bytes_[i] = 0;
    }
    out_buffer_pos_ = out_buffer_size_ = 0;
    for (auto& b : is_forbidden) b = false;

    const uint8_t forbidden_arr[] = {
      0, 1, 2, 3, 4,
      5, 6, 7, 8, 11,
      12, 14, 15, 16, 17,
      19, 20, 21, 22, 23,
      24, 25, 26, 27, 28,
      29, 30, 31
    };
    for (auto c : forbidden_arr) is_forbidden[c] = true;
    for (size_t i = 0; i < 256; ++i) {
      is_space[i] = isspace(i) ? 1u : 0u;
      is_word_or_ascii_art[i] = IsWordOrAsciiArtChar(i);
    }
    no_spaces_ = 0;

    buffer_.Resize(256 * KB);
    // Exe pattern
    uint8_t p[] = { 0x4D, 0x5A, 0x90, 0x00, 0x03, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0xFF, 0xFF, };
    exe_pattern.clear();
    for (auto& c : p) exe_pattern.push_back(c);
  }

  inline bool empty() const {
    return size() == 0;
  }

  inline size_t size() const {
    return buffer_.Size();
  }

  void put(int c) {
    // Profile can't extend past the end of the buffer.
    if (current_block_.length() > 0) {
      current_block_.pop();
      if (buffer_.Full()) {
        flush();
      }
      buffer_.PushBack(c);
    } else {
      out_buffer_[out_buffer_pos_++] = static_cast<uint8_t>(c);
      auto num_bytes = Detector::DetectedBlock::getSizeFromHeaderByte(out_buffer_[0]);
      if (out_buffer_pos_ == num_bytes) {
        current_block_.read(&out_buffer_[0]);
        if (current_block_.profile() == Detector::kProfileEOF) {
          out_buffer_pos_ = 0;
        }
        out_buffer_pos_ = 0;
      }
    }
  }

  Detector::Profile detect() {
    if (current_block_.length() > 0) {
      return current_block_.profile();
    }
    if (current_block_.profile() == Detector::kProfileEOF) {
      return Detector::kProfileEOF;
    }
    return Detector::kProfileBinary;
  }

  void flush() {
    // TODO: Optimize
    BufferedStreamWriter<4 * KB> sout(stream_);
    while (!buffer_.Empty()) {
      sout.put(buffer_.Front());
      buffer_.PopFront();
    }
    sout.flush();
  }

  inline uint32_t at(uint32_t index) const {
    assert(index < buffer_.Size());
    return buffer_[index];
  }

  void dumpInfo() {
    std::cout << "Detector overhead " << formatNumber(overhead_bytes_) << " small=" << small_len_ << std::endl;
    for (size_t i = 0; i < Detector::kProfileCount; ++i) {
      std::cout << Detector::profileToString(static_cast<Detector::Profile>(i)) << "("
        << formatNumber(num_blocks_[i]) << ") : " << formatNumber(num_bytes_[i]) << std::endl;
    }
  }

  static bool IsWordOrAsciiArtChar(uint8_t c) {
    return IsWordChar(c) || c == '|' || c == '_' || c == '-';
  }

};

// Detector analyzer, analyze a whole stream.
class Analyzer {
public:
  typedef std::vector<Detector::DetectedBlock> Blocks;

  void analyze(InStream* stream, size_t file_idx = 0) {
    InDetector detector(stream);
    detector.setOptVar(opt_var_);
    detector.init();
    for (;;) {
    next_block:
      auto block = detector.detectBlock();
      if (block.profile() == Detector::kProfileEOF) {
        break;
      }
      for (size_t i = 0; i < block.length(); ++i) {
        auto c = detector.popChar();
        if (c == EOF) {
          block.setLength(i);
          break;
        }
        if (block.profile() == Detector::kProfileText) {
          dict_builder_.AddChar(c);
        }
      }
      const size_t size = blocks_.size();
      if (size > 0 && blocks_.back().profile() == block.profile()) {
        // Same type, extend.
        blocks_.back().extend(block.length());
      } else {
        const size_t min_binary_length = 1;
        // replace <text> <bin> <text> with <text> if |<bin>| < min_binary_length.
        if (block.profile() == Detector::kProfileText && size >= 2) {
          auto& b1 = blocks_[size - 1];
          auto& b2 = blocks_[size - 2];
          if (b1.profile() == Detector::kProfileBinary &&
            b2.profile() == Detector::kProfileText &&
            b1.length() < min_binary_length) {
            b2.extend(b1.length() + block.length());
            blocks_.pop_back();
            continue;
          }
        }
        blocks_.push_back(block);
      }
    }
  }
  void dump() {
    uint64_t blocks[Detector::kProfileCount] = { 0 };
    uint64_t bytes[Detector::kProfileCount] = { 0 };
    for (auto& b : blocks_) {
      ++blocks[b.profile()];
      bytes[b.profile()] += b.length();
    }
    for (size_t i = 0; i < Detector::kProfileCount; ++i) {
      if (bytes[i] > 0) {
        std::cout << Detector::profileToString(static_cast<Detector::Profile>(i))
          << " : " << blocks[i] << "(" << prettySize(bytes[i]) << ")" << std::endl;
      }
    }
  }
  Blocks& getBlocks() {
    return blocks_;
  }
  Dict::Builder& getDictBuilder() {
    return dict_builder_;
  }
  void setOpt(size_t opt_var) {
    opt_var_ = opt_var;
  }
  Analyzer() : opt_var_(0) {}

private:
  Blocks blocks_;
  Dict::Builder dict_builder_;
  uint64_t opt_var_;
};

#endif
