/*	MCM file compressor

  Copyright (C) 2016, Google Inc.
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

#ifndef _WORD_MODEL_HPP_
#define _WORD_MODEL_HPP_

#include <cstddef>      // for size_t
#include <cstdint>      // for uint32_t, uint8_t, uint64_t

#include "Reorder.hpp"  // for ReorderMap
#include "UTF8.hpp"     // for UTF8Decoder
#include "Util.hpp"     // for MakeLowerCase, rotate_left

class WordModel {
public:
  // Hashes.
  uint64_t prev;
  uint64_t h1, h2;

  // UTF decoder.
  UTF8Decoder<false> decoder;

  // Length of the model.
  size_t len;
  static const size_t kMaxLen = 31;

  // Transform table.
  static const uint32_t transform_table_size = 256;
  uint32_t transform[transform_table_size];

  uint64_t opt_var_ = 0;
  size_t* opts_ = nullptr;

  void setOpt(uint64_t n) {
    opt_var_ = n;
  }

  void SetOpts(size_t* opts) {
    opts_ = opts;
  }

  inline uint32_t& trans(char c) {
    uint32_t index = (uint32_t)(uint8_t)c;
    check(index < transform_table_size);
    return transform[index];
  }

  WordModel() {}

  void Init(const ReorderMap<uint8_t, 256>& reorder) {
    uint32_t index = 0;
    for (auto& t : transform) t = transform_table_size;
    for (uint32_t i = 'a'; i <= 'z'; ++i) {
      transform[reorder[i]] = index++;
    }
    for (uint32_t i = 'A'; i <= 'Z'; ++i) {
      transform[reorder[i]] = transform[reorder[MakeLowerCase(static_cast<int>(i))]];
    }
    // 6,38,92,3
    trans(reorder[6]) = index++;
    trans(reorder[38]) = index++;
    trans(reorder[92]) = index++;
    trans(reorder[3]) = index++;
    for (size_t i = 128; i < 256; ++i) {
      if (transform[reorder[i]] == transform_table_size) {
        transform[reorder[i]] = index++;
      }
    }

    len = 0;
    prev = 0;
    reset();
    decoder.init();
  }

  inline void reset() {
    h1 = 0x1F20239A;
    h2 = 0xBE5FD47A;
    len = 0;
  }

  inline uint32_t getHash() const {
    auto ret = (h1 * 15) ^ (h2 * 41);
    ret ^= ret >> 4;
    return ret;
  }

  inline uint32_t getPrevHash() const {
    return prev;
  }

  inline uint32_t getMixedHash() const {
    auto ret = getHash();
    if (len < 2) {
      ret ^= getPrevHash();
    }
    return ret;
  }

  inline uint32_t get01Hash() const {
    return getHash() ^ getPrevHash();
  }

  inline size_t getLength() const {
    return len;
  }

  // Return true if just finished word.
  bool update(uint8_t c) {
    const auto cur = transform[c];
    if (cur != transform_table_size) {
      h1 = HashFunc(cur, h1);
      h2 = h1 * 24;
      len += len < 16;
    } else if (len) {
      prev = rotate_left(getHash() * 21, 14);
      reset();
      return true;
    }
    return false;
  }

  inline uint64_t HashFunc(uint64_t c, uint64_t h) {
    /*
    h ^= (h * (1 + 24 * 1)) >> 24;
    // h *= 61;
    h += c;
    h += rotate_left(h, 12);
    return h ^ (h >> 8);
    */
    return (h * 43) + c;
  }
};

class DictXMLModel : public WordModel {
  
  // 40 82 6
public:
  void Init(const ReorderMap<uint8_t, 256>& reorder) {
    last_char_ = 0;
    dict_remain_ = 0;
    escape_ = reorder[0x3];
    upper1_ = reorder[0x4];
    upper2_ = reorder[0x6];
    WordModel::Init(reorder);
  }

  void update(uint8_t c) {
    if (c >= 128) {
      if (last_char_ == escape_) {
        WordModel::update(c);
      } else if (dict_remain_ == 0) {
        if (c < 128 + 40) {
          dict_remain_ = 1;
        } else if (c < 128 + 40 + 82) {
          dict_remain_ = 2;
        } else {
          dict_remain_ = 3;
        }
      } else {
        --dict_remain_;
      }
    } else {
      dict_remain_ = 0;
    }
    last_char_ = c;
    WordModel::update(c);
  }

  uint32_t getMixedHash() {
    auto ret = WordModel::getMixedHash();
    /* if (dict_remain_) {
      ret ^= dict_remain_ * 12931991;
    } */
    return ret;
  }

private:
  size_t dict_remain_;
  uint8_t last_char_;
  uint8_t escape_ = 0;
  uint8_t upper1_ = 0;
  uint8_t upper2_ = 0;
};

#endif
