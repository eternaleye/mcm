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

#ifndef _DICT_HPP_
#define _DICT_HPP_

#include <algorithm>        // for max, min, sort, copy, count
#include <chrono>           // for high_resolution_clock, duration, duration...
#include <iostream>         // for operator<<, basic_ostream, basic_ostream_...
#include <memory>           // for allocator_traits<>::value_type
#include <string>           // for string, basic_string, hash, operator==
#include <unordered_map>    // for unordered_map, operator!=, _Node_iterator
#include <utility>          // for pair
#include <vector>           // for vector, vector<>::reverse_iterator

#include <cassert>          // for assert
#include <cstddef>          // for size_t
#include <cstdint>          // for uint8_t, uint32_t, int64_t

#include "Filter.hpp"       // for ByteStreamFilter
#include "Stream.hpp"       // for ReadMemoryStream, WriteVectorStream
#include "Util.hpp"         // for FrequencyCounter
#include "WordCounter.hpp"  // for WordCount::CompareLexicographically

enum WordModifier {
  kWordNormal,
  // Toggle case of first char.
  kWordLowerFirstChar,
  // Toggle case of whole word.
  kWordToggleAll,
  kWordModifierCount,
};

class CodeWordMap {
  static const size_t kMapCount = 256;
  bool map_[kMapCount] = {};
public:
  void Add(size_t start, size_t end) {
    for (size_t i = start; i < end; ++i) {
      map_[i] = true;
    }
  }

  bool Get(size_t i) const {
    return map_[i];
  }

  size_t Count() const {
    return std::count(map_, map_ + kMapCount, true);
  }
};

class Dict {
public:
  static const size_t kMinWordLen = 3;
  static const size_t kMaxWordLen = 256;
  static const size_t kInvalidChar = 256;
  static const bool kOverlapCodewords = true;
  typedef std::pair<uint32_t, std::string> WCPair;

  struct ReverseCompareString {
    bool operator()(const std::string& a, const std::string& b) const {
      const size_t count = std::min(a.length(), b.length());
      const char* ptr_a = &a[a.length() - 1];
      const char* ptr_b = &b[b.length() - 1];
      for (size_t i = 0; i < count; ++i) {
        if (*ptr_a != *ptr_b) {
          return *ptr_a < *ptr_b;
        }
        --ptr_a;
        --ptr_b;
      }
      return a.length() < b.length();
    }
  };

  class CodeWord {
  public:
    uint32_t code_ = 0;
    WordCC expected_case_ = kWordCCNone;

    void setCode(uint32_t code) { code_ = code; }
    CodeWord(uint8_t num_bytes = 0u, uint8_t c1 = 0u, uint8_t c2 = 0u, uint8_t c3 = 0u) {
      code_ = num_bytes;
      code_ = (code_ << 8) | c1;
      code_ = (code_ << 8) | c2;
      code_ = (code_ << 8) | c3;
    }
    uint32_t byte1() const { return (code_ >> 16) & 0xFFu; }
    uint32_t byte2() const { return (code_ >> 8) & 0xFFu; }
    uint32_t byte3() const { return (code_ >> 0) & 0xFFu; }
    size_t numBytes() const {
      const auto bytes = code_ >> 24;
      assert(bytes <= 3);
      return bytes;
    }
    WordCC ExpectedCase() {
      return expected_case_;
    }
  };

  // Simple collection of words and their codes.
  class CodeWordSet {
  public:
    // Map from words to their codes.
    size_t num1_;
    size_t num2_;
    size_t num3_;
    std::vector<WordCount> codewords_;

    std::vector<WordCount>* GetCodeWords() {
      return &codewords_;
    }
  };

  class Builder {
    static const size_t kSuffixSize = 100 * MB;
    // Suffix array buffer.
    std::vector<uint8_t> buffer_;
    size_t buffer_pos_;
    // Current word.
    static const size_t kMinWordLen = 3;
    static const size_t kMaxWordLen = 0x20;
    static const size_t kDefaultMinOccurrences = 8;
    uint8_t word_[kMaxWordLen];
    size_t word_pos_;
    // CC: first char EOR whole word.
    WordCounter words_;
    FrequencyCounter<256> counter_;

  public:
    void GetWords(std::vector<WordCount>& out, size_t min_occurences = kDefaultMinOccurrences) {
      words_.GetWords(out, min_occurences);
      words_.Clear();
    }

    FrequencyCounter<256>& GetFrequencies() {
      return counter_;
    }

    void AddChar(uint8_t c) {
      counter_.Add(c);
      // Add to current word.
      if (IsWordChar(c)) {
        if (word_pos_ < kMaxWordLen) {
          word_[word_pos_++] = c;
        }
      } else {
        if (word_pos_ >= kMinWordLen) {
          WordCC cc_type = GetWordCase(word_, word_pos_);
          if (cc_type == kWordCCAll) {
            for (size_t i = 0; i < word_pos_; ++i) {
              word_[i] = MakeLowerCase(word_[i]);
            }
          } else if (cc_type == kWordCCFirstChar) {
            word_[0] = MakeLowerCase(word_[0]);
          } else if (cc_type == kWordCCInvalid && (false)) {
            for (size_t i = 0; i < word_pos_; ++i) {
              word_[i] = MakeLowerCase(word_[i]);
            }
            cc_type = kWordCCNone;
          }
          if (cc_type != kWordCCInvalid) {
            words_.AddWord(word_, word_ + word_pos_, cc_type);
          }
        }
        word_pos_ = 0;
      }
    }
    void init() {
      buffer_pos_ = 0;
      buffer_.reserve(kSuffixSize);
      word_pos_ = 0;
      words_.Init(256 * MB);
    }
    Builder() {
      init();
    }
    const std::vector<uint8_t>* getBuffer() const {
      return &buffer_;
    }
  };

  class CodeWordGeneratorFast {
    static const bool kVerbose = true;
  public:
    void Generate(Builder& builder, CodeWordSet* words, size_t min_occurrences, size_t num_1 = 32, size_t num_2 = 32, size_t num_code_words = 128) {
      auto start_time = std::chrono::high_resolution_clock::now();
      auto* cw = words->GetCodeWords();
      cw->clear();

      std::vector<WordCount> word_pairs;
      builder.GetWords(word_pairs, min_occurrences);
      const auto occurences = word_pairs.size();
      std::sort(word_pairs.rbegin(), word_pairs.rend(), WordCount::CompareSavings(1));

      // Calculate number of 1 byte codewords in case its more than the original max.
      words->num1_ = std::min(static_cast<size_t>(num_1), word_pairs.size());
      while (words->num1_ + 1 < word_pairs.size()) {
        // Remain.
        size_t remain = num_code_words - words->num1_;
        const size_t new_2 = (remain - 1) * (remain - 1);
        if (remain == 0 || new_2 < word_pairs.size() - words->num1_) {
          break;
        }
        ++words->num1_;
      }

      int64_t save1 = 0, save2 = 0, save3 = 0; // Number of bytes saved.
      size_t num1 = words->num1_, num2 = 0, num3 = 0; // Number of first byte which are 1b/2b/3b.
      size_t count1 = 0, count2 = 0, count3 = 0; /// Number of words.
      for (size_t i = 0; i < words->num1_; ++i) {
        const auto& p = word_pairs[i];
        ++count1;
        cw->push_back(p);
        save1 += p.Savings(1);
      }
      std::sort(cw->begin(), cw->end(), WordCount::CompareLexicographically());
      word_pairs.erase(word_pairs.begin(), word_pairs.begin() + count1);
      std::sort(word_pairs.rbegin(), word_pairs.rend(), WordCount::CompareSavings(2));
      // 2 byte codes.
      for (num3 = 0; num3 + num1 < num_code_words - num_2; ++num3) {
        const size_t count3 = num3 * (kOverlapCodewords ? num_code_words * num_code_words : num3 * num3);
        auto num2 = (num_code_words - num3 - num1);
        const size_t count2 = num2 * (kOverlapCodewords ? num_code_words : num2);
        if (count2 + count3 >= word_pairs.size()) break;
      }
      words->num3_ = num3;
      num2 = num_code_words - num1 - num3;
      words->num2_ = num2;
      for (size_t b1 = 0; b1 < num2; ++b1) {
        for (size_t b2 = 0; b2 < (kOverlapCodewords ? num_code_words : num2); ++b2) {
          if (count2 < word_pairs.size()) {
            const auto& p = word_pairs[count2++];
            cw->push_back(p);
            save2 += p.Savings(2);
          }
        }
      }
      word_pairs.erase(word_pairs.begin(), word_pairs.begin() + count2);
      std::sort(word_pairs.rbegin(), word_pairs.rend(), WordCount::CompareSavings(3));

      constexpr bool kPopBads = true;
      if (kPopBads) {
        // Remove dictionary replacements that will actually increase size.
        while (!word_pairs.empty() && word_pairs.back().Savings(3) <= 0) {
          // std::cerr << "POP " << word_pairs.back().word << std::endl;
          word_pairs.pop_back();
        }
      }
      // 3 byte codes.
      for (size_t b1 = 0; b1 < num3; ++b1) {
        for (size_t b2 = 0; b2 < (kOverlapCodewords ? num_code_words : num3); ++b2) {
          for (size_t b3 = 0; b3 < (kOverlapCodewords ? num_code_words : num3); ++b3) {
            if (count3 < word_pairs.size()) {
              const auto& p = word_pairs[count3++];
              cw->push_back(p);
              save3 += p.Savings(3);
            }
          }
        }
      }
      word_pairs.erase(word_pairs.begin(), word_pairs.begin() + count3);
      auto start = count1;
      std::sort(cw->begin() + start, cw->begin() + start + count2, WordCount::CompareLexicographically());
      start += count2;
      std::sort(cw->begin() + start, cw->begin() + start + count3, WordCount::CompareLexicographically());

      if (kVerbose) {
        // Remaining chars.
        int64_t remain = 0;
        for (const auto& p : word_pairs) remain += p.Savings(3);
        auto end_time = std::chrono::high_resolution_clock::now();
        const std::chrono::duration<double, std::ratio<1>> time = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(end_time - start_time);
        std::cout << "Constructed dict words=" << count1 << "+" << count2 << "+" << count3 << "=" << occurences
          << " save=" << save1 << "+" << save2 << "+" << save3 << "=" << save1 + save2 + save3
          << " extra=" << remain
          << " time=" << time.count() << "s"
          << std::endl;
      }
    }
  };

  void SetUpCodeWords(CodeWordMap& codes) {
    codes.Add(128, 255);
  }

  class EncodeMap {
  public:
    struct Entry {
      CodeWord code_word;
      WordCC word_case;
    };

    void Add(const std::string& word, const CodeWord& code_word) {
      std::string lower_case(word);
      auto word_case = GetWordCase(reinterpret_cast<const uint8_t*>(word.c_str()), word.length());
      for (auto& c : lower_case) {
        c = MakeLowerCase(c);
      }
      map_.emplace(lower_case, Entry{ code_word, word_case });
    }

    const Entry* Find(const std::string& word) const {
      auto it = map_.find(word);
      return it != map_.end() ? &it->second : nullptr;
    }

  private:
    std::unordered_map<std::string, Entry> map_;
  };

  // Encodes / decods words / code words.
  class Filter : public ByteStreamFilter<16 * KB, 16 * KB> {
    // Capital conersion.
    size_t escape_char_;
    size_t escape_cap_first_;
    size_t escape_cap_word_;
    // Currrent word
    uint8_t word_[kMaxWordLen];
    size_t word_pos_;
    // Read / write dict buffer.
    std::vector<uint8_t> dict_buffer_;
    size_t dict_buffer_pos_;
    size_t dict_buffer_size_;

    // Encoding data structures.
    EncodeMap encode_map_;

    // State
    uint8_t last_char_;
    bool capital_mode_ = false;

    // Decode data structures
    std::vector<std::string> words1b;
    size_t word1bstart;
    std::vector<std::string> words2b;
    size_t word2bstart;
    std::vector<std::string> words3b;
    size_t word3bstart;

    // Optimizations
    uint8_t is_word_char_[256];

    // Options
    static constexpr bool kOnlyDict = false;
    static const size_t kCodeWordStart = 128u;

    // Stats
    static const bool kStats = true;
    size_t escape_count_ = 0;
    size_t escape_count_word_ = 0;
    size_t escape_count_first_ = 0;

    // Frequencies
    FrequencyCounter<256> freq_;
  public:
    // Serialize to and from.
    // num words
    // escape
    // escape cap first
    // escape cap word
    // 1b count
    // 2b count
    // 3b count

    FrequencyCounter<256> GetFrequencies() {
      return freq_;
    }

    void SetFrequencies(const FrequencyCounter<256>& freq) {
      freq_ = freq;
    }

    // Creates an encodable dictionary array.
    void AddCodeWords(std::vector<WordCount>* words,
                      uint8_t num1,
                      uint8_t num2,
                      uint8_t num3,
                      FrequencyCounter<256>* fc,
                      size_t num_codes = kCodeWordStart) {
      // Create the dict array.
      WriteVectorStream wvs(&dict_buffer_);
      // Save space for dict size.
      dict_buffer_.push_back(0u);
      dict_buffer_.push_back(0u);
      dict_buffer_.push_back(0u);
      dict_buffer_.push_back(0u);
      // Encode escapes and such.
      dict_buffer_.push_back(static_cast<uint8_t>(escape_char_));
      dict_buffer_.push_back(static_cast<uint8_t>(escape_cap_first_));
      dict_buffer_.push_back(static_cast<uint8_t>(escape_cap_word_));
      dict_buffer_.push_back(num1);
      dict_buffer_.push_back(num2);
      dict_buffer_.push_back(num3);
      dict_buffer_.push_back(num_codes);
      // Encode words.
      std::string last;
      for (const auto& w : *words) {
        const std::string& s = w.Word();
        size_t common_len = 0;
        wvs.writeString(&s[0] + common_len, '\0');
        last = s;
      }
      // Save size.
      dict_buffer_pos_ = 0;
      dict_buffer_size_ = dict_buffer_.size();
      dict_buffer_[0] = static_cast<uint8_t>(dict_buffer_size_ >> 24);
      dict_buffer_[1] = static_cast<uint8_t>(dict_buffer_size_ >> 16);
      dict_buffer_[2] = static_cast<uint8_t>(dict_buffer_size_ >> 8);
      dict_buffer_[3] = static_cast<uint8_t>(dict_buffer_size_ >> 0);
      // Generate the actual encode map.
      generate(*words, num1, num2, num3, true, fc, num_codes);
      // Dictionary is prepended to output, make sure to add the bytes to the frequency counter.
      if (fc != nullptr) {
        fc->AddRegion(&dict_buffer_[0], dict_buffer_.size());
      }
      std::cout << "Dictionary words=" << words->size() << " size=" << prettySize(dict_buffer_.size()) << std::endl;
    }
    void createFromBuffer() {
      ReadMemoryStream rms(&dict_buffer_);
      size_t pos = 0;
      // Save space for dict size.
      for (size_t i = 0; i < 4; ++i) {
        rms.get();
      }
      // Encode escapes and such.
      escape_char_ = rms.get();
      escape_cap_first_ = rms.get();
      escape_cap_word_ = rms.get();
      const size_t num1 = rms.get();
      const size_t num2 = rms.get();
      const size_t num3 = rms.get();
      const size_t num_codes = rms.get();
      word1bstart = kCodeWordStart;
      word2bstart = word1bstart + num1;
      word3bstart = word2bstart + num2;
      // Encode words.
      std::vector<WordCount> words;
      while (rms.tellg() != dict_buffer_.size()) {
        WordCount wc(rms.readString());
        words.push_back(wc);
      }
      // Generate the actual encode map.
      generate(words, num1, num2, num3, false, nullptr, num_codes);
      std::cout << "Dictionary words=" << words.size() << " size=" << prettySize(dict_buffer_.size()) << std::endl;
    }
    void generate(std::vector<WordCount>& words,
                  size_t num1,
                  size_t num2,
                  size_t num3,
                  bool encode,
                  FrequencyCounter<256>* fc = nullptr,
                  size_t num_codes = kCodeWordStart) {
      const size_t code_word_start = 256 - num_codes;
      const size_t end1 = code_word_start + num1;
      const size_t end2 = end1 + num2;
      const size_t end3 = end2 + num3;
      size_t idx = 0;
      for (size_t b1 = code_word_start; b1 < end1; ++b1) {
        if (idx < words.size()) {
          if (encode) {
            encode_map_.Add(words[idx].Word(), CodeWord(1, static_cast<uint8_t>(b1)));
            if (fc != nullptr) {
              words[idx].UpdateFrequencies(fc, escape_cap_first_, escape_cap_word_);
              fc->Add(b1, words[idx].Count());
            }
            ++idx;
          } else {
            words1b.push_back(words[idx++].Word());
          }
        }
      }
      for (size_t b1 = end1; b1 < end2; ++b1) {
        for (size_t b2 = (kOverlapCodewords ? code_word_start : end1); b2 < (kOverlapCodewords ? 256u : end2); ++b2) {
          if (idx < words.size()) {
            if (encode) {
              encode_map_.Add(words[idx].Word(), CodeWord(2, static_cast<uint8_t>(b1), static_cast<uint8_t>(b2)));
              if (fc != nullptr) {
                words[idx].UpdateFrequencies(fc, escape_cap_first_, escape_cap_word_);
                fc->Add(b1, words[idx].Count());
                fc->Add(b2, words[idx].Count());
              }
              ++idx;
            } else {
              words2b.push_back(words[idx++].Word());
            }
          }
        }
      }
      for (size_t b1 = end2; b1 < end3; ++b1) {
        for (size_t b2 = (kOverlapCodewords ? code_word_start : end2); b2 < (kOverlapCodewords ? 256u : end3); ++b2) {
          for (size_t b3 = (kOverlapCodewords ? code_word_start : end2); b3 < (kOverlapCodewords ? 256u : end3); ++b3) {
            if (idx < words.size()) {
              if (encode) {
                encode_map_.Add(words[idx].Word(), CodeWord(3, static_cast<uint8_t>(b1), static_cast<uint8_t>(b2), static_cast<uint8_t>(b3)));
                if (fc != nullptr) {
                  words[idx].UpdateFrequencies(fc, escape_cap_first_, escape_cap_word_);
                  fc->Add(b1, words[idx].Count());
                  fc->Add(b2, words[idx].Count());
                  fc->Add(b3, words[idx].Count());
                  ++idx;
                }
              } else {
                words3b.push_back(words[idx++].Word());
              }
            }
          }
        }
      }
    }
    virtual void forwardFilter(uint8_t* out, size_t* out_count, uint8_t* in, size_t* in_count) {
      uint8_t* in_ptr = in;
      uint8_t* out_ptr = out;
      const uint8_t* const in_limit = in + *in_count;
      const uint8_t* const out_limit = out + *out_count;
      const size_t remain_dict = dict_buffer_size_ - dict_buffer_pos_;
      if (remain_dict > 0) {
        const size_t max_write = std::min(remain_dict, *out_count);
        std::copy(&dict_buffer_[0] + dict_buffer_pos_, &dict_buffer_[0] + dict_buffer_pos_ + max_write, out_ptr);
        out_ptr += max_write;
        dict_buffer_pos_ += max_write;
      } else if (kOnlyDict) {
        *out_count = *in_count = 0;
        return;
      }
      while (in_ptr < in_limit && out_ptr + 5 < out_limit) {
        if (!is_word_char_[last_char_]) {
          if (is_word_char_[*in_ptr]) {
            // Calculate maximum word length.
            size_t word_len = 0;
            while (word_len < kMaxWordLen && in_ptr + word_len < in_limit && is_word_char_[in_ptr[word_len]]) {
              ++word_len;
            }
            if (in_ptr + word_len >= in_limit && word_len != in_limit - in) {
              // If the word is all the remaining chars and not the whole string, then it may be a prefix.
              break;
            }
            // Using prefix codes makes compression worse.
            const bool kSupportPrefix = true;
            bool next_word = false;
            const size_t max_out = static_cast<size_t>(out_limit - out_ptr);
            const size_t min_len = kSupportPrefix ? std::min(std::max(word_len, kMinWordLen), static_cast<size_t>(6)) : word_len;
            if (word_len <= kMaxWordLen) {
              for (size_t cur_len = word_len; cur_len >= min_len; --cur_len) {
                WordCC cc = GetWordCase(in_ptr, cur_len);
                std::string word(in_ptr, in_ptr + cur_len);
                if (cc == kWordCCAll) {
                  for (auto& c : word) c = MakeLowerCase(c);
                } else if (cc == kWordCCFirstChar) {
                  word[0] = MakeLowerCase(word[0]);
                }
                const EncodeMap::Entry* entry = encode_map_.Find(word);
                if (entry != nullptr) {
                  if (cc == kWordCCAll) {
                    *(out_ptr++) = static_cast<uint8_t>(escape_cap_word_);
                    if (kStats) ++escape_count_word_;
                  } else if (cc != kWordCCNone) {
                    check(entry->word_case != kWordCCFirstChar);
                    *(out_ptr++) = static_cast<uint8_t>(escape_cap_first_);
                    if (kStats) ++escape_count_first_;
                  } else {
                    check(entry->word_case != kWordCCFirstChar);
                    // *(out_ptr++) = static_cast<uint8_t>(escape_cap_first_);
                    // if (kStats) ++escape_count_first_;
                  }
                  auto& code_word = entry->code_word;
                  const auto num_bytes = code_word.numBytes();
                  dcheck(num_bytes >= 1 && num_bytes <= 3);
                  *(out_ptr++) = code_word.byte1();
                  if (num_bytes > 1) *(out_ptr++) = static_cast<uint8_t>(code_word.byte2());
                  if (num_bytes > 2) *(out_ptr++) = static_cast<uint8_t>(code_word.byte3());
                  in_ptr += cur_len;
                  last_char_ = 'a';
                  next_word = true;
                  break;
                }
              }
            }
            if (next_word) {
              continue;
            }
            if (word_len < max_out) {
              WordCC cc = GetWordCase(in_ptr, word_len);
              last_char_ = 'a';
              std::string word(in_ptr, in_ptr + word_len);
              size_t upper_count = 0;
              if (cc == kWordCCAll) {
                for (auto& c : word) c = MakeLowerCase(c);
                *(out_ptr++) = static_cast<uint8_t>(escape_cap_word_);
                if (kStats) ++escape_count_word_;
                std::copy(&word[0], &word[0] + word_len, out_ptr);
                in_ptr += word_len;
                out_ptr += word_len;
                break;
              } else if (cc == kWordCCFirstChar) {
                word[0] = MakeLowerCase(word[0]);
                *(out_ptr++) = static_cast<uint8_t>(escape_cap_first_);
                if (kStats) ++escape_count_first_;
                std::copy(&word[0], &word[0] + word_len, out_ptr);
                in_ptr += word_len;
                out_ptr += word_len;
                break;
              }
            }
          }
          if (*in_ptr == escape_char_ || *in_ptr == escape_cap_first_ || *in_ptr == escape_cap_word_ || *in_ptr >= 128) {
            if (kStats) ++escape_count_;
            *(out_ptr++) = static_cast<uint8_t>(escape_char_);
          }
        }
        *(out_ptr++) = last_char_ = *(in_ptr++);
      }
      dcheck(in_ptr <= in_limit);
      dcheck(out_ptr <= out_limit);
      *in_count = in_ptr - in;
      *out_count = out_ptr - out;
    }
    virtual void reverseFilter(uint8_t* out, size_t* out_count, uint8_t* in, size_t* in_count) {
      const uint8_t* in_ptr = in;
      uint8_t* out_ptr = out;
      const uint8_t* const in_limit = in + *in_count;
      const uint8_t* const out_limit = out + *out_count;
      while (dict_buffer_.size() < dict_buffer_size_ && in_ptr < in_limit) {
        dict_buffer_.push_back(*(in_ptr++));
        if (dict_buffer_.size() == 4) {
          dict_buffer_size_ = static_cast<uint32_t>(dict_buffer_[0]) << 24;
          dict_buffer_size_ |= static_cast<uint32_t>(dict_buffer_[1]) << 16;
          dict_buffer_size_ |= static_cast<uint32_t>(dict_buffer_[2]) << 8;
          dict_buffer_size_ |= static_cast<uint32_t>(dict_buffer_[3]) << 0;
        } else if (dict_buffer_.size() == dict_buffer_size_) {
          createFromBuffer();
        }
      }
      const auto* max = in_ptr + 4 < in_limit ? in_limit - 4 : in_limit;
      const size_t start_byte = 128;
      while (in_ptr < max && out_ptr + kMaxWordLen < out_limit) {
        int c = *(in_ptr++);
        if (!is_word_char_[last_char_]) {
          const bool first_cap = c == escape_cap_first_;
          const bool all_cap = c == escape_cap_word_;
          if (c >= 128 || first_cap || all_cap) {
            if (first_cap || all_cap) {
              c = *(in_ptr++);
            }
            if (c >= word1bstart) {
              std::string* word;
              if (c < word2bstart) {
                word = &words1b[c - word1bstart];
              } else if (c < word3bstart) {
                int c2 = *(in_ptr++);
                assert(c2 >= 128);
                word = &words2b[(c - word2bstart) * 128 + c2 - start_byte];
              } else {
                assert(c >= word3bstart);
                int c2 = *(in_ptr++);
                int c3 = *(in_ptr++);
                assert(c2 >= start_byte);
                assert(c3 >= start_byte);
                word = &words3b[(c - word3bstart) * 128 * 128 + (c2 - start_byte) * 128 + c3 - start_byte];
              }
              const size_t word_len = word->length();
              auto* word_start = &word->operator[](0);
              std::copy(word_start, word_start + word_len, out_ptr);
              const size_t capital_c = all_cap ? word_len : static_cast<size_t>(first_cap);
              for (size_t i = 0; i < capital_c; ++i) {
                out_ptr[i] = MakeUpperCase(out_ptr[i]);
              }
              out_ptr += word_len;
              last_char_ = out_ptr[-1];
              continue;
            } else if (first_cap && c >= 'a' && c <= 'z') {
              c = MakeUpperCase(c);
            } else if (all_cap) {
              capital_mode_ = true;
            }
          }
          if (c == escape_char_) {
            c = *(in_ptr++);
          }
        }
        if (capital_mode_ && c >= 'a' && c <= 'z') {
          c = MakeUpperCase(c);
        } else {
          capital_mode_ = false;
        }
        *(out_ptr++) = last_char_ = c;
      }
      dcheck(in_ptr <= in_limit);
      dcheck(out_ptr <= out_limit);
      *in_count = in_ptr - in;
      *out_count = out_ptr - out;
    }
    Filter(Stream* stream) : ByteStreamFilter(stream), dict_buffer_pos_(0), dict_buffer_size_(4), last_char_(0) {
      init();
    }
    Filter(Stream* stream, size_t escape_char, size_t escape_cap_first = kInvalidChar,
      size_t escape_cap_word = kInvalidChar)
      : ByteStreamFilter(stream)
      , escape_char_(escape_char)
      , escape_cap_first_(escape_cap_first)
      , escape_cap_word_(escape_cap_word)
      , last_char_(0) {
      init();
    }
    ~Filter() {
      if (kStats) {
        std::cout << std::endl << "Escape " << escape_count_ << " word " << escape_count_word_ << " first " << escape_count_first_ << std::endl;
      }
    }
    void init() {
      for (int i = 0; i < 256; ++i) {
        is_word_char_[i] = static_cast<uint8_t>(IsWordChar(i));
      }
    }
  };

  Dict() {

  }
};

#endif
