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

#ifndef _MIXER_HPP_
#define _MIXER_HPP_

#include <vector>       // for vector

#include <cassert>      // for assert
#include <cstddef>      // for size_t
#include <cstdint>      // for uint32_t, int64_t, uint16_t

#include <emmintrin.h>  // for _mm_extract_epi16, _mm_insert_epi16, _mm_add_...

template <const uint32_t A, const uint32_t B, const uint32_t C, const uint32_t D>
struct shuffle {
  enum {
    value = (D << 6) | (C << 4) | (B << 2) | A,
  };
};

template <typename T, const uint32_t kWeights>
class Mixer {
public:
  // Each mixer has its own set of weights.
  T w_[kWeights];

  // Skew weight.
  int skew_;

  // Current learn rate.
  int learn_;
public:
  Mixer() {
    Init(12);
  }

  inline static uint32_t NumWeights() {
    return kWeights;
  }

  inline int GetLearn() const {
    return learn_;
  }

  inline int NextLearn(size_t max_shift) {
    auto before = learn_;
    ++learn_;
    learn_ -= learn_ >> max_shift;
    return before;
  }

  inline T GetWeight(uint32_t index) const {
    assert(index < kWeights);
    return w_[index];
  }

  inline void SetWeight(uint32_t index, T weight) {
    assert(index < kWeights);
    w_[index] = weight;
  }

  void Init(int prob_shift, int extra = 0) {
    for (auto& cw : w_) {
      cw = static_cast<T>(((16 + extra) << prob_shift) / kWeights / 16);
    }
    // Last weight is skew.
    skew_ = 0;
    learn_ = 0;
  }

  // "Fast" version
  inline int P(
    int prob_shift,
    int p0 = 0, int p1 = 0, int p2 = 0, int p3 = 0, int p4 = 0, int p5 = 0, int p6 = 0, int p7 = 0,
    int p8 = 0, int p9 = 0, int p10 = 0, int p11 = 0, int p12 = 0, int p13 = 0, int p14 = 0, int p15 = 0) const {
    int64_t ptotal = skew_;
    if (kWeights > 0) ptotal += p0 * static_cast<int>(GetWeight(0));
    if (kWeights > 1) ptotal += p1 * static_cast<int>(GetWeight(1));
    if (kWeights > 2) ptotal += p2 * static_cast<int>(GetWeight(2));
    if (kWeights > 3) ptotal += p3 * static_cast<int>(GetWeight(3));
    if (kWeights > 4) ptotal += p4 * static_cast<int>(GetWeight(4));
    if (kWeights > 5) ptotal += p5 * static_cast<int>(GetWeight(5));
    if (kWeights > 6) ptotal += p6 * static_cast<int>(GetWeight(6));
    if (kWeights > 7) ptotal += p7 * static_cast<int>(GetWeight(7));
    if (kWeights > 8) ptotal += p8 * static_cast<int>(GetWeight(8));
    if (kWeights > 9) ptotal += p9 * static_cast<int>(GetWeight(9));
    if (kWeights > 10) ptotal += p10 * static_cast<int>(GetWeight(10));
    if (kWeights > 11) ptotal += p11 * static_cast<int>(GetWeight(11));
    if (kWeights > 12) ptotal += p12 * static_cast<int>(GetWeight(12));
    if (kWeights > 13) ptotal += p13 * static_cast<int>(GetWeight(13));
    if (kWeights > 14) ptotal += p14 * static_cast<int>(GetWeight(14));
    if (kWeights > 15) ptotal += p15 * static_cast<int>(GetWeight(15));
    return ptotal >> prob_shift;
  }

  inline bool Update(int pr, uint32_t bit,
    uint32_t prob_shift = 12, int limit = 24, int delta_round = 250, int skew_learn = 1,
    int learn_mult = 31, size_t shift = 16,
    int p0 = 0, int p1 = 0, int p2 = 0, int p3 = 0, int p4 = 0, int p5 = 0, int p6 = 0, int p7 = 0,
    int p8 = 0, int p9 = 0, int p10 = 0, int p11 = 0, int p12 = 0, int p13 = 0, int p14 = 0, int p15 = 0) {
    const int64_t base_learn = static_cast<int64_t>(bit << prob_shift) - pr;
    // const int delta_round = (1 << shift) >> (prob_shift - delta);
    const int64_t err = base_learn * learn_mult;
    const bool ret = err < static_cast<int64_t>(-delta_round) || err > static_cast<int64_t>(delta_round);
    if (ret) {
      UpdateRec<0>(p0, err, shift);
      UpdateRec<1>(p1, err, shift);
      UpdateRec<2>(p2, err, shift);
      UpdateRec<3>(p3, err, shift);
      UpdateRec<4>(p4, err, shift);
      UpdateRec<5>(p5, err, shift);
      UpdateRec<6>(p6, err, shift);
      UpdateRec<7>(p7, err, shift);
      UpdateRec<8>(p8, err, shift);
      UpdateRec<9>(p9, err, shift);
      UpdateRec<10>(p10, err, shift);
      UpdateRec<11>(p11, err, shift);
      UpdateRec<12>(p12, err, shift);
      UpdateRec<13>(p13, err, shift);
      UpdateRec<14>(p14, err, shift);
      UpdateRec<15>(p15, err, shift);
      skew_ += err << skew_learn;
      learn_ += learn_ < limit;
    }
    return ret;
  }

private:
  template <const int kIndex>
  inline void UpdateRec(int64_t p, int64_t err, size_t shift) {
    if (kWeights > kIndex) {
      w_[kIndex] += (err * p) >> shift;
    }
  }
};

template <typename Mixer>
class MixerArray {
  std::vector<Mixer> mixers_;
  Mixer* cur_mixers_;
public:
  template <typename... Args>
  void Init(size_t count, Args... args) {
    mixers_.resize(count);
    for (auto& m : mixers_) {
      m.Init(args...);
    }
    SetContext(0);
  }

  inline size_t Size() const {
    return mixers_.size();
  }

  inline void SetContext(size_t ctx) {
    cur_mixers_ = &mixers_[ctx];
  }

  inline size_t GetContext() const {
    return cur_mixers_ - &mixers_[0];
  }

  inline Mixer* GetMixer() {
    return cur_mixers_;
  }

  inline Mixer* GetMixer(size_t idx) {
    return &mixers_[idx];
  }
};

#endif
