/*	MCM file compressor

  Copyright (C) 2015, Google Inc.
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

#ifndef _SSE_HPP_
#define _SSE_HPP_

#include <algorithm>  // for min
#include <vector>     // for vector

#include <cstddef>    // for size_t
#include <cstdint>    // for uint32_t

#include "Mixer.hpp"  // for Mixer, MixerArray
#include "Model.hpp"  // for bitLearnModel, fastBitModel
#include "Util.hpp"   // for dcheck, Clamp, check

// template <size_t kProbBits, size_t kStemBits = 5, class StationaryModel = fastBitModel<int, kProbBits, 8, 30>>
template <size_t kProbBits, size_t kStemBits = 5, class StationaryModel = bitLearnModel<kProbBits, 8, 30>>
class SSE {
  static const size_t kStems = (1 << kStemBits) + 1;
  static const size_t kMaxP = 1 << kProbBits;
  static const size_t kProbMask = (1 << (kProbBits - kStemBits)) - 1;
  static const size_t kRound = 1 << (kProbBits - kStemBits - 1);
  size_t pw = 0;
  size_t opt = 0;
  size_t count = 0;
public:
  std::vector<StationaryModel> models;

  void setOpt(size_t var) {
    opt = var;
  }

  template <typename Table>
  void init(size_t num_ctx, const Table* table) {
    pw = opt = count = 0;
    check(num_ctx > 0);
    models.resize(num_ctx * kStems);
    for (size_t i = 0; i < kStems; ++i) {
      auto& m = models[i];
      int p = std::min(static_cast<uint32_t>(i << (kProbBits - kStemBits)), static_cast<uint32_t>(kMaxP));
      m.init(table != nullptr ? table->sq(p - 2048) : p);
    }
    size_t ctx = 1;
    while (ctx * 2 <= num_ctx) {
      std::copy(&models[0], &models[kStems * ctx], &models[0] + kStems * ctx);
      ctx *= 2;
    }
    std::copy(&models[0], &models[kStems * (num_ctx - ctx)], &models[0] + ctx * kStems);
  }

  inline int p(size_t p, size_t ctx) {
    dcheck(p < kMaxP);
    const size_t idx = p >> (kProbBits - kStemBits);
    dcheck(idx < kStems);
    const size_t s1 = ctx * kStems + idx;
    const size_t mask = p & kProbMask;
    pw = s1 + (mask >> (kProbBits - kStemBits - 1));
    return (models[s1].getP() * (1 + kProbMask - mask) + models[s1 + 1].getP() * mask) >> (kProbBits - kStemBits);
  }

  inline void update(size_t bit) {
    models[pw].update(bit);
  }
};

#endif
