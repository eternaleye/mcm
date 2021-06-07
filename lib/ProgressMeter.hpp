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

#ifndef _PROGRESS_METER_HPP_
#define _PROGRESS_METER_HPP_

#include <chrono>              // for duration, operator+, operator>=, time_...
#include <condition_variable>  // for condition_variable
#include <iomanip>             // for operator<<, setprecision
#include <iostream>            // for operator<<, basic_ostream, ostringstream
#include <mutex>               // for mutex, unique_lock
#include <string>              // for operator<<, string
#include <thread>              // for thread
#include <utility>             // for swap

#include <cstddef>             // for size_t
#include <cstdint>             // for uint64_t, uint8_t, uintptr_t, uint32_t
#include <cstdio>              // for EOF

#include "Stream.hpp"          // for Stream
#include "Util.hpp"            // for KB

class ProgressMeter {
  uint64_t count;
  std::chrono::high_resolution_clock::time_point start;
  bool encode;
public:
  ProgressMeter(bool encode = true)
    : count(0)
    , start(std::chrono::high_resolution_clock::now())
    , encode(encode) {
  }

  ~ProgressMeter() {
  }

  inline uint64_t getCount() const {
    return count;
  }

  inline uint64_t addByte() {
    return ++count;
  }

  bool isEncode() const {
    return encode;
  }

  // Surprisingly expensive to call...
  void printRatio(uint64_t comp_size, const std::string& extra) {
    printRatio(comp_size, count, extra);
  }

  // Surprisingly expensive to call...
  void printRatio(uint64_t out_size, uint64_t in_size, const std::string& extra) const {
    auto cur_time = std::chrono::high_resolution_clock::now();
    auto time_delta = cur_time - start;
    if (time_delta == std::chrono::high_resolution_clock::duration::zero()) {
        time_delta = std::chrono::high_resolution_clock::duration::min();
    }
    const std::chrono::duration<double, std::ratio<1>> delta_secs = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(time_delta);
    std::ostringstream oss;
    if (encode) {
      const auto ratio = double(out_size) / in_size;
      const uint32_t rate = uint32_t(double(in_size / KB) / delta_secs.count());
      if (in_size >= KB) {
        oss << in_size / KB << "KB";
      } else {
        oss << in_size << "B";
      }
      oss << " -> ";
      if (out_size >= KB) {
        oss << out_size / KB << "KB";
      } else {
        oss << out_size << "B";
      }
      oss << " @ " << rate << "KB/s";
      oss << " ratio: " << std::setprecision(5) << std::fixed << ratio << extra.c_str();
    } else {
      const uint32_t rate = uint32_t(double(out_size / KB) / delta_secs.count());
      if (out_size >= KB) {
        oss << out_size / KB << "KB";
      } else {
        oss << out_size << "B";
      }
      oss << " <- ";
      if (in_size >= KB) {
        oss << in_size / KB << "KB";
      } else {
        oss << in_size << "B";
      }
      oss << " @ " << rate << "KB/s";
    }
    std::cout << oss.str() << "\t\r" << std::flush;
  }

  inline void addBytePrint(uint64_t total, const char* extra = "") {
    if (!(addByte() & 0xFFFF)) {
      // 4 updates per second.
      // if (clock() - prev_time > 250) {
      // 	printRatio(total, extra);
      // }
    }
  }
};

class AutoUpdater {
public:
  AutoUpdater(uintptr_t interval = 250)
    : done_(false), interval_(interval) {
    thread_ = new std::thread(Callback, this);
  }
  virtual ~AutoUpdater() {
    done();
  }
  void done() {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (done_) return;
      done_ = true;
      cond_.notify_one();
    }
    thread_->join();
    delete thread_;
  }
  virtual void print() = 0;

protected:
  bool done_;
  std::thread* thread_;
  std::mutex mutex_;
  std::condition_variable cond_;
  const uintptr_t interval_;

  void run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!done_) {
      auto cur = std::chrono::high_resolution_clock::now();
      auto target = cur + std::chrono::milliseconds(interval_);
      while (true) {
        cond_.wait_for(lock, std::chrono::milliseconds(interval_));  // target - cur
        cur = std::chrono::high_resolution_clock::now();
        if (done_) return;
        if (cur >= target) break;
      }
      if (!done_) {
        print();
      }
    }
  }
  static void Callback(AutoUpdater* thr) {
    thr->run();
  }
};


class ProgressThread : public AutoUpdater {
public:
  ProgressThread(InStream* in_stream, OutStream* out_stream, bool encode = true, uint64_t sub_out = 0, uintptr_t interval = 250)
    : AutoUpdater(interval), in_stream_(in_stream), out_stream_(out_stream), sub_out_(sub_out), meter_(encode) {
  }
  ~ProgressThread() {
    done();
  }
  virtual void print() {
    auto out_c = out_stream_->tellp() - sub_out_;
    auto in_c = in_stream_->tellg();
    if (in_c != 0 && out_c != 0) {
      meter_.printRatio(out_c, in_c, "");
    }
  }

protected:
  InStream* const in_stream_;
  OutStream* const out_stream_;
  const uint64_t sub_out_;
  ProgressMeter meter_;
};

#endif
