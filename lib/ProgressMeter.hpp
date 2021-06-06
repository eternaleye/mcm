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
  void printRatio(uint64_t comp_size, uint64_t in_size, const std::string& extra) const {
    const auto ratio = double(comp_size) / in_size;
    auto cur_time = std::chrono::high_resolution_clock::now();
    auto time_delta = cur_time - start;
    if (time_delta == std::chrono::high_resolution_clock::duration::zero()) {
        time_delta = std::chrono::high_resolution_clock::duration::min();
    }
    const std::chrono::duration<double, std::ratio<1>> delta_secs = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(time_delta);
    const uint32_t rate = uint32_t(double(in_size / KB) / delta_secs.count());
    std::ostringstream oss;
    oss << in_size / KB << "KB ";
    if (comp_size > KB) {
      oss << (encode ? "->" : "<-") << " " << comp_size / KB << "KB ";
    } else {
      oss << ", ";
    }
    oss << rate << "KB/s";
    if (comp_size > KB) {
      oss << " ratio: " << std::setprecision(5) << std::fixed << ratio << extra.c_str();
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

// DEPRECIATED
class ProgressStream : public Stream {
  static const size_t kUpdateInterval = 512 * KB;
public:
  ProgressStream(Stream* in_stream, Stream* out_stream, bool encode = true)
    : in_stream_(in_stream), out_stream_(out_stream), meter_(encode), update_count_(0) {
  }
  virtual ~ProgressStream() {}
  virtual int get() {
    uint8_t b;
    if (read(&b, 1) == 0) {
      return EOF;
    }
    return b;
  }
  virtual size_t read(uint8_t* buf, size_t n) {
    size_t ret = in_stream_->read(buf, n);
    update_count_ += ret;
    if (update_count_ > kUpdateInterval) {
      update_count_ -= kUpdateInterval;
      meter_.printRatio(out_stream_->tellp(), in_stream_->tellg(), "");
    }
    return ret;
  }
  virtual void put(int c) {
    uint8_t b = c;
    write(&b, 1);
  }
  virtual void write(const uint8_t* buf, size_t n) {
    out_stream_->write(buf, n);
    addCount(n);
  }
  void addCount(size_t delta) {
    update_count_ += delta;
    if (update_count_ > kUpdateInterval) {
      update_count_ -= kUpdateInterval;
      size_t comp_size = out_stream_->tellp(), other_size = in_stream_->tellg();
      if (!meter_.isEncode()) {
        std::swap(comp_size, other_size);
      }
      meter_.printRatio(comp_size, other_size, "");
    }
  }
  virtual uint64_t tellg() const {
    return meter_.isEncode() ? in_stream_->tellg() : out_stream_->tellp();
  }
  virtual uint64_t tellp() const {
    return meter_.isEncode() ? in_stream_->tellg() : out_stream_->tellp();
  }
  virtual void seekg(uint64_t pos) {
    meter_.isEncode() ? in_stream_->seekg(pos) : out_stream_->seekp(pos);
  }
  virtual void seekp(uint64_t pos) {
    meter_.isEncode() ? in_stream_->seekg(pos) : out_stream_->seekp(pos);
  }

private:
  Stream* const in_stream_;
  Stream* const out_stream_;
  ProgressMeter meter_;
  uint64_t update_count_;
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
  ProgressThread(Stream* in_stream, Stream* out_stream, bool encode = true, uint64_t sub_out = 0, uintptr_t interval = 250)
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
  Stream* const in_stream_;
  Stream* const out_stream_;
  const uint64_t sub_out_;
  ProgressMeter meter_;
};

#endif
