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

#ifndef _FILTER_HPP_
#define _FILTER_HPP_

#include <algorithm>   // for min, copy
#include <memory>      // for unique_ptr

#include <cstdint>     // for uint8_t, uint32_t, uint64_t
#include <cstdio>      // for size_t, EOF

#include "Stream.hpp"  // for Stream
#include "Util.hpp"    // for KB, check, FrequencyCounter

/*
Filter usage:

// in_stream -> filter -> out_stream
Filter f(in_stream);
compress(out_stream, &f);

// in_stream -> filter -> out_stream
Filter f(out_stream);
compres(&f, in_stream);

*/

class Filter : public Stream {
protected:
  uint64_t opt_var_ = 0;
public:

  void setOpt(uint64_t opt_var) {
    opt_var_ = opt_var;
  }

  virtual FrequencyCounter<256> GetFrequencies() {
    return FrequencyCounter<256>();
  }

  virtual void flush() {}
};

template <class T, uint32_t kCapacity>
class StaticBuffer {
public:
  StaticBuffer() : pos_(0), size_(0) {
  }
  inline const T& operator[](size_t i) const {
    return data_[i];
  }
  inline T& operator[](size_t i) {
    return data_[i];
  }
  inline size_t pos() const {
    return pos_;
  }
  inline size_t size() const {
    return size_;
  }
  inline size_t capacity() const {
return kCapacity;
  }
  inline size_t reamainCapacity() const {
    return capacity() - size();
  }
  inline T get() {
    (pos_ < size_);
    return data_[pos_++];
  }
  inline void read(T* ptr, size_t len) {
    dcheck(pos_ + len <= size_);
    std::copy(&data_[pos_], &data_[pos_ + len], &ptr[0]);
    pos_ += len;
  }
  inline void put(T c) {
    dcheck(pos_ < size_);
    data_[pos_++] = c;
  }
  inline void write(const T* ptr, size_t len) {
    dcheck(pos_ + len <= size_);
    std::copy(&ptr[0], &ptr[len], &data_[pos_]);
    pos_ += len;
  }
  inline size_t remain() const {
    return size_ - pos_;
  }
  void erase(size_t chars) {
    dcheck(chars <= pos());
    std::move(&data_[chars], &data_[size()], &data_[0]);
    pos_ -= std::min(pos_, chars);
    size_ -= std::min(size_, chars);
  }
  void addPos(size_t n) {
    pos_ += n;
    dcheck(pos_ <= size());
  }
  void addSize(size_t n) {
    size_ += n;
    dcheck(size_ <= capacity());
  }
  T* begin() {
    return &operator[](0);
  }
  T* end() {
    return &operator[](size_);
  }
  T* limit() {
    return &operator[](capacity());
  }

private:
  size_t pos_;
  size_t size_;
  T data_[kCapacity];
};

// Byte filter is complicated since filters are not necessarily a 1:1 mapping.
template<uint32_t kInBufferSize = 16 * KB, uint32_t kOutBufferSize = 16 * KB>
class ByteStreamFilter : public Filter {
public:
  void flush() {
    while (in_buffer_.pos() != 0) {
      refillWriteAndProcess();
    }
  }
  explicit ByteStreamFilter(Stream* stream) : stream_(stream), count_(0) {
  }
  virtual int get() {
    if (out_buffer_.remain() == 0) {
      if (refillReadAndProcess() == 0) {
        return EOF;
      }
    }
    return out_buffer_.get();
  }
  virtual size_t read(uint8_t* buf, size_t n) {
    const uint8_t* start_ptr = buf;
    while (n != 0) {
      size_t remain = out_buffer_.remain();
      if (remain == 0) {
        if ((remain = refillReadAndProcess()) == 0) {
          break;
        }
      }
      const size_t read_count = std::min(remain, n);
      out_buffer_.read(buf, read_count);
      buf += read_count;
      n -= read_count;
    }
    return buf - start_ptr;
  }
  virtual void put(int c) {
    if (in_buffer_.remain() == 0) {
      if (refillWriteAndProcess() == 0) {
        check(false);
      }
    }
    in_buffer_.put(c);
  }
  virtual void write(const uint8_t* buf, size_t n) {
    while (n != 0) {
      size_t remain = in_buffer_.remain();
      if (remain == 0) {
        remain = refillWriteAndProcess();
        check(remain != 0);
      }
      const size_t len = std::min(n, remain);
      in_buffer_.write(buf, len);
      buf += len;
      n -= len;
    }
  }
  virtual void forwardFilter(uint8_t* out, size_t* out_count, uint8_t* in, size_t* in_count) = 0;
  virtual void reverseFilter(uint8_t* out, size_t* out_count, uint8_t* in, size_t* in_count) = 0;
  uint64_t tellg() const {
    return count_;
  }
  uint64_t tellp() const {
    return count_;
  }

private:
  size_t refillWriteAndProcess() {
    size_t in_pos = 0;
    size_t out_limit = out_buffer_.capacity();
    size_t in_limit = in_buffer_.pos() - in_pos;
    reverseFilter(&out_buffer_[0], &out_limit, &in_buffer_[in_pos], &in_limit);
    in_pos += in_limit;
    stream_->write(&out_buffer_[0], out_limit);
    in_buffer_.erase(in_pos);
    in_buffer_.addSize(in_buffer_.reamainCapacity());
    return in_buffer_.remain();
  }
  size_t refillReadAndProcess() {
    refillRead();  // Try to refill as much of the inbuffer as possible.
    out_buffer_.erase(out_buffer_.pos());  // Erase the characters we already read from the out buffer.
    size_t out_limit = out_buffer_.reamainCapacity();
    size_t in_limit = in_buffer_.pos();
    forwardFilter(out_buffer_.end(), &out_limit, in_buffer_.begin(), &in_limit);
    out_buffer_.addSize(out_limit);  // Add the characters we processed to out.
    count_ += out_limit;
    in_buffer_.erase(in_limit);  // Erase the caracters we processed in in.
    return out_buffer_.size();
  }
  void refillRead() {
    // Read from input until buffer is full.
    size_t count = stream_->read(in_buffer_.end(), in_buffer_.reamainCapacity());
    in_buffer_.addSize(count);
    in_buffer_.addPos(count);
  }

  // In buffer, contains either transformed or untransformed.
  StaticBuffer<uint8_t, kInBufferSize> in_buffer_;
  // Out buffer (passed through filter or reverse filter).
  StaticBuffer<uint8_t, kInBufferSize> out_buffer_;

protected:
  // Proxy stream.
  Stream* const stream_;
  uint64_t count_;
};

#endif
