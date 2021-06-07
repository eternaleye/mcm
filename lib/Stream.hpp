#ifndef STREAM_HPP_
#define STREAM_HPP_

#include <algorithm>          // for copy, min, max
#include <iostream>           // for operator<<, basic_ostream, endl, basic_ostream<...
#include <string>             // for string, char_traits
#include <vector>             // for vector

#include <cassert>            // for assert
#include <climits>            // for CHAR_BIT
#include <cstddef>            // for size_t
#include <cstdio>             // for EOF
#include <cstdint>            // for uint8_t, uint64_t, uint32_t, uint16_t

#include <libmcm/Error.hpp>   // for unimplemented_error
#include <libmcm/Stream.hpp>  // for Stream

#include "Util.hpp"           // for KB

class WriteStream : public Stream {
public:
  virtual int get() {
    throw libmcm::unimplemented_error(__FUNCTION__);
  }
  virtual void put(int c) = 0;
  virtual ~WriteStream() {}
};

class VoidWriteStream : public WriteStream {
  uint64_t pos_;
public:
  VoidWriteStream() : pos_(0) {}
  virtual ~VoidWriteStream() {}
  virtual void write(const uint8_t*, size_t n) {
    pos_ += n;
  }
  virtual void put(int) {
    ++pos_;
  }
  virtual uint64_t tellp() const {
    return pos_;
  }
  virtual void seekp(uint64_t pos) {
    pos_ = pos;
  }
};

class ReadMemoryStream : public InStream {
public:
  ReadMemoryStream(const std::vector<uint8_t>* buffer)
    : buffer_(buffer->data())
    , pos_(buffer->data())
    , limit_(buffer->data() + buffer->size()) {
  }
  ReadMemoryStream(const uint8_t* buffer, const uint8_t* limit) : buffer_(buffer), pos_(buffer), limit_(limit) {
  }
  virtual int get() {
    if (pos_ >= limit_) {
      return EOF;
    }
    return *pos_++;
  }
  virtual size_t read(uint8_t* buf, size_t n) {
    const size_t remain = limit_ - pos_;
    const size_t read_count = std::min(remain, n);
    std::copy(pos_, pos_ + read_count, buf);
    pos_ += read_count;
    return read_count;
  }
  virtual uint64_t tellg() const {
    return pos_ - buffer_;
  }

private:
  const uint8_t* const buffer_;
  const uint8_t* pos_;
  const uint8_t* const limit_;
};

class WriteMemoryStream : public OutStream {
public:
  explicit WriteMemoryStream(uint8_t* buffer) : buffer_(buffer), pos_(buffer) {
  }
  virtual void put(int c) {
    *pos_++ = static_cast<uint8_t>(static_cast<unsigned int>(c));
  }
  virtual void write(const uint8_t* data, uint32_t count) {
    std::copy(data, data + count, pos_);
    pos_ += count;
  }
  virtual uint64_t tellp() const {
    return pos_ - buffer_;
  }

private:
  uint8_t* buffer_;
  uint8_t* pos_;
};

class WriteVectorStream : public OutStream {
public:
  explicit WriteVectorStream(std::vector<uint8_t>* buffer) : buffer_(buffer) {
  }
  virtual void put(int c) {
    buffer_->push_back(c);
  }
  virtual void write(const uint8_t* data, uint32_t count) {
    buffer_->insert(buffer_->end(), data, data + count);
  }
  virtual uint64_t tellp() const {
    return buffer_->size();
  }

private:
  std::vector<uint8_t>* const buffer_;
};

template <typename T>
class OStreamWrapper : public std::ostream {
  class StreamBuf : public std::streambuf {
  public:
  };
public:
};

template <const uint32_t buffer_size>
class BufferedStreamReader {
public:
  InStream* stream;
  size_t buffer_count, buffer_pos;
  uint8_t buffer[buffer_size];
  bool done_;

  bool done() const { return done_; }

  BufferedStreamReader(InStream* stream) {
    assert(stream != nullptr);
    init(stream);
  }
  virtual ~BufferedStreamReader() {
  }
  void init(InStream* new_stream) {
    stream = new_stream;
    buffer_pos = 0;
    buffer_count = 0;
    done_ = false;
  }
  inline size_t remain() const {
    return buffer_count - buffer_pos;
  }
  inline int get() {
    if (remain() == 0 && Refill() == false) {
      return EOF;
    }
    return buffer[buffer_pos++];
  }
  // Range7 is being icky
  void put(int c) {
    throw libmcm::unimplemented_error(__FUNCTION__);
  }
  uint64_t tellg() const {
    return stream->tellg() + buffer_pos;
  }
private:
  bool Refill() {
    buffer_pos = 0;
    buffer_count = stream->read(buffer, buffer_size);
    if (buffer_count == 0) {
      done_ = true;
      return false;
    }
    return true;
  }
};

template <const uint32_t kBufferSize>
class BufferedStreamWriter {
public:
  BufferedStreamWriter(OutStream* stream) {
    assert(stream != nullptr);
    init(stream);
  }
  virtual ~BufferedStreamWriter() {
    flush();
    assert(ptr_ == buffer_);
  }
  void init(OutStream* new_stream) {
    stream_ = new_stream;
    ptr_ = buffer_;
  }
  void flush() {
    stream_->write(buffer_, ptr_ - buffer_);
    ptr_ = buffer_;
  }
  inline void put(uint8_t c) {
    if (ptr_ >= end()) {
      flush();
    }
    *(ptr_++) = c;
  }
  // Range7 is being icky
  int get() {
    throw libmcm::unimplemented_error(__FUNCTION__);
  }
  uint64_t tellp() const {
    return stream_->tellp() + (ptr_ - buffer_);
  }

private:
  const uint8_t* end() const {
    return &buffer_[kBufferSize];
  }
  OutStream* stream_;
  uint8_t buffer_[kBufferSize];
  uint8_t* ptr_;
};

class VerifyStream : public Stream {
public:
  InStream* const stream_;
  uint64_t differences_;
  uint64_t count_, ref_count_;

  VerifyStream(InStream* stream, size_t ref_count) : stream_(stream), count_(0), ref_count_(ref_count) {
    init();
  }
  int get() {
    throw libmcm::unimplemented_error(__FUNCTION__);
  }
  uint64_t getCount() const {
    return count_;
  }
  void resetCount() {
    count_ = 0;
  }
  void init() {
    differences_ = 0;
  }
  void put(int c) {
    auto ref = stream_->get();
    if (c != ref) {
      difference(ref, c);
    }
    ++count_;
  }
  void seekp(uint64_t pos) {
    stream_->seekg(pos);
  }
  void write(const uint8_t* buf, size_t n) {
    uint8_t buffer[4 * KB];
    while (n != 0) {
      size_t count = stream_->read(buffer, std::min(static_cast<size_t>(4 * KB), n));
      for (size_t i = 0; i < count; ++i) {
        auto ref = buffer[i];
        if (buf[i] != buffer[i]) {
          difference(buffer[i], buf[i]);
        }
      }
      buf += count;
      n -= count;
      count_ += count;
    }
  }
  void difference(int ref, int c) {
    if (differences_ == 0) {
      std::cerr << "Difference found at byte! " << stream_->tellg() << " b1: " << "ref: "
        << static_cast<int>(ref) << " new: " << static_cast<int>(c) << std::endl;
    }
    ++differences_;
  }
  virtual uint64_t tellp() const {
    return count_;
  }
  void summary() {
    if (count_ != ref_count_) {
      std::cerr << "ERROR: Missing bytes " << count_ << "/" << ref_count_ << " differences=" << differences_ << std::endl;
    } else {
      if (differences_) {
        std::cerr << "ERROR: differences=" << differences_ << std::endl;
      } else {
        std::cout << "No differences found!" << std::endl;
      }
    }
  }
};

#endif
