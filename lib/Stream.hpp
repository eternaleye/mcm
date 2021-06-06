#ifndef STREAM_HPP_
#define STREAM_HPP_

#include <algorithm>          // for copy, min, max
#include <iostream>           // for operator<<, basic_ostream, endl, basic_ostream<...
#include <string>             // for string, char_traits
#include <vector>             // for vector

#include <cassert>            // for assert
#include <cstddef>            // for size_t
#include <cstdio>             // for EOF
#include <cstdint>            // for uint8_t, uint64_t, uint32_t, uint16_t

#include <libmcm/Error.hpp>   // for unimplemented_error
#include <libmcm/Stream.hpp>  // for Stream

#include "Util.hpp"           // for kBitsPerByte

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
  virtual uint64_t tell() const {
    return pos_;
  }
  virtual void seek(uint64_t pos) {
    pos_ = pos;
  }
};

class ReadStream : public Stream {
public:
  virtual int get() = 0;
  virtual void put(int c) {
    throw libmcm::unimplemented_error(__FUNCTION__);
  }
  virtual ~ReadStream() {}
};

class ReadMemoryStream : public ReadStream {
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
  virtual uint64_t tell() const {
    return pos_ - buffer_;
  }

private:
  const uint8_t* const buffer_;
  const uint8_t* pos_;
  const uint8_t* const limit_;
};

class WriteMemoryStream : public WriteStream {
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
  virtual uint64_t tell() const {
    return pos_ - buffer_;
  }

private:
  uint8_t* buffer_;
  uint8_t* pos_;
};

class WriteVectorStream : public WriteStream {
public:
  explicit WriteVectorStream(std::vector<uint8_t>* buffer) : buffer_(buffer) {
  }
  virtual void put(int c) {
    buffer_->push_back(c);
  }
  virtual void write(const uint8_t* data, uint32_t count) {
    buffer_->insert(buffer_->end(), data, data + count);
  }
  virtual uint64_t tell() const {
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
  Stream* stream;
  size_t buffer_count, buffer_pos;
  uint8_t buffer[buffer_size];
  bool done_;

  bool done() const { return done_; }

  BufferedStreamReader(Stream* stream) {
    assert(stream != nullptr);
    init(stream);
  }
  virtual ~BufferedStreamReader() {
  }
  void init(Stream* new_stream) {
    stream = new_stream;
    buffer_pos = 0;
    buffer_count = 0;
    done_ = false;
  }
  inline size_t remain() const {
    return buffer_count - buffer_pos;
  }
  inline int get() {
    if (UNLIKELY(remain() == 0 && Refill() == false)) {
      return EOF;
    }
    return buffer[buffer_pos++];
  }
  void put(int c) {
    throw libmcm::unimplemented_error(__FUNCTION__);
  }
  uint64_t tell() const {
    return stream->tell() + buffer_pos;
  }
private:
  bool Refill() {
    buffer_pos = 0;
    buffer_count = stream->read(buffer, buffer_size);
    if (UNLIKELY(buffer_count == 0)) {
      done_ = true;
      return false;
    }
    return true;
  }
};

template <const uint32_t kBufferSize>
class BufferedStreamWriter {
public:
  BufferedStreamWriter(Stream* stream) {
    assert(stream != nullptr);
    init(stream);
  }
  virtual ~BufferedStreamWriter() {
    flush();
    assert(ptr_ == buffer_);
  }
  void init(Stream* new_stream) {
    stream_ = new_stream;
    ptr_ = buffer_;
  }
  void flush() {
    stream_->write(buffer_, ptr_ - buffer_);
    ptr_ = buffer_;
  }
  inline void put(uint8_t c) {
    if (UNLIKELY(ptr_ >= end())) {
      flush();
    }
    *(ptr_++) = c;
  }
  int get() {
    throw libmcm::unimplemented_error(__FUNCTION__);
  }
  uint64_t tell() const {
    return stream_->tell() + (ptr_ - buffer_);
  }

private:
  const uint8_t* end() const {
    return &buffer_[kBufferSize];
  }
  Stream* stream_;
  uint8_t buffer_[kBufferSize];
  uint8_t* ptr_;
};

template <const bool kLazy = true>
class MemoryBitStream {
  uint8_t* __restrict data_;
  uint32_t buffer_;
  uint32_t bits_;
  static const uint32_t kBitsPerSizeT = sizeof(uint32_t) * kBitsPerByte;
public:
  inline MemoryBitStream(uint8_t* data) : data_(data), buffer_(0), bits_(0) {
  }

  uint8_t* getData() {
    return data_;
  }

  inline void tryReadByte() {
    if (bits_ <= kBitsPerSizeT - kBitsPerByte) {
      readByte();
    }
  }

  inline void readByte() {
    buffer_ = (buffer_ << kBitsPerByte) | *data_++;
    bits_ += kBitsPerByte;
  }

  inline uint32_t readBits(uint32_t bits) {
    if (kLazy) {
      while (bits_ < bits) {
        readByte();
      }
    } else {
      // This might be slower
      tryReadByte();
      tryReadByte();
      tryReadByte();
    }
    bits_ -= bits;
    uint32_t ret = buffer_ >> bits_;
    buffer_ -= ret << bits_;
    return ret;
  }

  inline void flushByte() {
    bits_ -= kBitsPerByte;
    uint32_t byte = buffer_ >> bits_;
    buffer_ -= byte << bits_;
    *data_++ = byte;
  }

  void flush() {
    while (bits_ > kBitsPerByte) {
      flushByte();
    }
    *data_++ = buffer_;
  }

  inline void writeBits(uint32_t data, uint32_t bits) {
    bits_ += bits;
    buffer_ = (buffer_ << bits) | data;
    while (bits_ >= kBitsPerByte) {
      flushByte();
    }
  }
};

class VerifyStream : public WriteStream {
public:
  Stream* const stream_;
  uint64_t differences_;
  uint64_t count_, ref_count_;

  VerifyStream(Stream* stream, size_t ref_count) : stream_(stream), count_(0), ref_count_(ref_count) {
    init();
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
  void seek(uint64_t pos) {
    stream_->seek(pos);
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
      std::cerr << "Difference found at byte! " << stream_->tell() << " b1: " << "ref: "
        << static_cast<int>(ref) << " new: " << static_cast<int>(c) << std::endl;
    }
    ++differences_;
  }
  virtual uint64_t tell() const {
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
