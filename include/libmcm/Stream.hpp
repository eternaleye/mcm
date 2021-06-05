#ifndef _LIBMCM_STREAM_HPP_
#define _LIBMCM_STREAM_HPP_

#include <libmcm/Error.hpp>  // for unimplemented_error

// TODO: Drop in favor of std::basic_streambuf
class Stream {
    public:
        virtual uint64_t tell() const {
            throw libmcm::unimplemented_error(__FUNCTION__);
        }
        virtual int get() = 0;
        virtual void put(int c) = 0;
        virtual size_t read(uint8_t* buf, size_t n) {
            size_t count;
            for (count = 0; count < n; ++count) {
                auto c = get();
                if (c == EOF) {
                    break;
                }
                buf[count] = c;
            }
            return count;
        }
        // Not thread safe by default.
        virtual size_t readat(uint64_t pos, uint8_t* buf, size_t n) {
            seek(pos);
            return read(buf, n);
        }
        virtual void write(const uint8_t* buf, size_t n) {
            for (;n; --n) {
                put(*(buf++));
            }
        }
        virtual void writeat(uint64_t pos, const uint8_t* buf, size_t n) {
            seek(pos);
            write(buf, n);
        }
        virtual void seek(uint64_t pos) {
            throw libmcm::unimplemented_error(__FUNCTION__);
        }
        virtual ~Stream() {
        }
        // Helper
        void put16(uint16_t n) {
            put(static_cast<uint8_t>(n >> 8));
            put(static_cast<uint8_t>(n >> 0));
        }
        uint16_t get16() {
            uint16_t ret = 0;
            ret = (ret << 8) | static_cast<uint16_t>(get());
            ret = (ret << 8) | static_cast<uint16_t>(get());
            return ret;
        }
#if 0
        inline void leb128Encode(int64_t n) {
            bool neg = n < 0;
            if (neg) n = -n;
            leb128Encode(static_cast<uint64_t>((n << 1) | (neg ? 1u : 0)));
        }
#endif
        inline void leb128Encode(uint64_t n) {
            while (n >= 0x80) {
                auto c = static_cast<uint8_t>(0x80 | (n & 0x7F));
                put(c);
                n >>= 7;
            }
            put(static_cast<uint8_t>(n));
        }
        uint64_t leb128Decode() {
            uint64_t ret = 0;
            uint64_t shift = 0;
            while (true) {
                const uint8_t c = get();
                ret |= static_cast<uint64_t>(c & 0x7F) << shift;
                shift += 7;
                if ((c & 0x80) == 0) break;
            }
            return ret;
        }
        void writeString(const char* str, char terminator) {
            while (*str != '\0') {
                put(*(str++));
            }
            put(terminator);
        }
        std::string readString() {
            std::string s;
            for (;;) {
                int c = get();
                if (c == EOF || c == '\0') break;
                s.push_back(static_cast<char>(c));
            }
            return s;
        }
};

#endif // _LIBMCM_STREAM_HPP_
