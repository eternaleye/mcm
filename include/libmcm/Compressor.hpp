#ifndef _LIBMCM_COMPRESSOR_HPP_
#define _LIBMCM_COMPRESSOR_HPP_

#include <memory>             // for unique_ptr

#include <libmcm/Stream.hpp>  // for Stream

class Compressor {
    public:
        class Factory {
            public:
                virtual std::unique_ptr<Compressor> create() = 0;
        };

        template <typename CompressorType>
        class FactoryOf : public Compressor::Factory {
            virtual std::unique_ptr<Compressor> create() {
                // TODO<C++17>: Use std::make_unique
                return new CompressorType();
            }
        };

        // Optimization variable for brute forcing.
        virtual bool setOpt(uint32_t opt) {
            return true;
        }

        virtual bool setOpts(size_t* opts) {
            return true;
        }

        virtual uint32_t getOpt() const {
            return 0;
        }

        virtual void setMemUsage(uint32_t level) {}

        virtual bool failed() {
            return false;
        }

        // Compress n bytes.
        virtual void compress(InStream* in, OutStream* out, uint64_t max_count = 0xFFFFFFFFFFFFFFFF) = 0;

        // Decompress n bytes, the calls must line up. You can't do C(20)C(30)D(50)
        virtual void decompress(InStream* in, OutStream* out, uint64_t max_count = 0xFFFFFFFFFFFFFFFF) = 0;

        virtual ~Compressor() {}
};

#endif // _LIBMCM_COMPRESSOR_HPP
