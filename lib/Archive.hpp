/*	MCM file compressor

  Copyright (C) 2014, Google Inc.
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

#ifndef ARCHIVE_HPP_
#define ARCHIVE_HPP_

#include <cstddef>         // for size_t
#include <cstdint>         // for uint16_t, uint64_t, uint8_t
#include <iosfwd>          // for ostream
#include <memory>          // for unique_ptr
#include <string>          // for string
#include <vector>          // for vector

#include "Compressor.hpp"  // for Compressor, CompressorType
#include "Detector.hpp"    // for Detector, Detector::Profile, Analyzer, Ana...
#include "File.hpp"        // for FileSegmentStream::FileSegments, FileInfo ...
#include "Util.hpp"        // for FrequencyCounter

class Filter;
class Stream;

// Force filter
enum FilterType {
  kFilterTypeNone,
  kFilterTypeDict,
  kFilterTypeX86,
  kFilterTypeAuto,
  kFilterTypeCount,
};

enum CompLevel {
  kCompLevelStore,
  kCompLevelTurbo,
  kCompLevelFast,
  kCompLevelMid,
  kCompLevelHigh,
  kCompLevelMax,
  kCompLevelSimple,
};
std::ostream& operator<<(std::ostream& os, CompLevel comp_level);

enum LZPType {
  kLZPTypeAuto,
  kLZPTypeEnable,
  kLZPTypeDisable,
};

class CompressionOptions {
public:
  static const size_t kDefaultMemUsage = 6;
  static const CompLevel kDefaultLevel = kCompLevelMid;
  static const FilterType kDefaultFilter = kFilterTypeAuto;
  static const LZPType kDefaultLZPType = kLZPTypeAuto;

public:
  size_t mem_usage_ = kDefaultMemUsage;
  CompLevel comp_level_ = kDefaultLevel;
  FilterType filter_type_ = kDefaultFilter;
  LZPType lzp_type_ = kDefaultLZPType;
  std::string dict_file_;
  std::string out_dict_file_;
};

// File headers are stored in a list of blocks spread out through data.
namespace Archive {
  class Header {
  public:
    static const size_t kCurMajorVersion = 0;
    static const size_t kCurMinorVersion = 84;
    static const size_t kMagicStringLength = 10;

    static const char* getMagic() {
      return "MCMARCHIVE";
    }
    Header();
    void read(InStream* stream);
    void write(OutStream* stream);
    bool isArchive() const;
    bool isSameVersion() const;
    uint16_t majorVersion() const {
      return major_version_;
    }
    uint16_t minorVersion() const {
      return minor_version_;
    }

  private:
    char magic_[10]; // MCMARCHIVE
    uint16_t major_version_ = kCurMajorVersion;
    uint16_t minor_version_ = kCurMinorVersion;
  };

  class Algorithm {
  public:
    Algorithm() {}
    Algorithm(const CompressionOptions& options, Detector::Profile profile);
    Algorithm(InStream* stream);
    // Freq is the approximate distribution of input frequencies for the compressor.
    Compressor* CreateCompressor(const FrequencyCounter<256>& freq);
    void read(InStream* stream);
    void write(OutStream* stream);
    Filter* createFilter(Stream* stream, Analyzer* analyzer, CompressionOptions& opts, size_t opt_var, size_t* opt_vars);
    Detector::Profile profile() const {
      return profile_;
    }

  private:
    uint8_t mem_usage_;
    CompressorType algorithm_;
    bool lzp_enabled_;
    FilterType filter_;
    Detector::Profile profile_;
  };

  class SolidBlock {
  public:
    Algorithm algorithm_;
    std::vector<FileSegmentStream::FileSegments> segments_;
    // Not stored, obtianed from segments.
    uint64_t total_size_ = 0u;

    SolidBlock() = default;
    SolidBlock(const Algorithm& algorithm) : algorithm_(algorithm) {}
    void write(OutStream* stream);
    void read(InStream* stream);
  };

  class Blocks : public std::vector<std::unique_ptr<SolidBlock>> {
  public:
    void write(OutStream* stream);
    void read(InStream* stream);
  };
};

class Archiver {
public:
  // Compression.
  Archiver(Stream* stream, const CompressionOptions& options);

  CompressionOptions& Options() {
    return options_;
  }

  bool setOpt(size_t var) {
    opt_var_ = var;
    return true;
  }

  bool setOpts(size_t* vars) {
    opt_vars_ = vars;
    setOpt(vars[0]);
    return true;
  }

  void writeBlocks();

  // Analyze and compress. Returns how many bytes wre compressed.
  uint64_t compress(const std::vector<FileInfo>& in_files);

  size_t* opt_vars_ = nullptr;
private:
  /*Out*/Stream* stream_;
  Archive::Header header_;
  CompressionOptions options_;
  size_t opt_var_;
  FileList files_;  // File list.
  Archive::Blocks blocks_;  // Solid blocks.
};

class Unarchiver {
public:
  // Decompression.
  Unarchiver(Stream* stream);

  const Archive::Header& getHeader() const {
    return header_;
  }

  CompressionOptions& Options() {
    return options_;
  }

  bool setOpt(size_t var) {
    opt_var_ = var;
    return true;
  }

  bool setOpts(size_t* vars) {
    opt_vars_ = vars;
    setOpt(vars[0]);
    return true;
  }

  void readBlocks();

  // Decompress.
  void decompress(const std::string& out_dir, bool verify = false);

  // List files and info.
  void list();

  size_t* opt_vars_ = nullptr;
private:
  /*In*/Stream* stream_;
  Archive::Header header_;
  CompressionOptions options_;
  size_t opt_var_;
  FileList files_;  // File list.
  Archive::Blocks blocks_;  // Solid blocks.
};

#endif
