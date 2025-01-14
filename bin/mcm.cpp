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

#include <algorithm>               // for copy_n, max, min, copy
#include <chrono>                  // for steady_clock, seconds, duration_cast, dur...
#include <iomanip>                 // for operator<<, setprecision
#include <iostream>                // for operator<<, basic_ostream, endl, basic_os...
#include <limits>                  // for numeric_limits
#include <numeric>                 // for accumulate
#include <random>                  // for random_device, uniform_int_distribution
#include <string>                  // for operator==, operator<<, string, char_traits
#include <vector>                  // for vector

#include <cassert>                 // for assert
#include <cstddef>                 // for size_t
#include <cstdint>                 // for uint64_t, uint32_t, uint8_t

#include <libmcm/Compressor.hpp>   // for Compressor (ptr only)

#include "compressors/LZ.hpp"      // for LZ16, SimpleEncoder
#include "compressors/LZ-inl.hpp"  // for LZ16::compress, LZ16::decompress

#include "Archive.hpp"             // for CompressionOptions, Archiver, Unarchiver, Archive::Header
#include "File.hpp"                // for FileInfo, File
#include "MatchFinder.hpp"         // for FastMatchFinder, MemoryMatchFinder
#include "Stream.hpp"              // for VoidWriteStream
#include "Tests.hpp"               // for RunAllTests
#include "Util.hpp"                // for formatNumber

static constexpr bool kReleaseBuild = false;

// TODO<C++17>: Use std::filesystem
std::string trimDir(const std::string& str) {
  return str.substr(0, str.length() - (str.back() == '\\' || str.back() == '/'));
}

// TODO<C++17>: Use std::filesystem
std::string trimExt(const std::string& str) {
  std::streamsize start = 0, pos;
  if ((pos = str.find_last_of('\\')) != std::string::npos) {
    start = std::max(start, pos + 1);
  }
  if ((pos = str.find_last_of('/')) != std::string::npos) {
    start = std::max(start, pos + 1);
  }
  return str.substr(static_cast<uint32_t>(start));
}

template <typename T>
static void ReplaceSubstring(T* data, size_t old_pos, size_t len, size_t new_pos, size_t cur_len) {
  if (old_pos == new_pos) {
    return;
  }
  std::vector<T> temp(len);
  // Delete cur and reinsert.
  std::copy(&data[old_pos], &data[old_pos + len], &temp[0]);
  cur_len -= len;
  std::move(&data[old_pos + len], &data[old_pos + len + cur_len], &data[old_pos]);
  // Reinsert.
  new_pos = new_pos % (cur_len + 1);
  std::move(&data[new_pos], &data[new_pos + cur_len], &data[new_pos + len]);
  std::copy(&temp[0], &temp[len], &data[new_pos]);
}

static void printHeader() {
  std::cout
    << "======================================================================" << std::endl
    << "mcm compressor v" << Archive::Header::kCurMajorVersion << "." << Archive::Header::kCurMinorVersion
    << ", by Mathieu Chartier (c)2016 Google Inc." << std::endl
    << "Experimental, may contain bugs. Contact mathieu.a.chartier@gmail.com" << std::endl
    << "Special thanks to: Matt Mahoney, Stephan Busch, Christopher Mattern." << std::endl
    << "======================================================================" << std::endl;
}

class Options {
public:
  // Block size of 0 -> file size / #threads.
  static const uint64_t kDefaultBlockSize = 0;
  enum Mode {
    kModeUnknown,
    // Compress -> Decompress -> Verify.
    // (File or directory).
    kModeTest,
    // Compress infinite times with different opt vars.
    kModeOpt,
    // In memory test.
    kModeMemTest,
    // Single file test.
    kModeSingleTest,
    // Add a single file.
    kModeAdd,
    kModeExtract,
    kModeExtractAll,
    // Single hand mode.
    kModeCompress,
    kModeDecompress,
    // List & other
    kModeList,
  };
  Mode mode = kModeUnknown;
  bool opt_mode = false;
  CompressionOptions options_;
  Compressor* compressor = nullptr;
  uint32_t threads = 1;
  uint64_t block_size = kDefaultBlockSize;
  FileInfo archive_file;
  std::vector<FileInfo> files;
  const std::string kDictArg = "-dict=";
  const std::string kOutDictArg = "-out-dict=";
  std::string dict_file;

  int usage(const std::string& name) {
    printHeader();
    std::cout
      << "Caution: Experimental, use only for testing!" << std::endl
      << "Usage: " << name << " [commands] [options] <infile|dir> <outfile>(default infile.mcm)" << std::endl
      << "Options: d for decompress" << std::endl
      << "-{t|f|m|h|x}{1 .. 11} compression option" << std::endl
      << "t is turbo, f is fast, m is mid, h is high, x is max (default " << CompressionOptions::kDefaultLevel << ")" << std::endl
      << "0 .. 11 specifies memory with 32mb .. 5gb per thread (default " << CompressionOptions::kDefaultMemUsage << ")" << std::endl
      << "10 and 11 are only supported on 64 bits" << std::endl
      << "-test tests the file after compression is done" << std::endl
      // << "-b <mb> specifies block size in MB" << std::endl
      // << "-t <threads> the number of threads to use (decompression requires the same number of threads" << std::endl
      << "Examples:" << std::endl
      << "Compress: " << name << " -m9 enwik8 enwik8.mcm" << std::endl
      << "Decompress: " << name << " d enwik8.mcm enwik8.ref" << std::endl;
    return 0;
  }

  int parse(int argc, char* argv[]) {
    assert(argc >= 1);
    std::string program(trimExt(argv[0]));
    // Parse options.
    int i = 1;
    bool has_comp_args = false;
    for (;i < argc;++i) {
      const std::string arg(argv[i]);
      Mode parsed_mode = kModeUnknown;
      if (arg == "-test") parsed_mode = kModeSingleTest; // kModeTest;
      else if (arg == "-memtest") parsed_mode = kModeMemTest;
      else if (arg == "-opt") parsed_mode = kModeOpt;
      else if (arg == "-stest") parsed_mode = kModeSingleTest;
      else if (arg == "c") parsed_mode = kModeCompress;
      else if (arg == "l") parsed_mode = kModeList;
      else if (arg == "d") parsed_mode = kModeDecompress;
      else if (arg == "a") parsed_mode = kModeAdd;
      else if (arg == "e") parsed_mode = kModeExtract;
      else if (arg == "x") parsed_mode = kModeExtractAll;
      if (parsed_mode != kModeUnknown) {
        if (mode != kModeUnknown) {
          std::cerr << "Multiple commands specified" << std::endl;
          return 2;
        }
        mode = parsed_mode;
        switch (mode) {
          case kModeAdd:
          case kModeExtract:
          case kModeExtractAll: {
            if (++i >= argc) {
              std::cerr << "Expected archive" << std::endl;
              return 3;
            }
            // Archive is next.
            archive_file = FileInfo(argv[i]);
            break;
          }
        }
      } else if (arg == "-opt") opt_mode = true;
      else if (arg == "-filter=none") options_.filter_type_ = kFilterTypeNone;
      else if (arg == "-filter=dict") options_.filter_type_ = kFilterTypeDict;
      else if (arg == "-filter=x86") options_.filter_type_ = kFilterTypeX86;
      else if (arg == "-filter=auto") options_.filter_type_ = kFilterTypeAuto;
      else if (arg.substr(0, std::min(kDictArg.length(), arg.length())) == kDictArg) {
        options_.dict_file_ = arg.substr(kDictArg.length());
      } else if (arg.substr(0, std::min(kOutDictArg.length(), arg.length())) == kOutDictArg) {
        options_.out_dict_file_ = arg.substr(kOutDictArg.length());
      } else if (arg == "-lzp=auto") options_.lzp_type_ = kLZPTypeAuto;
      else if (arg == "-lzp=true") options_.lzp_type_ = kLZPTypeEnable;
      else if (arg == "-lzp=false") options_.lzp_type_ = kLZPTypeDisable;
      else if (arg == "-b") {
        if (i + 1 >= argc) {
          return usage(program);
        }
        std::istringstream iss(argv[++i]);
        iss >> block_size;
        block_size *= MB;
        if (!iss.good()) {
          return usage(program);
        }
      } else if (arg == "-store") {
        options_.comp_level_ = kCompLevelStore;
        has_comp_args = true;
      } else if (arg[0] == '-') {
        if (arg[1] == 't') options_.comp_level_ = kCompLevelTurbo;
        else if (arg[1] == 'f') options_.comp_level_ = kCompLevelFast;
        else if (arg[1] == 'm') options_.comp_level_ = kCompLevelMid;
        else if (arg[1] == 'h') options_.comp_level_ = kCompLevelHigh;
        else if (arg[1] == 'x') options_.comp_level_ = kCompLevelMax;
        else if (arg[1] == 's') options_.comp_level_ = kCompLevelSimple;
        else {
          std::cerr << "Unknown option " << arg << std::endl;
          return 4;
        }
        has_comp_args = true;
        const std::string mem_string = arg.substr(2);
        if (mem_string == "0") options_.mem_usage_ = 0;
        else if (mem_string == "1") options_.mem_usage_ = 1;
        else if (mem_string == "2") options_.mem_usage_ = 2;
        else if (mem_string == "3") options_.mem_usage_ = 3;
        else if (mem_string == "4") options_.mem_usage_ = 4;
        else if (mem_string == "5") options_.mem_usage_ = 5;
        else if (mem_string == "6") options_.mem_usage_ = 6;
        else if (mem_string == "7") options_.mem_usage_ = 7;
        else if (mem_string == "8") options_.mem_usage_ = 8;
        else if (mem_string == "9") options_.mem_usage_ = 9;
        else if (mem_string == "10" || mem_string == "11") {
          if (sizeof(void*) < 8) {
            std::cerr << arg << " is only supported with 64 bit" << std::endl;
            return usage(program);
          }
          options_.mem_usage_ = (mem_string == "10") ? 10 : 11;
        } else if (!mem_string.empty()) {
          std::cerr << "Unknown mem level " << mem_string << std::endl;
          return 4;
        }
      } else if (!arg.empty()) {
        if (mode == kModeAdd || mode == kModeExtract) {
          files.push_back(FileInfo(argv[i]));  // Read in files.
        } else {
          break;  // Done parsing.
        }
      }
    }
    if (mode == kModeUnknown) {
      const int remaining_args = argc - i;
      // No args, need to figure out what to do:
      // decompress: mcm <archive> 
      // TODO add files to archive: mcm <archive> <files>
      // create archive: mcm <files>
      mode = kModeCompress;
      if (!has_comp_args && remaining_args == 1) {
        // Try to open file.
        File fin;
        if (fin.open(argv[i], std::ios_base::in | std::ios_base::binary) == 0) {
          Unarchiver archive(&fin);
          const auto& header = archive.getHeader();
          if (header.isArchive()) {
            mode = kModeDecompress;
          }
        }
      }
    }
    const bool single_file_mode =
      mode == kModeCompress || mode == kModeDecompress || mode == kModeSingleTest ||
      mode == kModeMemTest || mode == kModeOpt || kModeList;
    if (single_file_mode && i < argc) {
      std::string in_file, out_file;
      // Read in file and outfile.
      in_file = argv[i++];
      if (i < argc) {
        out_file = argv[i++];
      } else {
        if (mode == kModeDecompress) {
          out_file = in_file + ".decomp";
        } else {
          out_file = trimDir(in_file) + ".mcm";
        }
      }
      if (mode == kModeMemTest) {
        // No out file for memtest.
        files.push_back(FileInfo(trimDir(in_file)));
      } else if (mode == kModeCompress || mode == kModeSingleTest || mode == kModeOpt) {
        archive_file = FileInfo(trimDir(out_file));
        files.push_back(FileInfo(trimDir(in_file)));
      } else {
        archive_file = FileInfo(trimDir(in_file));
        files.push_back(FileInfo(trimDir(out_file)));
      }
    }
    if (mode != kModeMemTest &&
      (archive_file.getName().empty() || (files.empty() && mode != kModeList))) {
      std::cerr << "Error, input or output files missing" << std::endl;
      usage(program);
      return 5;
    }
    return 0;
  }
};

extern void RunBenchmarks();
int main(int argc, char* argv[]) {
  if (!kReleaseBuild) {
    RunAllTests();
  }
  Options options;
  auto ret = options.parse(argc, argv);
  if (ret) {
    std::cerr << "Failed to parse arguments" << std::endl;
    return ret;
  }
  switch (options.mode) {
  case Options::kModeMemTest: {
    constexpr size_t kCompIterations = kIsDebugBuild ? 1 : 1;
    constexpr size_t kDecompIterations = kIsDebugBuild ? 1 : 25;
    // Read in the whole file.
    std::vector<uint64_t> lengths;
    uint64_t long_length = 0;
    for (const auto& file : options.files) {
      File f(file.getName());
      lengths.push_back(f.length());
      long_length += lengths.back();
    }
    auto length = static_cast<size_t>(long_length);
    check(length < 300 * MB);
    auto* in_buffer = new uint8_t[length];
    // Read in the files.
    uint32_t index = 0;
    uint64_t read_pos = 0;
    for (const auto& file : options.files) {
      File f(file.getName(), std::ios_base::in | std::ios_base::binary);
      size_t count = f.read(in_buffer + read_pos, static_cast<size_t>(lengths[index]));
      check(count == lengths[index]);
      index++;
    }
    // Create the memory compressor.
    typedef SimpleEncoder<8, 16> Encoder;
    auto* compressor = new LZ16<FastMatchFinder<MemoryMatchFinder>>;
    auto out_buffer = new uint8_t[compressor->getMaxExpansion(length)];
    const auto comp_start = std::chrono::high_resolution_clock::now();
    uint32_t comp_size;
    static const bool opt_mode = false;
    if (opt_mode) {
      uint32_t best_size = 0xFFFFFFFF;
      uint32_t best_opt = 0;
      std::ofstream opt_file("opt_result.txt");
      for (uint32_t opt = 0; ; ++opt) {
        compressor->setOpt(opt);
        comp_size = compressor->compress(in_buffer, out_buffer, length);
        opt_file << "opt " << opt << " = " << comp_size << std::endl << std::flush;
        std::cout << "Opt " << opt << " / " << best_opt << " =  " << comp_size << "/" << best_size << std::endl;
        if (comp_size < best_size) {
          best_opt = opt;
          best_size = comp_size;
        }
      }
    } else {
      for (uint32_t i = 0; i < kCompIterations; ++i) {
        comp_size = compressor->compress(in_buffer, out_buffer, length);
      }
    }

    const auto comp_end = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double, std::ratio<1>> comp_time = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(comp_end - comp_start);
    std::cout << "Compression took: " << comp_time.count() << "s reduction: " << length << " -> " << comp_size << " = " << float(double(length) / double(comp_size)) << " rate: "
      << prettySize(static_cast<uint64_t>(long_length * kCompIterations / comp_time.count())) << "/s" << std::endl;
    std::fill(in_buffer, in_buffer + length, 0);
    const auto decomp_start = std::chrono::high_resolution_clock::now();
    for (uint32_t i = 0; i < kDecompIterations; ++i) {
      compressor->decompress(out_buffer, in_buffer, length);
    }
    const auto decomp_end = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double, std::ratio<1>> decomp_time = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(decomp_end - decomp_start);
    std::cout << "Decompression took: " << decomp_time.count() << "s rate: "
      << prettySize(static_cast<uint64_t>(long_length * kDecompIterations / decomp_time.count())) << "/s" << std::endl;
    index = 0;
    for (const auto& file : options.files) {
      File f(file.getName(), std::ios_base::in | std::ios_base::binary);
      const auto count = static_cast<uint32_t>(f.read(out_buffer, static_cast<uint32_t>(lengths[index])));
      check(count == lengths[index]);
      for (uint32_t i = 0; i < count; ++i) {
        if (out_buffer[i] != in_buffer[i]) {
          std::cerr << "File" << file.getName() << " doesn't match at byte " << i << std::endl;
          check(false);
        }
      }
      index++;
    }
    std::cout << "Decompression verified" << std::endl;
    break;
  }
  case Options::kModeSingleTest:
  case Options::kModeOpt:
  case Options::kModeCompress:
  case Options::kModeTest: {
    printHeader();

    int err = 0;

    std::string out_file = options.archive_file.getName();
    File fout;

    if (options.mode == Options::kModeOpt) {
      std::cout << "Optimizing" << std::endl;
      uint64_t best_size = std::numeric_limits<uint64_t>::max();
      size_t best_var = 0;
      std::ofstream opt_file("opt_result.txt");
      // static const size_t kOpts = 10624;
      static const size_t kOpts = 3;
      // size_t opts[kOpts] = {0,1,2,3,15,14,4,6,7,8,9,17,12,11,13,5,10,18,20,19,21,26,22,28,23,24,16,25,27,29,31,32,36,33,34,35,37,30,38,39,};
      // size_t opts[kOpts] = {}; for (size_t i = 0; i < kOpts; ++i) opts[i] = i;
      size_t opts[kOpts] = {};
      //size_t opts[] = {7,14,1,12,3,4,11,15,9,16,5,6,18,13,19,30,45,20,21,22,23,17,8,2,26,10,32,43,36,35,42,29,34,24,25,37,31,33,39,38,0,41,28,40,44,58,46,59,92,27,60,61,91,63,95,47,64,124,94,62,93,96,123,125,72,69,65,67,83,68,66,73,82,70,80,76,71,81,77,87,78,74,79,84,75,48,49,50,51,52,53,54,55,56,57,86,88,97,98,99,100,85,101,90,103,104,89,105,107,102,108,109,110,111,106,113,112,114,115,116,119,118,120,121,117,122,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,151,144,145,146,147,148,149,150,152,153,155,156,157,154,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,226,239,227,228,229,230,231,232,233,234,235,236,237,238,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255,};
      size_t best_opts[kOpts] = {};
      size_t cur_index = 0;
      size_t len = 1;
      // size_t kMaxIndex = 128 - len;
      // size_t kMaxIndex = cm::kModelCount;
      size_t kMaxIndex = 12345;
      size_t bads = 0;
      size_t best_cur = 0;
      static constexpr size_t kAvgCount = 2;
      std::chrono::high_resolution_clock::duration total[kAvgCount] = {};
      size_t count[kAvgCount] = {};
      std::chrono::high_resolution_clock::duration min_time{std::numeric_limits<std::chrono::high_resolution_clock::rep>::max()};
      const bool kPerm = false;
      std::random_device randomness;
      std::uniform_int_distribution<size_t> index_increments{1, kMaxIndex};
      if (kPerm) {
        for (size_t i = 0;; ++i) {
          if ((i & 255) == 255 && false) {
            len -= len > 1;
            kMaxIndex = 128 - len;
          }
          auto a = i % kMaxIndex;
          auto b = (i + index_increments(randomness)) % kMaxIndex;
          VoidWriteStream fout;
          Archiver archive(&fout, options.options_);
          if (i != 0) {
            ReplaceSubstring(opts, a, len, b, kOpts);
          }
          if (!archive.setOpts(opts)) {
            std::cerr << "Failed to set opts" << std::endl;
            continue;
          }
          uint64_t in_bytes = archive.compress(options.files);
          if (in_bytes == 0) continue;
          const auto size = fout.tellp();
          std::cout << i << ": swap " << a << " to " << b << " " << size << std::endl;
          if (size < best_size) {
            std::cout << "IMPROVEMENT " << i << ": " << size << std::endl;
            opt_file << i << ": " << size << " ";
            for (auto opt : opts) opt_file << opt << ",";
            opt_file << std::endl;
            std::copy_n(opts, kOpts, best_opts);
            best_size = size;
          } else {
            std::copy_n(best_opts, kOpts, opts);
          }
        }
      } else {
        for (auto o : opts) check(o <= kMaxIndex);
        for (size_t i = 0;; ++i) {
          const auto start = std::chrono::high_resolution_clock::now();
          VoidWriteStream fout;
          Archiver archive(&fout, options.options_);
          if (!archive.setOpts(opts)) {
            continue;
          }
          uint64_t in_bytes = archive.compress(options.files);
          if (in_bytes == 0) continue;
          const auto time = std::chrono::high_resolution_clock::now() - start;
          total[i % kAvgCount] += time;
          min_time = std::min(min_time, time);
          const auto size = fout.tellp();
          opt_file << "opts ";
          for (auto opt : opts) opt_file << opt << ",";
          ++count[i % kAvgCount];
          auto before_index = cur_index;
          auto before_opt = opts[before_index];
          if (size < best_size) {
            best_size = size;
            std::copy_n(opts, kOpts, best_opts);
            best_var = opts[cur_index];
            bads = 0;
          } 
          if (opts[cur_index] >= kMaxIndex) {
            std::copy_n(best_opts, kOpts, opts);
            cur_index = (cur_index + 1) % kOpts;
            opts[cur_index] = 0;
          } else {
            ++opts[cur_index];
          }

          std::ostringstream ss;
          std::chrono::high_resolution_clock::duration avgs[kAvgCount] = {};
          for (size_t i = 0; i < kAvgCount; ++i) {
            if (count[i] != 0) avgs[i] = total[i] / count[i];
          }
          auto avg = std::accumulate(total, total + kAvgCount, std::chrono::high_resolution_clock::duration(0)) / std::accumulate(count, count + kAvgCount, 0u);
          const std::chrono::duration<double, std::ratio<1>> time_secs = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(time);
          const std::chrono::duration<double, std::ratio<1>> avg_secs = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(avg);
          ss << " -> " << formatNumber(size) << " best " << best_var << " in " << time_secs.count() << "s avg "
             << avg_secs.count() << "(";
          for (auto& d : avgs) ss << d.count() << ",";
          ss << ") min " << min_time.count();

          opt_file << ss.str() << std::endl << std::flush;

          std::cout << "opt[" << before_index << "]=" << before_opt << " best=" << best_var << "(" << formatNumber(best_size) << ") "
            << formatNumber(in_bytes) << ss.str() << std::endl;
        }
      }
    } else {
      const auto start = std::chrono::high_resolution_clock::now();
      if (err = fout.open(out_file, std::ios_base::out | std::ios_base::binary)) {
        std::cerr << "Error opening: " << out_file << " (" << errstr(err) << ")" << std::endl;
        return 2;
      }

      std::cout << "Compressing to " << out_file << " mode=" << options.options_.comp_level_ << " mem=" << options.options_.mem_usage_ << std::endl;
      Archiver archive(&fout, options.options_);
      uint64_t in_bytes = archive.compress(options.files);
      const std::chrono::duration<double, std::ratio<1>> time =  std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::high_resolution_clock::now() - start);
      std::cout << "Done compressing " << formatNumber(in_bytes) << " -> " << formatNumber(fout.tellp())
        << " in " << std::setprecision(3) << time.count() << "s"
        << " bpc=" << double(fout.tellp()) * 8.0 / double(in_bytes) << std::endl;

      fout.close();

      if (options.mode == Options::kModeSingleTest) {
        if (err = fout.open(out_file, std::ios_base::in | std::ios_base::binary)) {
          std::cerr << "Error opening: " << out_file << " (" << errstr(err) << ")" << std::endl;
          return 1;
        }
        Unarchiver archive(&fout);
        archive.list();
        std::cout << "Verifying archive decompression" << std::endl;
        archive.decompress("", true);
      }
    }
    break;
  }
  case Options::kModeAdd: {
    // Add a single file.
    break;
  }
  case Options::kModeList: {
    auto in_file = options.archive_file.getName();
    File fin;
    int err = 0;
    if (err = fin.open(in_file, std::ios_base::in | std::ios_base::binary)) {
      std::cerr << "Error opening: " << in_file << " (" << errstr(err) << ")" << std::endl;
      return 1;
    }
    printHeader();
    std::cout << "Listing files in archive " << in_file << std::endl;
    Unarchiver archive(&fin);
    const auto& header = archive.getHeader();
    if (!header.isArchive()) {
      std::cerr << "Attempting to open non mcm compatible file" << std::endl;
      return 1;
    }
    if (!header.isSameVersion()) {
      std::cerr << "Attempting to open old version " << header.majorVersion() << "." << header.minorVersion() << std::endl;
      return 1;
    }
    archive.list();
    fin.close();
    break;
  }
  case Options::kModeDecompress: {
    auto in_file = options.archive_file.getName();
    File fin;
    File fout;
    int err = 0;
    if (err = fin.open(in_file, std::ios_base::in | std::ios_base::binary)) {
      std::cerr << "Error opening: " << in_file << " (" << errstr(err) << ")" << std::endl;
      return 1;
    }
    printHeader();
    std::cout << "Decompresing archive " << in_file << std::endl;
    Unarchiver archive(&fin);
    const auto& header = archive.getHeader();
    if (!header.isArchive()) {
      std::cerr << "Attempting to decompress non archive file" << std::endl;
      return 1;
    }
    if (!header.isSameVersion()) {
      std::cerr << "Attempting to decompress other version " << header.majorVersion() << "." << header.minorVersion() << std::endl;
      return 1;
    }
    // archive.decompress(options.files.back().getName());
    archive.decompress("");
    fin.close();
    // Decompress the single file in the archive to the output out.
    break;
  }
  case Options::kModeExtract: {
    // Extract a single file from multi file archive.
    break;
  }
  case Options::kModeExtractAll: {
    // Extract all the files in the archive.
    // archive.ExtractAll();
    break;
  }
  }
  return 0;
}
