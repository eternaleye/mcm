/*	MCM file compressor

  Copyright (C) 2016, Google Inc.
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

#include "Util.hpp"

#include <algorithm>  // for max, copy
#include <fstream>    // for ifstream, ostringstream, basic_ostream, ios_base
#include <iomanip>    // for operator<<, setfill, setw

#include <cstring>    // for strerror

bool fileExists(const char* name) {
  std::ifstream fin(name, std::ios_base::in);
  return fin.good();
}

std::string prettySize(uint64_t size) {
  uint64_t units;
  const char* name;
  if (size >= GB) {
    units = GB;
    name = "GB";
  } else if (size >= MB) {
    units = MB;
    name = "MB";
  } else if (size >= KB) {
    units = KB;
    name = "KB";
  } else {
    units = 1;
    name = "B";
  }
  std::ostringstream oss;
  oss << static_cast<float>(static_cast<double>(size) / static_cast<double>(units)) << name;
  return oss.str();
}

double clockToSeconds(clock_t c) {
  return double(c) / double(CLOCKS_PER_SEC);
}

std::string errstr(int err) {
  // char buffer[1024];
  // strerror_s(buffer, sizeof(buffer), err);
  return strerror(err);
}

uint64_t computeRate(uint64_t size, uint64_t delta_time) {
  if (delta_time == 0) {
    return 0;
  }
  double seconds = static_cast<double>(delta_time) / static_cast<double>(CLOCKS_PER_SEC);
  return static_cast<uint64_t>(size / seconds);
}

std::vector<uint8_t> loadFile(const std::string& name, uint32_t max_size) {
  std::vector<uint8_t> ret;
  std::ifstream fin(name.c_str(), std::ios_base::in | std::ios_base::binary);
  for (uint32_t i = 0; i < max_size; ++i) {
    int c = fin.get();
    if (fin.eof()) {
      break;
    }
    ret.push_back(static_cast<uint8_t>(static_cast<uint32_t>(c)));
  }
  return ret;
}

std::string formatNumber(uint64_t n) {
  std::string ret;
  while (n >= 1000) {
    std::ostringstream oss;
    oss << std::setw(3) << std::setfill('0') << n % 1000;
    ret = "," + oss.str() + ret;
    n /= 1000;
  }
  return std::to_string(n) + ret;
}

std::string trimDir(const std::string& str) {
  return str.substr(0, str.length() - (str.back() == '\\' || str.back() == '/'));
}

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

std::string getExt(const std::string& str) {
  for (int i = str.length(); i > 0; --i) {
    auto c = str[i - 1];
    if (c == '\\' || c == '/') break;
    if (c == '.') {
      return str.substr(i);
    }
  }
  return "";
}

std::pair<std::string, std::string> GetFileName(const std::string& str) {
  int i = str.length();
  for (; i > 0; --i) {
    if (str[i - 1] == '\\' || str[i - 1] == '/') {
      return std::pair<std::string, std::string>(str.substr(0, i), str.substr(i));
    }
  }
  // No directory.
  return std::pair<std::string, std::string>("", str);
}

bool IsAbsolutePath(const std::string& path) {
  if (path[0] == '/' || path[0] == '\\') {
    // Unix absolute path path.
    return true;
  }
  if (path.length() >= 3 && path[1] == ':' && (path[2] == '\\' || path[2] == '/')) {
    // Windows absolute path.
    return true;
  }
  return false;
}

void RunUtilTests() {
  check(IsAbsolutePath("/test/asdf"));
  check(IsAbsolutePath("\\test.abc"));
  check(IsAbsolutePath("C:/test/"));
  check(IsAbsolutePath("C:\\test/"));
  check(!IsAbsolutePath(""));
  check(!IsAbsolutePath("test/abc"));
  check(!IsAbsolutePath("test.txt"));
}
