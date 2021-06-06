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

#include <algorithm>     // for max, copy
#include <fstream>       // for ifstream, ostringstream, basic_ostream, ios_base
#include <iomanip>       // for operator<<, setfill, setw
#include <system_error>  // for system_error, system_category

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

std::string errstr(int err) {
  return std::system_error(err, std::system_category()).what();
}

uint64_t computeRate(uint64_t size, uint64_t delta_time) {
  if (delta_time == 0) {
    return 0;
  }
  double seconds = static_cast<double>(delta_time) / static_cast<double>(CLOCKS_PER_SEC);
  return static_cast<uint64_t>(size / seconds);
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
