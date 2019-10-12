// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "mynteye/device/types.h"

#include <algorithm>
#include <iomanip>
#include <sstream>

#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

std::string Version::to_string() const {
  std::stringstream s;
  s << static_cast<int>(major_) << "." << static_cast<int>(minor_);
  return s.str();
}

std::vector<std::string> Version::split(const std::string &s) {
  std::vector<std::string> result;
  auto e = s.end();
  auto i = s.begin();
  while (i != e) {
    i = std::find_if_not(i, e, [](char c) { return c == '.'; });
    if (i == e)
      break;
    auto j = std::find(i, e, '.');
    result.emplace_back(i, j);
    i = j;
  }
  return result;
}

Version::value_t Version::parse_part(const std::string &name, size_t part) {
  if (name.empty()) return 0;
  return std::stoi(split(name)[part]);
}

std::string Type::to_string() const {
  std::stringstream s;
  s << std::hex << std::uppercase << std::setfill('0') << std::setw(2)
    << vendor_ << std::setfill('0') << std::setw(2) << product_;
  return s.str();
}

Type::value_t Type::parse_part(
    const std::string &name, size_t pos, size_t count) {
  return std::stoi(name.substr(pos, count), 0, 16);
}

std::string ISPVersion::to_string() const {
  std::stringstream s;
  if (major() == 0x01) {
    s << "S2000-";
  } else if (major() == 0x02) {
    s << "S2110-";
  } else if (major() == 0x03) {
    s << "S210A-";
  } else if (major() == 0x04) {
    s << "S2X0C-";
  } else if (major() == 0x05) {
    s << "S200B-";
  }
  s << static_cast<int>(minor());
  return s.str();
}

MYNTEYE_END_NAMESPACE
