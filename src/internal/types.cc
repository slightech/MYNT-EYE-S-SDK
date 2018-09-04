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
#include "internal/types.h"

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

MYNTEYE_END_NAMESPACE
