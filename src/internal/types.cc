#include "internal/types.h"

#include <glog/logging.h>

#include <algorithm>
#include <iomanip>
#include <iostream>

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
