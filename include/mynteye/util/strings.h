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
#ifndef MYNTEYE_UTIL_STRINGS_H_
#define MYNTEYE_UTIL_STRINGS_H_
#pragma once

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

/** The strings error */
class MYNTEYE_API strings_error : public std::runtime_error {
 public:
  explicit strings_error(const std::string &what_arg) noexcept
      : std::runtime_error(std::move(what_arg)) {}
  explicit strings_error(const char *what_arg) noexcept
      : std::runtime_error(std::move(what_arg)) {}
};

namespace strings {

MYNTEYE_API
int hex2int(const std::string &text);

MYNTEYE_API
bool starts_with(const std::string &text, const std::string &prefix);

MYNTEYE_API
bool ends_with(const std::string &text, const std::string &suffix);

MYNTEYE_API
std::vector<std::string> split(
    const std::string &text, const std::string &delimiters);

MYNTEYE_API void ltrim(std::string &s);  // NOLINT
MYNTEYE_API void rtrim(std::string &s);  // NOLINT
MYNTEYE_API void trim(std::string &s);   // NOLINT

MYNTEYE_API
std::string trim_copy(const std::string &text);

}  // namespace strings

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_UTIL_STRINGS_H_
