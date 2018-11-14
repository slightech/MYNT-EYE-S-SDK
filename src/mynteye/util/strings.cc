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
#include "mynteye/util/strings.h"

#include <algorithm>
#include <cctype>
#include <exception>
#include <locale>

MYNTEYE_BEGIN_NAMESPACE

namespace strings {

namespace {

// The most elegant way to iterate the words of a string
//   https://stackoverflow.com/questions/236129/the-most-elegant-way-to-iterate-the-words-of-a-string

template <class ContainerT>
void tokenize(
    const std::string &str, ContainerT &tokens,  // NOLINT
    const std::string &delimiters = " ", bool trimEmpty = false) {
  std::string::size_type pos, lastPos = 0, length = str.length();

  using value_type = typename ContainerT::value_type;
  using size_type = typename ContainerT::size_type;

  while (lastPos < length + 1) {
    pos = str.find_first_of(delimiters, lastPos);
    if (pos == std::string::npos) {
      pos = length;
    }

    if (pos != lastPos || !trimEmpty) {
      tokens.push_back(
          value_type(str.data() + lastPos, (size_type)pos - lastPos));
    }

    lastPos = pos + 1;
  }
}

}  // namespace

int hex2int(const std::string &text) {
  try {
    return std::stoi(text, nullptr, 16);
  } catch (const std::exception &e) {
    throw new strings_error("strings conversion error");
  }
}

bool starts_with(const std::string &text, const std::string &prefix) {
  return text.compare(0, prefix.length(), prefix) == 0;
}

bool ends_with(const std::string &text, const std::string &suffix) {
  if (suffix.length() > text.length())
    return false;
  return text.compare(
             text.length() - suffix.length(), suffix.length(), suffix) == 0;
}

std::vector<std::string> split(
    const std::string &text, const std::string &delimiters) {
  std::vector<std::string> tokens;
  tokenize(text, tokens, delimiters);
  return tokens;
}

// What's the best way to trim std::string?
//   https://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring

void ltrim(std::string &s) {  // NOLINT
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
            return !std::isspace(ch);
          }));
}

void rtrim(std::string &s) {  // NOLINT
  s.erase(
      std::find_if(
          s.rbegin(), s.rend(), [](int ch) { return !std::isspace(ch); })
          .base(),
      s.end());
}

void trim(std::string &s) {  // NOLINT
  ltrim(s);
  rtrim(s);
}

std::string trim_copy(const std::string &text) {
  std::string s = text;
  trim(s);
  return s;
}

}  // namespace strings

MYNTEYE_END_NAMESPACE
