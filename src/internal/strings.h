#ifndef MYNTEYE_INTERNAL_STRINGS_H_  // NOLINT
#define MYNTEYE_INTERNAL_STRINGS_H_
#pragma once

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

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
std::vector<std::string> split(
    const std::string &text, const std::string &delimiters);

MYNTEYE_API void ltrim(std::string &s);  // NOLINT
MYNTEYE_API void rtrim(std::string &s);  // NOLINT
MYNTEYE_API void trim(std::string &s);   // NOLINT

MYNTEYE_API
std::string trim_copy(const std::string &text);

}  // namespace strings

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_STRINGS_H_ NOLINT
