#ifndef MYNTEYE_INTERNAL_STRINGS_H_  // NOLINT
#define MYNTEYE_INTERNAL_STRINGS_H_
#pragma once

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

class strings_error : public std::runtime_error {
 public:
  explicit strings_error(const std::string &what_arg) noexcept
      : std::runtime_error(std::move(what_arg)) {}
  explicit strings_error(const char *what_arg) noexcept
      : std::runtime_error(std::move(what_arg)) {}
};

namespace strings {

int hex2int(const std::string &text);

bool starts_with(const std::string &text, const std::string &prefix);

std::vector<std::string> split(
    const std::string &text, const std::string &delimiters);

void ltrim(std::string &s);  // NOLINT
void rtrim(std::string &s);  // NOLINT
void trim(std::string &s);   // NOLINT

std::string trim_copy(const std::string &text);

}  // namespace strings

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_STRINGS_H_ NOLINT
