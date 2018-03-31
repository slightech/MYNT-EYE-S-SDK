#ifndef MYNTEYE_INTERNAL_TYPES_H_  // NOLINT
#define MYNTEYE_INTERNAL_TYPES_H_
#pragma once

#include <cstdint>

#include "mynteye/mynteye.h"

#define FOURCC(a, b, c, d)                          \
  ((std::uint32_t)(a) | ((std::uint32_t)(b) << 8) | \
   ((std::uint32_t)(c) << 16) | ((std::uint32_t)(d) << 24))  // NOLINT

MYNTEYE_BEGIN_NAMESPACE

/**
 * @ingroup enumerations
 * @brief Formats define how each stream can be encoded.
 */
enum class Format : std::uint32_t {
  YUYV = FOURCC('Y', 'U', 'Y', 'V'),
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_TYPES_H_ NOLINT
