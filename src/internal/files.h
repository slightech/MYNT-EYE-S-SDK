#ifndef MYNTEYE_INTERNAL_FILES_H_  // NOLINT
#define MYNTEYE_INTERNAL_FILES_H_
#pragma once

#include <string>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

namespace files {

bool mkdir(const std::string &path);

}  // namespace files

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_FILES_H_ NOLINT
