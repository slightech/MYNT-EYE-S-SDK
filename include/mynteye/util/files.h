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
#ifndef MYNTEYE_UTIL_FILES_H_
#define MYNTEYE_UTIL_FILES_H_
#pragma once

#include <string>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

namespace files {

MYNTEYE_API bool mkdir(const std::string &path);

}  // namespace files

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_UTIL_FILES_H_
