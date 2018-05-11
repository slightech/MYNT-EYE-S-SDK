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
#include "internal/files.h"

#include <glog/logging.h>

#if defined(OS_WIN) && !defined(OS_MINGW) && !defined(OS_CYGWIN)
#include <direct.h>
#else
#include <sys/stat.h>
#endif

#include "internal/strings.h"

MYNTEYE_BEGIN_NAMESPACE

namespace files {

bool _mkdir(const std::string &path) {
#if defined(OS_MINGW) || defined(OS_CYGWIN)
  const int status = ::mkdir(path.c_str());
#elif defined(OS_WIN)
  const int status = ::_mkdir(path.c_str());
#else
  const int status =
      ::mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
  if (status != 0 && errno != EEXIST) {
    VLOG(2) << "Create directory failed (status " << status
            << "), path: " << path;
    return false;
  }
  if (errno == EEXIST) {
    VLOG(2) << "Create directory needless (already exist), path: " << path;
    return true;
  } else {
    VLOG(2) << "Create directory success, path: " << path;
    return true;
  }
}

bool mkdir(const std::string &path) {
  auto &&dirs = strings::split(path, OS_SEP);
  auto &&size = dirs.size();
  if (size <= 0)
    return false;
  std::string p{dirs[0]};
  if (!_mkdir(p))
    return false;
  for (std::size_t i = 1; i < size; i++) {
    p.append(OS_SEP).append(dirs[i]);
    if (!_mkdir(p))
      return false;
  }
  return true;
}

}  // namespace files

MYNTEYE_END_NAMESPACE
