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
#include "mynteye/api/dl.h"

#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

#if defined(MYNTEYE_OS_WIN) && !defined(MYNTEYE_OS_MINGW) && \
    !defined(MYNTEYE_OS_CYGWIN)

namespace {

// How to get the error message from the error code returned by GetLastError()?
//   https://stackoverflow.com/questions/9272415/how-to-convert-dword-to-char
std::string GetLastErrorAsString() {
  DWORD dw = ::GetLastError();
  if (dw == 0)
    return std::string();
  LPSTR lpMsgBuf;
  size_t size = FormatMessageA(
      FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
          FORMAT_MESSAGE_IGNORE_INSERTS,
      NULL, dw, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&lpMsgBuf, 0,
      NULL);
  std::string message(lpMsgBuf, size);
  LocalFree(lpMsgBuf);
  return message;
}

}  // namespace

#endif

DL::DL() : handle(nullptr) {
  VLOG(2) << __func__;
}

DL::DL(const char *filename) : handle(nullptr) {
  VLOG(2) << __func__;
  Open(filename);
}

DL::~DL() {
  VLOG(2) << __func__;
  Close();
}

bool DL::Open(const char *filename) {
  if (handle != nullptr) {
    VLOG(2) << "Already opened, do nothing";
    // Close();
    return false;
  }
#if defined(MYNTEYE_OS_WIN) && !defined(MYNTEYE_OS_MINGW) && \
    !defined(MYNTEYE_OS_CYGWIN)
  handle = LoadLibraryEx(filename, nullptr, 0);
#else
  handle = dlopen(filename, RTLD_LAZY);
#endif
  if (handle == nullptr) {
    VLOG(2) << "Open library failed: " << filename;
    return false;
  } else {
    return true;
  }
}

bool DL::IsOpened() {
  return handle != nullptr;
}

void *DL::Sym(const char *symbol) {
  if (handle == nullptr) {
    VLOG(2) << "Not opened, do nothing";
    return nullptr;
  }
#if defined(MYNTEYE_OS_WIN) && !defined(MYNTEYE_OS_MINGW) && !defined(MYNTEYE_OS_CYGWIN)
  void *f = GetProcAddress(handle, symbol);
  if (f == nullptr) {
    VLOG(2) << "Load symbol failed: " << symbol;
  }
#else
  dlerror();  // reset errors
  void *f = dlsym(handle, symbol);
  const char *error = dlerror();
  if (error != nullptr) {
    VLOG(2) << "Load symbol failed: " << symbol;
    f = nullptr;
  }
#endif
  return f;
}

int DL::Close() {
  int ret = 0;
  if (handle == nullptr) {
    VLOG(2) << "Not opened, do nothing";
  } else {
#if defined(MYNTEYE_OS_WIN) && !defined(MYNTEYE_OS_MINGW) && !defined(MYNTEYE_OS_CYGWIN)
    ret = FreeLibrary(handle) ? 0 : 1;
#else
    ret = dlclose(handle);
#endif
    handle = nullptr;
  }
  return ret;
}

const char *DL::Error() {
#if defined(MYNTEYE_OS_WIN) && !defined(MYNTEYE_OS_MINGW) && !defined(MYNTEYE_OS_CYGWIN)
  return GetLastErrorAsString().c_str();
#else
  return dlerror();
#endif
}

MYNTEYE_END_NAMESPACE
