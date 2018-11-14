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
#ifndef MYNTEYE_API_DL_H_
#define MYNTEYE_API_DL_H_
#pragma once

#include "mynteye/mynteye.h"

#if defined(MYNTEYE_OS_WIN) && !defined(MYNTEYE_OS_MINGW) && \
    !defined(MYNTEYE_OS_CYGWIN)
#include <Windows.h>
#else
#include <dlfcn.h>
#endif

MYNTEYE_BEGIN_NAMESPACE

#if defined(MYNTEYE_OS_WIN) && !defined(MYNTEYE_OS_MINGW) && \
    !defined(MYNTEYE_OS_CYGWIN)
using DLLIB = HMODULE;
#else
using DLLIB = void *;
#endif

// Dynamic loading
//   https://en.wikipedia.org/wiki/Dynamic_loading
// C++ dlopen mini HOWTO
//   http://tldp.org/HOWTO/C++-dlopen/
class MYNTEYE_API DL {
 public:
  DL();
  explicit DL(const char *filename);
  ~DL();

  bool Open(const char *filename);

  bool IsOpened();

  void *Sym(const char *symbol);

  template <typename Func>
  Func *Sym(const char *symbol);

  int Close();

  const char *Error();

 private:
  DLLIB handle;
};

template <typename Func>
Func *DL::Sym(const char *symbol) {
  void *f = Sym(symbol);
  return reinterpret_cast<Func *>(f);
}

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_API_DL_H_
