#ifndef MYNTEYE_INTERNAL_DL_H_  // NOLINT
#define MYNTEYE_INTERNAL_DL_H_
#pragma once

#include "mynteye/mynteye.h"

#if defined(OS_WIN) && !defined(OS_MINGW) && !defined(OS_CYGWIN)
#include <Windows.h>
#else
#include <dlfcn.h>
#endif

MYNTEYE_BEGIN_NAMESPACE

#if defined(OS_WIN) && !defined(OS_MINGW) && !defined(OS_CYGWIN)
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

#endif  // MYNTEYE_INTERNAL_DL_H_  NOLINT
