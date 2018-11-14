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
#ifndef MYNTEYE_GLOBAL_H_
#define MYNTEYE_GLOBAL_H_
#pragma once

#ifdef _WIN32
  #define MYNTEYE_OS_WIN
  #ifdef _WIN64
    #define MYNTEYE_OS_WIN64
  #else
    #define MYNTEYE_OS_WIN32
  #endif
  #if defined(__MINGW32__) || defined(__MINGW64__)
    #define MYNTEYE_OS_MINGW
    #ifdef __MINGW64__
      #define MYNTEYE_OS_MINGW64
    #else
      #define MYNTEYE_OS_MINGW32
    #endif
  #elif defined(__CYGWIN__) || defined(__CYGWIN32__)
    #define MYNTEYE_OS_CYGWIN
  #endif
#elif __APPLE__
  #include <TargetConditionals.h>
  #if TARGET_IPHONE_SIMULATOR
    #define MYNTEYE_OS_IPHONE
    #define MYNTEYE_OS_IPHONE_SIMULATOR
  #elif TARGET_OS_IPHONE
    #define MYNTEYE_OS_IPHONE
  #elif TARGET_OS_MAC
    #define MYNTEYE_OS_MAC
  #else
    #error "Unknown Apple platform"
  #endif
#elif __ANDROID__
  #define MYNTEYE_OS_ANDROID
#elif __linux__
  #define MYNTEYE_OS_LINUX
#elif __unix__
  #define MYNTEYE_OS_UNIX
#elif defined(_POSIX_VERSION)
  #define MYNTEYE_OS_POSIX
#else
  #error "Unknown compiler"
#endif

#ifdef MYNTEYE_OS_WIN
#define MYNTEYE_DECL_EXPORT __declspec(dllexport)
#define MYNTEYE_DECL_IMPORT __declspec(dllimport)
#define MYNTEYE_DECL_HIDDEN
#else
#define MYNTEYE_DECL_EXPORT __attribute__((visibility("default")))
#define MYNTEYE_DECL_IMPORT __attribute__((visibility("default")))
#define MYNTEYE_DECL_HIDDEN __attribute__((visibility("hidden")))
#endif

#if defined(MYNTEYE_OS_WIN) && !defined(MYNTEYE_OS_MINGW) && \
  !defined(MYNTEYE_OS_CYGWIN)
#define MYNTEYE_OS_SEP "\\"
#else
#define MYNTEYE_OS_SEP "/"
#endif

#define MYNTEYE_STRINGIFY_HELPER(X) #X
#define MYNTEYE_STRINGIFY(X) MYNTEYE_STRINGIFY_HELPER(X)

#define MYNTEYE_DISABLE_COPY(Class) \
  Class(const Class &) = delete; \
  Class &operator=(const Class &) = delete;

#define MYNTEYE_UNUSED(x) (void)x;

#endif  // MYNTEYE_GLOBAL_H_
