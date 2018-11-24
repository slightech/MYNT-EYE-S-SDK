# Copyright 2018 Slightech Co., Ltd. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
ifndef _COMMON_DEFS_MAKE_
_COMMON_DEFS_MAKE_ := 1

SHELL := /bin/bash

EMPTY :=
SPACE := $(EMPTY) $(EMPTY)
COMMA := ,
COLON := :
SEMICOLON := ;
QUOTE := "
SINGLE_QUOTE := '
OPEN_PAREN := (
CLOSE_PAREN := )

# Options
#
#   VS_CODE: ignore to auto detect, otherwise specify the version
#            15|2017, 14|2015, 12|2013, 11|2012, 10|2010, 9|2008, 8|2005
#   BUILD_TYPE: Debug|Release
#
# e.g. make [TARGET] VS_CODE=2017 BUILD_TYPE=Debug

BUILD_TYPE ?= Release

# Host detection

ifeq ($(OS),Windows_NT)

HOST_OS := Win

ifeq ($(PROCESSOR_ARCHITEW6432),AMD64)
  HOST_ARCH := x64
else
  ifeq ($(PROCESSOR_ARCHITECTURE),AMD64)
    HOST_ARCH := x64
  else ifeq ($(PROCESSOR_ARCHITECTURE),x86)
    HOST_ARCH := x86
  else
    DUMMY := $(error "Can't detect host arch")
  endif
endif

else

UNAME_S := $(shell uname -s)
ifneq ($(findstring Linux,$(UNAME_S)),)
  HOST_OS := Linux
else ifneq ($(findstring Darwin,$(UNAME_S)),)
  HOST_OS := Mac
else ifneq ($(findstring MINGW,$(UNAME_S)),)
  HOST_OS := Win
else ifneq ($(findstring MSYS,$(UNAME_S)),)
  # Need MSYS on Windows
  HOST_OS := Win
else
  DUMMY := $(error "Can't detect host os")
endif

UNAME_M := $(shell uname -m)
ifneq ($(findstring x86_64,$(UNAME_M)),)
  HOST_ARCH := x64
else ifneq ($(findstring x86,$(UNAME_M)),)
  HOST_ARCH := x86
else ifneq ($(findstring i686,$(UNAME_M)),)
  HOST_ARCH := x86
else ifneq ($(findstring i386,$(UNAME_M)),)
  HOST_ARCH := x86
else ifneq ($(findstring arm,$(UNAME_M)),)
  HOST_ARCH := Arm
else ifneq ($(findstring aarch64,$(UNAME_M)),)
  HOST_ARCH := AArch64
else
  DUMMY := $(error "Can't detect host arch")
endif

endif

HOST_NAME := $(HOST_OS)
ifeq ($(HOST_OS),Win)
  UNAME_S := $(shell uname -s)
  ifneq ($(UNAME_S),)
    ifneq ($(findstring MINGW,$(UNAME_S)),)
      HOST_NAME := MinGW
    else ifneq ($(findstring MSYS,$(UNAME_S)),)
      HOST_NAME := MSYS
    endif
  endif
else ifeq ($(HOST_OS),Linux)
  UNAME_A := $(shell uname -a)
  ifneq ($(findstring tegra,$(UNAME_A)),)
    HOST_NAME := Tegra
  else ifneq ($(findstring jetsonbot,$(UNAME_A)),)
    HOST_NAME := Tegra
  else ifneq ($(findstring firefly,$(UNAME_A)),)
   HOST_NAME := Firefly
  else ifneq ($(findstring ubuntu,$(UNAME_A)),)
   HOST_NAME := Ubuntu
  endif
endif

# Function

mkinfo = $(info + $1)

lower = $(shell echo $1 | tr '[:upper:]' '[:lower:]')

# Command

SH := $(SHELL)
ECHO := echo -e
FIND := $(shell ./scripts/getfind.sh)

ifeq ($(HOST_OS),Win)
  ifeq ($(HOST_NAME),MinGW)
    CC := x86_64-w64-mingw32-gcc
    CXX := x86_64-w64-mingw32-g++
    MAKE := mingw32-make
    BUILD := $(MAKE)
  else
    CC := cl
    CXX := cl
    MAKE := make
    BUILD := msbuild.exe ALL_BUILD.vcxproj /property:Configuration=$(BUILD_TYPE)
  endif
else
  # mac & linux
  # Set realpath for linux because of compiler not found with wrong path when cmake again
  CC := /usr/bin/cc
  CXX := /usr/bin/c++
  MAKE := make
  BUILD := $(MAKE)
endif

ifeq ($(HOST_OS),Mac)
  LDD := otool -L
else
  LDD := ldd
endif

# CMake

CMAKE := cmake
CMAKE := $(CMAKE) -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)
ifneq ($(CC),)
  CMAKE := $(CMAKE) -DCMAKE_C_COMPILER=$(CC)
endif
ifneq ($(CXX),)
  CMAKE := $(CMAKE) -DCMAKE_CXX_COMPILER=$(CXX)
endif
ifneq ($(MAKE),)
  ifeq ($(HOST_OS),Win)
    ifeq ($(HOST_NAME),MinGW)
      CMAKE := $(CMAKE) -DCMAKE_MAKE_PROGRAM=$(MAKE)
    endif
  else
    CMAKE := $(CMAKE) -DCMAKE_MAKE_PROGRAM=$(MAKE)
  endif
endif

CMAKE_OPTIONS :=
# CMAKE_OPTIONS += -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON
CMAKE_OPTIONS_AFTER :=

ifeq ($(HOST_OS),Win)

ifeq ($(HOST_NAME),MinGW)
  CMAKE += -G "MinGW Makefiles"
else ifeq ($(HOST_ARCH),x64)
  ifeq ($(VS_CODE),)
    WHICH_CL := $(shell which cl)
    ifeq ($(WHICH_CL),)
      $(error "Visual Studio version is unknown. Could set VS_CODE to specify it, e.g. make [TARGET] VS_CODE=2017")
    endif
    # C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\...
    # C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\...
    VS_CODE := $(shell echo "$(WHICH_CL)" | grep -Po "(?<=Visual Studio[ /])[0-9]+")
  endif
  # $(call mkinfo,"VS_CODE: $(VS_CODE)")
  ifeq ($(filter $(VS_CODE),15 2017),$(VS_CODE))
    CMAKE += -G "Visual Studio 15 2017 Win64"
  else ifeq ($(filter $(VS_CODE),14 2015),$(VS_CODE))
    CMAKE += -G "Visual Studio 14 2015 Win64"
  else ifeq ($(filter $(VS_CODE),12 2013),$(VS_CODE))
    CMAKE += -G "Visual Studio 12 2013 Win64"
  else ifeq ($(filter $(VS_CODE),11 2012),$(VS_CODE))
    CMAKE += -G "Visual Studio 11 2012 Win64"
  else ifeq ($(filter $(VS_CODE),10 2010),$(VS_CODE))
    CMAKE += -G "Visual Studio 10 2010 Win64"
  else ifeq ($(filter $(VS_CODE),9 2008),$(VS_CODE))
    CMAKE += -G "Visual Studio 9 2008 Win64"
  else ifeq ($(filter $(VS_CODE),8 2005),$(VS_CODE))
    CMAKE += -G "Visual Studio 8 2005 Win64"
  else
    $(error "Visual Studio version is not proper, VS_CODE: $(VS_CODE)")
  endif
endif

endif

# Package

PKGVERSION := $(shell ./scripts/version.sh)
PKGNAME := mynteye-s-$(PKGVERSION)-$(HOST_OS)-$(HOST_ARCH)
ifeq ($(HOST_OS),Linux)
  PKGNAME := $(PKGNAME)-gcc$(shell gcc -dumpversion | cut -c 1-1)
endif
PKGNAME := $(call lower,$(PKGNAME))

# Shell

# `sh` is not possible to export a function
#   function __cp() {}; export -f __cp;

define echo
	text="$1"; options="$2"; \
	[ -z "$2" ] && options="1;33"; \
	$(ECHO) "\033[$${options}m$${text}\033[0m"
endef

define rm
	[ ! -h "$1" ] && [ ! -e "$1" ] || (rm -rf "$1" && $(ECHO) "RM: $1")
endef

define rm_f
	dir="$2"; [ -e "$${dir}" ] || dir="."; \
	$(FIND) "$${dir}" -mindepth 1 -maxdepth 1 -name "$1" | while read -r p; do \
		$(call rm,$$p); \
	done
endef

define mkdir
	([ -e "$1" ] || mkdir -p "$1")
endef

define cd
	$(call mkdir,$1) && cd "$1" && $(ECHO) "CD: $1"
endef

define cp
	(([ -d "$1" ] && $(call mkdir,$2) && cp -Rpv$3 "$1/." "$2") || \
		([ -f "$1" ] && $(call mkdir,$$(dirname "$2")) && cp -Rpv$3 "$1" "$2"))
endef

define cp_if
	if [ -e "$2" ]; then \
		$(ECHO) "CP: $1 > $2 already done"; \
	else \
		$(ECHO) "CP: $1 > $2" && $(call cp,$1,$2); \
	fi
endef

define cmake_build
	work_dir="$1"; \
	build_dir="$2"; [ -z "$2" ] && build_dir=..; \
	build_options="$3"; \
	$(call cd,$${work_dir}) && $(CMAKE) $${build_options} $(CMAKE_OPTIONS) $${build_dir} $(CMAKE_OPTIONS_AFTER) && $(BUILD)
endef

endif # _COMMON_DEFS_MAKE_
