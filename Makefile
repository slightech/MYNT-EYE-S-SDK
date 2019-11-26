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
include CommonDefs.mk

MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(patsubst %/,%,$(dir $(MKFILE_PATH)))

# CMAKE_INSTALL_PREFIX:
#   https://cmake.org/cmake/help/latest/variable/CMAKE_INSTALL_PREFIX.html
#
#   UNIX: /usr/local
#   Windows: c:/Program Files/${PROJECT_NAME}

# Options
#
#   SUDO: sudo command
#
# e.g. make [TARGET] SUDO=

SUDO ?= sudo
CMAKE_BUILD_EXTRA_OPTIONS ?=

.DEFAULT_GOAL := all

help:
	@echo "Usage:"
	@echo "  make help            show help message"
	@echo "  make apidoc          make api doc"
	@echo "  make opendoc         open api doc (html)"
	@echo "  make init            init project"
	@echo "  make build           build project"
	@echo "  make install         install project"
	@echo "  make samples         build samples"
	@echo "  make pkg             package sdk(windows)"
	@echo "  make ros             build ros wrapper"
	@echo "  make clean|cleanall  clean generated or useless things"

.PHONY: help

all: init samples ros

.PHONY: all

# doc

doc: apidoc

apidoc: cleandoc
	@$(call echo,Make $@)
	@cd docs; make html

opendoc: apidoc
	@$(call echo,Make $@)
	@$(SH) ./scripts/open.sh docs/_build/html/index.html

cleandoc:
	@$(call rm,./docs/_build/)
	@$(call rm,./docs/_doxygen/)

.PHONY: doc apidoc opendoc cleandoc

# init

init:
	@$(call echo,Make $@)
	@$(SH) ./scripts/init.sh $(INIT_OPTIONS)

.PHONY: init

# build

build:
	@$(call echo,Make $@)
ifeq ($(HOST_OS),Win)
	@$(call cmake_build,./_build,..,-DCMAKE_INSTALL_PREFIX=$(MKFILE_DIR)/_install $(CMAKE_BUILD_EXTRA_OPTIONS))
else
	@$(call cmake_build,./_build,..,$(CMAKE_BUILD_EXTRA_OPTIONS))
endif

.PHONY: build

# install

install: uninstall build
	@$(call echo,Make $@)
ifeq ($(HOST_OS),Win)
ifneq ($(HOST_NAME),MinGW)
	@cd ./_build; msbuild.exe INSTALL.vcxproj /property:Configuration=Release
else
	@cd ./_build; make install
endif
else
ifeq ($(HOST_OS),Linux)
	@cd ./_build; $(SUDO) make install
else
	@cd ./_build; make install
endif
endif

.PHONY: install

uninstall:
	@$(call echo,Make $@)
ifeq ($(HOST_OS),Linux)
	$(SUDO) rm -rf /usr/local/include/mynteye/
	$(SUDO) rm -rf /usr/local/lib/libmynteye.so*
	$(SUDO) rm -rf /usr/local/lib/cmake/mynteye/
	$(SUDO) rm -rf /usr/local/share/mynteye/
endif

.PHONY: uninstall

# samples

samples: install
	@$(call echo,Make $@)
	@$(call cmake_build,./samples/_build)

.PHONY: samples

# pkg

pkg: clean
	@$(call echo,Make $@)
ifeq ($(HOST_OS),Win)
	@$(SH) ./scripts/win/winpack.sh "$(PKGNAME)"
else
	$(error "Can't make pkg on $(HOST_OS)")
endif

cleanpkg:
	@$(call echo,Make $@)
	@$(call rm_f,$(PKGNAME)*)

.PHONY: pkg cleanpkg

# ros

ros: install
	@$(call echo,Make $@)
ifeq ($(HOST_OS),Linux)
	@cd ./wrappers/ros && catkin_make -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)
else
	$(error "Can't make ros on $(HOST_OS)")
endif

.PHONY: ros

cleanros:
	@$(call echo,Make $@)
	@$(call rm,./wrappers/ros/build/)
	@$(call rm,./wrappers/ros/devel/)
	@$(call rm,./wrappers/ros/install/)
	@$(call rm,./wrappers/ros/.catkin_workspace)
	@$(call rm,./wrappers/ros/src/CMakeLists.txt)
	@$(call rm_f,*INFO*,$(HOME)/.ros/)
	@$(call rm_f,*WARNING*,$(HOME)/.ros/)
	@$(call rm_f,*ERROR*,$(HOME)/.ros/)
	@$(call rm_f,*FATAL*,$(HOME)/.ros/)

.PHONY: cleanros


# clean

clean:
	@$(call echo,Make $@)
	@$(call rm,./_build/)
	@$(call rm,./_output/)
	@$(call rm,./_install/)
	@$(call rm,./samples/_build/)
	@$(call rm,./samples/_output/)
	@$(MAKE) cleanlog
ifeq ($(HOST_OS),Linux)
	@$(MAKE) cleanros
endif

cleanlog:
	@$(call rm_f,*INFO*)
	@$(call rm_f,*WARNING*)
	@$(call rm_f,*ERROR*)
	@$(call rm_f,*FATAL*)

cleanall: clean cleandoc
	@$(FIND) . -type f -name ".DS_Store" -print0 | xargs -0 rm -f

.PHONY: clean cleanlog cleanall

# others

host:
	@$(call echo,Make $@)
	@echo MKFILE_PATH: $(MKFILE_PATH)
	@echo MKFILE_DIR: $(MKFILE_DIR)
	@echo HOST_OS: $(HOST_OS)
	@echo HOST_ARCH: $(HOST_ARCH)
	@echo HOST_NAME: $(HOST_NAME)
	@echo SH: $(SH)
	@echo ECHO: $(ECHO)
	@echo FIND: $(FIND)
	@echo CC: $(CC)
	@echo CXX: $(CXX)
	@echo MAKE: $(MAKE)
	@echo BUILD: $(BUILD)
	@echo LDD: $(LDD)
	@echo CMAKE: $(CMAKE)
	@echo PKGNAME: $(PKGNAME)
	@echo CMAKE_BUILD_EXTRA_OPTIONS: $(CMAKE_BUILD_EXTRA_OPTIONS)

.PHONY: host
