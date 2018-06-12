#!/usr/bin/env bash
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

# _VERBOSE_=1
# _FORCE_INSRALL_=1

BASE_DIR=$(cd "$(dirname "$0")" && pwd)

source "$BASE_DIR/common/echo.sh"
source "$BASE_DIR/common/detect.sh"
source "$BASE_DIR/common/host.sh"

PYTHON="python"
if [ "$HOST_OS" = "Win" ]; then
  # default python on MSYS
  PYTHON="python2"
fi

_detect $PYTHON 1

PYTHON_FOUND="${PYTHON}_FOUND"
if [ -z "${!PYTHON_FOUND}" ]; then
  _echo_en "$PYTHON not found"
fi

if [ "$HOST_OS" = "Linux" ]; then
  _detect_install() {
    _detect_cmd "$1" || [ $(dpkg-query -W -f='${Status}' "$1" 2> /dev/null \
        | grep -c "ok installed") -gt 0 ]
  }
elif [ "$HOST_OS" = "Mac" ]; then
  _detect_install() {
    _detect_cmd "$1" || brew ls --versions "$1" > /dev/null
  }
else
  _detect_install() {
    _detect_cmd "$1"
  }
fi

_install_deps() {
  _cmd="$1"; shift; _deps_all=($@)
  _echo "Install cmd: $_cmd"
  _echo "Install deps: ${_deps_all[*]}"
  if [ -n "${_FORCE_INSRALL_}" ]; then
    _echo_d "$_cmd ${_deps_all[*]}"
    $_cmd ${_deps_all[@]}
    return
  fi
  _deps=()
  for _dep in "${_deps_all[@]}"; do
    _detect_install $_dep || _deps+=($_dep)
  done
  if [ ${#_deps[@]} -eq 0 ]; then
    _echo_i "All deps already exist"
  else
    _echo_d "$_cmd ${_deps[*]} (not exists)"
    $_cmd ${_deps[@]}
  fi
}

## deps

_echo_s "Init deps"

if [ "$HOST_OS" = "Linux" ]; then
  # detect apt-get
  _detect apt-get
  # apt-get install
  _install_deps "sudo apt-get install" build-essential curl cmake git clang-format
  _install_deps "sudo apt-get install" libv4l-dev
  if ! _detect_cmd clang-format; then
    # on Ubuntu 14.04, apt-cache search clang-format
    _install_deps "sudo apt-get install" clang-format-3.9
    sudo ln -sf clang-format-3.9 /usr/bin/clang-format
    sudo ln -sf clang-format-diff-3.9 /usr/bin/clang-format-diff
  fi
  # sudo
  SUDO="sudo"
elif [ "$HOST_OS" = "Mac" ]; then
  # detect brew
  if ! _detect_cmd brew; then
    _echo_sn "Install brew"
    _detect curl
    _detect ruby
    ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
  fi
  # brew install
  _install_deps "brew install" curl cmake git clang-format
  # link clang-format-diff (if not compatible with Python 3, fix it by yourself)
  [ -f "/usr/local/bin/clang-format-diff" ] || \
    ln -s /usr/local/share/clang/clang-format-diff.py /usr/local/bin/clang-format-diff
  _install_deps "brew install" libuvc
elif [ "$HOST_OS" = "Win" ]; then
  # detect pacman on MSYS
  _detect pacman
  # pacman install (common)
  _install_deps "pacman -S" curl git clang-format
  if [ "$HOST_NAME" = "MINGW" ]; then
    # pacman install (MINGW)
    _deps=()
    if [ "$HOST_ARCH" = "x64" ]; then
      _deps+=(mingw-w64-x86_64-toolchain mingw-w64-x86_64-cmake)
    elif [ "$HOST_ARCH" = "x86" ]; then
      _deps+=(mingw-w64-i686-toolchain mingw-w64-i686-cmake)
    else
      _echo_e "Unknown host arch :("
      exit 1
    fi
    if ! [ ${#_deps[@]} -eq 0 ]; then
      _echo_d "pacman -S ${_deps[*]}"
      pacman -S ${_deps[@]}
    fi
  else
    # detect cmake on MSYS
    _detect cmake
  fi
  # update
  #   pacman -Syu
  # search
  #   pacman -Ss make
  # autoremove
  #   pacman -Qtdq | pacman -Rs -
else  # unexpected
  _echo_e "Unknown host os :("
  exit 1
fi

## pip

# detect pip
if ! _detect_cmd pip; then
  if [ -n "${!PYTHON_FOUND}" ]; then
    _echo_sn "Install pip"
    [ -f "get-pip.py" ] || curl -O https://bootstrap.pypa.io/get-pip.py
    $SUDO $PYTHON get-pip.py
  else
    _echo_en "Skipped install pip, as $PYTHON not found"
  fi
fi
# pip install
if _detect_cmd pip; then
  _echo_d "pip install --upgrade autopep8 cpplint pylint requests"
  $SUDO pip install --upgrade autopep8 cpplint pylint requests
else
  _echo_en "Skipped pip install packages, as pip not found"
fi

## realpath

# detect realpath
if ! _detect_cmd realpath; then
  _echo_sn "Install realpath"
  if [ "$HOST_OS" = "Linux" ]; then
    # How to install realpath on Ubuntu 14.04
    #   https://www.howtoinstall.co/en/ubuntu/trusty/realpath
    sudo apt-get install coreutils realpath
  elif [ "$HOST_OS" = "Mac" ]; then
    brew install coreutils
  elif [ "$HOST_OS" = "Win" ]; then
    pacman -S coreutils
  else  # unexpected
    _echo_e "Unknown host os :("
    exit 1
  fi
fi

ROOT_DIR=$(realpath "$BASE_DIR/..")

## init

if [ -n "${!PYTHON_FOUND}" ]; then
  _echo_s "Init git hooks"
  $PYTHON "$ROOT_DIR/tools/linter/init-git-hooks.py"
else
  _echo_en "Skipped init git hooks, as $PYTHON not found"
fi

## cmake version

_echo_s "Expect cmake version >= 3.0"
cmake --version | head -1
if [ "$HOST_NAME" = "Ubuntu" ]; then
  # sudo apt remove cmake
  _echo "How to upgrade cmake in Ubuntu"
  _echo "  https://askubuntu.com/questions/829310/how-to-upgrade-cmake-in-ubuntu"
fi

exit 0
