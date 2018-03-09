#!/usr/bin/env bash
# _VERBOSE_=1
# _FORCE_INSRALL_=1

BASE_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(realpath "$BASE_DIR/..")

source "$BASE_DIR/common/echo.sh"
source "$BASE_DIR/common/detect.sh"
source "$BASE_DIR/common/host.sh"

PYTHON="python"
if [ "$HOST_OS" = "Win" ]; then
  # default python on MSYS
  PYTHON="python2"
fi

_detect curl
_detect $PYTHON

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
    _detect_cmd $_dep || _deps+=($_dep)
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
  _install_deps "sudo apt-get install" build-essential cmake git clang-format
  # sudo
  SUDO="sudo"
elif [ "$HOST_OS" = "Mac" ]; then
  # detect brew
  if ! _detect_cmd brew; then
    _echo_sn "Install brew"
    _detect ruby
    ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
  fi
  # brew install
  _install_deps "brew install" cmake git clang-format
  # link clang-format-diff (if not compatible with Python 3, fix it by yourself)
  [ -f "/usr/local/bin/clang-format-diff" ] || \
    ln -s /usr/local/share/clang/clang-format-diff.py /usr/local/bin/clang-format-diff
elif [ "$HOST_OS" = "Win" ]; then
  # detect pacman on MSYS
  _detect pacman
  # pacman install (common)
  _install_deps "pacman -S" git clang-format
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
  _echo_sn "Install pip"
  [ -f "get-pip.py" ] || curl -O https://bootstrap.pypa.io/get-pip.py
  $SUDO $PYTHON get-pip.py
fi
# pip install
_echo_d "pip install --upgrade autopep8 cpplint pylint requests"
$SUDO pip install --upgrade autopep8 cpplint pylint requests

## init

_echo_s "Init git hooks"
$PYTHON "$ROOT_DIR/tools/linter/init-git-hooks.py"

exit 0
