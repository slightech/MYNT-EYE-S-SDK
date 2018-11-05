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

BASE_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(realpath "$BASE_DIR/../..")
SCRIPTS_DIR=$(realpath "$BASE_DIR/..")

source "$SCRIPTS_DIR/common/echo.sh"
source "$SCRIPTS_DIR/common/detect.sh"

if [ ! -d "$ROOT_DIR/3rdparty/opencv" ]; then
  _echo_e "3rdparty/opencv not found, please manually download it to here."
  _echo_e
  _echo_e "  OpenCV Win pack 3.4.3: https://opencv.org/releases.html"
  exit 1
fi

if ! _detect_cmd makensis; then
  _echo_e "makensis not found, please manually download and install it."
  _echo_e
  _echo_e "  NSIS: http://nsis.sourceforge.net"
  exit 1
fi

export OpenCV_DIR="$ROOT_DIR/3rdparty/opencv/build"

_rm() {
  [ -e "$1" ] && (rm -r "$1" && _echo_i "RM: $1")
}

_md() {
  [ ! -d "$1" ] && (mkdir -p "$1" && _echo_i "MD: $1")
}

# _mv_subs <srcdir> <dstdir> [sub1 sub2 ...]
_mv_subs() {
  _src_dir="$1"; shift; _dst_dir="$1"; shift; _subs="$@";
  if [ -z "$_subs" ]; then
    _subs=`ls $_src_dir`
  fi
  for _sub in $_subs; do
    _src="$_src_dir/$_sub"
    [ ! -e "$_src" ] && (_echo_i "Not exist: $_src") && continue
    _dst=
    [ -f "$_src" ] && _dst="$_dst_dir"
    [ -d "$_src" ] && _dst="$_dst_dir/$_sub"
    [ -z "$_dst" ] && continue
    # _echo "_src: $_src"
    # _echo "_dst: $_dst"
    mv "$_src" "$_dst"
  done
}

################################################################################
# build release

make samples tools

################################################################################
# build debug

rm -r "$ROOT_DIR/_build"
rm -r "$ROOT_DIR/_output"
make build BUILD_TYPE=Debug

mv "$ROOT_DIR/_output/bin/mynteyed.dll" "$ROOT_DIR/_install/bin/mynteyed.dll"
mv "$ROOT_DIR/_output/lib/mynteyed.lib" "$ROOT_DIR/_install/lib/mynteyed.lib"

################################################################################
# move to _install

# 3rdparty/opencv
_md "$ROOT_DIR/_install/3rdparty"
mv "$ROOT_DIR/3rdparty/opencv" "$ROOT_DIR/_install/3rdparty/opencv"

# cmake
mv "$ROOT_DIR/cmake" "$ROOT_DIR/_install/cmake"

# samples
mv "$ROOT_DIR/samples/_output/bin" "$ROOT_DIR/_install/bin/samples"
mv "$ROOT_DIR/samples/_output/lib" "$ROOT_DIR/_install/lib/samples"
_rm "$ROOT_DIR/samples/_build"
_rm "$ROOT_DIR/samples/_output"
mv "$ROOT_DIR/samples" "$ROOT_DIR/_install/samples"

# tools
mv "$ROOT_DIR/tools/_output/bin" "$ROOT_DIR/_install/bin/tools"
mv "$ROOT_DIR/tools/_output/lib" "$ROOT_DIR/_install/lib/tools"
_rm "$ROOT_DIR/tools/_build"
_rm "$ROOT_DIR/tools/_output"
mv "$ROOT_DIR/tools/linter" "$ROOT_DIR/3rdparty/linter"
mv "$ROOT_DIR/tools" "$ROOT_DIR/_install/tools"

# platforms/win
mv "$ROOT_DIR/platforms/win/README.txt" "$ROOT_DIR/_install"

_rm "$ROOT_DIR/platforms/projects/vs2017/mynteyes_demo/.vs"
_rm "$ROOT_DIR/platforms/projects/vs2017/mynteyes_demo/x64"
_rm "$ROOT_DIR/platforms/projects/vs2017/mynteyes_demo/mynteyes_demo/x64"
_rm "$ROOT_DIR/platforms/projects/vs2017/mynteyes_demo/mynteyes_demo/mynteyes_demo.vcxproj.user"
mv "$ROOT_DIR/platforms/projects" "$ROOT_DIR/_install/projects"

################################################################################
# copy to _install

cp -f "$ROOT_DIR/scripts/win/cmake/mynteye-targets.cmake" "$ROOT_DIR/_install/lib/cmake/mynteye/"
cp -f "$ROOT_DIR/scripts/win/cmake/mynteye-targets-release.cmake" "$ROOT_DIR/_install/lib/cmake/mynteye/"

cp -f "$ROOT_DIR/scripts/win/generate.bat" "$ROOT_DIR/_install/samples/"
cp -f "$ROOT_DIR/scripts/win/generate.bat" "$ROOT_DIR/_install/tools/"

################################################################################
# archive exe

source "$ROOT_DIR/pkginfo.sh"
_pkgname="$1-opencv-$OpenCV_VERSION"
_rm "$ROOT_DIR/$_pkgname.exe"
mv "$ROOT_DIR/_install" "$ROOT_DIR/$_pkgname"

makensis "$ROOT_DIR/winpack.nsi"

mv "$ROOT_DIR/$_pkgname" "$ROOT_DIR/_install"

################################################################################
# remove from _install

_rm "$ROOT_DIR/_install/samples/generate.bat"
_rm "$ROOT_DIR/_install/tools/generate.bat"

################################################################################
# move back from _install

# 3rdparty/opencv
mv "$ROOT_DIR/_install/3rdparty/opencv" "$ROOT_DIR/3rdparty/opencv"

# cmake
mv "$ROOT_DIR/_install/cmake" "$ROOT_DIR/cmake"

# samples
mv "$ROOT_DIR/_install/samples" "$ROOT_DIR/samples"

# tools
mv "$ROOT_DIR/_install/tools" "$ROOT_DIR/tools"
mv "$ROOT_DIR/3rdparty/linter" "$ROOT_DIR/tools/linter"

# platforms/win
mv "$ROOT_DIR/_install/README.txt" "$ROOT_DIR/platforms/win"

mv "$ROOT_DIR/_install/projects" "$ROOT_DIR/platforms/projects"

################################################################################
# clean build

_rm "$ROOT_DIR/_build"
_rm "$ROOT_DIR/_output"


_echo_d "Win pack success"
