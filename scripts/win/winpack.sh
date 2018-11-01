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
  [ -e "$1" ] && (rm -r "$1" && _echo_in "RM: $1")
}

_md() {
  [ ! -d "$1" ] && (mkdir -p "$1" && _echo_in "MD: $1")
}

################################################################################
# build release

make install

################################################################################
# build debug

rm -r "$ROOT_DIR/_build"
rm -r "$ROOT_DIR/_output"
make build BUILD_TYPE=Debug

mv "$ROOT_DIR/_output/bin/mynteyed.dll" "$ROOT_DIR/_install/bin/mynteyed.dll"
mv "$ROOT_DIR/_output/lib/mynteyed.lib" "$ROOT_DIR/_install/lib/mynteyed.lib"

################################################################################
# copy to _install

cp -f "$ROOT_DIR/scripts/win/cmake/mynteye-targets.cmake" "$ROOT_DIR/_install/lib/cmake/mynteye/"
cp -f "$ROOT_DIR/scripts/win/cmake/mynteye-targets-release.cmake" "$ROOT_DIR/_install/lib/cmake/mynteye/"

################################################################################
# move to _install

# 3rdparty/opencv
_md "$ROOT_DIR/_install/3rdparty"
mv "$ROOT_DIR/3rdparty/opencv" "$ROOT_DIR/_install/3rdparty/opencv"

################################################################################
# archive exe

source "$ROOT_DIR/pkginfo.sh"
_pkgname=$1-opencv-$OpenCV_VERSION
mv "$ROOT_DIR/_install" "$ROOT_DIR/$_pkgname"

makensis "$ROOT_DIR/winpack.nsi"

mv "$ROOT_DIR/$_pkgname" "$ROOT_DIR/_install"

################################################################################
# move back from _install

# 3rdparty/opencv
mv "$ROOT_DIR/_install/3rdparty/opencv" "$ROOT_DIR/3rdparty/opencv"

################################################################################
# clean build

_rm "$ROOT_DIR/_build"
_rm "$ROOT_DIR/_output"


_echo_d "Win pack success"
