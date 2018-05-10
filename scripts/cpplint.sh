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
ROOT_DIR=$(realpath "$BASE_DIR/..")

source "$BASE_DIR/common/echo.sh"
source "$BASE_DIR/common/detect.sh"
source "$BASE_DIR/common/host.sh"

PYTHON="python"
if [ "$HOST_OS" = "Win" ]; then
  # default python on MSYS
  PYTHON="python2"
fi

CPPLINT="$PYTHON $ROOT_DIR/tools/linter/cpplint.py"
# CPPLINT="cpplint"

_detect $PYTHON

DIRS=(
  _build/include
  include
  src
)

FIND="$($BASE_DIR/getfind.sh)"

PATT="-name *.cc -o -name *.h"
PATT="$PATT -o -name *.cpp -o -name *.hpp"
# PATT="$PATT -o -name *.cu -o -name *.cuh"
# _echo "PATT: $PATT"

ret=0
_echo_s "WorkDir: $ROOT_DIR"
for dir in "${DIRS[@]}"; do
  if [ -d "$ROOT_DIR/$dir" ]; then
    _echo_i "WorkDir: $dir"
    for f in `$FIND "$ROOT_DIR/$dir" -type f \( ${PATT} \)`; do
      _echo_in "cpplint $f"
      $CPPLINT "--verbose=1" \
        "--filter=-legal/copyright,-build/c++11" \
        "--root=$ROOT_DIR/$dir" "$f"
      if [ $? -eq 0 ]; then
        _echo_dn "cpplint success"
      else
        _echo_en "cpplint failed"
        ret=1
      fi
    done
  else
    _echo_i "WorkDir: $dir - not found"
  fi
done

if [ $ret -eq 0 ]; then
  _echo_d "Well done"
else
  _echo_e "There are cpplint errors"
fi
