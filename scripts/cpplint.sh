#!/usr/bin/env bash

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
