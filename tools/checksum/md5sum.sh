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
SCRIPTS_DIR="$ROOT_DIR/scripts"

source "$SCRIPTS_DIR/common/echo.sh"
source "$SCRIPTS_DIR/common/host.sh"

if [ "$HOST_OS" = "Linux" ]; then
  _md5sum() { md5sum "$1"; }
elif [ "$HOST_OS" = "Mac" ]; then
  _md5sum() { md5 -q "$1"; }
elif [ "$HOST_OS" = "Win" ]; then
  _md5sum() { certutil -hashfile "$1" MD5; }
else  # unexpected
  _echo_e "Unknown host os :("
  exit 1
fi

PYTHON="python"
if [ "$HOST_OS" = "Win" ]; then
  # default python on MSYS
  PYTHON="python2"
fi

_get_size() {
PYTHON_ARG="$1" $PYTHON - <<EOF
import math
from os.path import getsize

def convert_size(size_bytes):
  if size_bytes == 0:
    return "0B"
  size_name = ("B", "KB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB")
  i = int(math.floor(math.log(size_bytes, 1024)))
  p = math.pow(1024, i)
  s = round(size_bytes / p, 2)
  return "%s %s" % (s, size_name[i])

import os
print(convert_size(getsize(os.environ["PYTHON_ARG"])))
EOF
}

_print_info() {
  file="$1"
  _echo "File: $file"
  _echo "Size: `_get_size "$file"`"
  _echo "MD5: `_md5sum "$file"`"
  _echo
}

if [ $# -eq 0 ]; then
  _echo_e "Usage: ./tools/checksum/md5sum.sh <file or directory>"
  exit 1
fi

if [ -d "$1" ]; then
  find "$1" -type f | while read -r f; do
    _print_info "$f"
  done
else
  _print_info "$1"
fi
