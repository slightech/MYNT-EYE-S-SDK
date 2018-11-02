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

[ -n "${_ECHO_SH_}" ] && return || readonly _ECHO_SH_=1
[ -n "${_VERBOSE_}" ] && echo "-- INCLUDE: echo.sh"

# if [ -n "$SCRIPTS_DIR" ]; then
#   source "$SCRIPTS_DIR/common/echo.sh"
# else
#   source "$(dirname "$0")/echo.sh"
# fi

ECHO="echo -e"

# task colors
COLOR_STRONG="1;35"  # Magenta
COLOR_INFO="1;34"    # Blue
COLOR_WARN="1;33"    # Yellow
COLOR_DONE="1;32"    # Green
COLOR_ERROR="1;31"   # Red
# action colors
COLOR_STRONG_NORMAL="35"
COLOR_INFO_NORMAL="34"
COLOR_WARN_NORMAL="33"
COLOR_DONE_NORMAL="32"
COLOR_ERROR_NORMAL="31"

_echo_() {
  text="$1"; shift; options="$1"; shift;
  [ -z "$options" ] && options="$COLOR_STRONG";
  $ECHO "\033[${options}m${text}\033[0m"
}

_echo_e_() {
  text="$1"; shift; options="$1"; shift;
  [ -z "$options" ] && options="$COLOR_ERROR";
  $ECHO >&2 "\033[${options}m${text}\033[0m"
}

_echo() {
  $ECHO "$@"
}

_echo_s() {
  _echo_ "$1" "$COLOR_STRONG"
}

_echo_i() {
  _echo_ "$1" "$COLOR_INFO"
}

_echo_w() {
  _echo_ "$1" "$COLOR_WARN"
}

_echo_d() {
  _echo_ "$1" "$COLOR_DONE"
}

_echo_e() {
  _echo_e_ "$1" "$COLOR_ERROR"
}

_echo_sn() {
  _echo_ "$1" "$COLOR_STRONG_NORMAL"
}

_echo_in() {
  _echo_ "$1" "$COLOR_INFO_NORMAL"
}

_echo_wn() {
  _echo_ "$1" "$COLOR_WARN_NORMAL"
}

_echo_dn() {
  _echo_ "$1" "$COLOR_DONE_NORMAL"
}

_echo_en() {
  _echo_e_ "$1" "$COLOR_ERROR_NORMAL"
}
