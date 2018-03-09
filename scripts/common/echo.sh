#!/usr/bin/env sh

[ -n "${_ECHO_SH_}" ] && return || readonly _ECHO_SH_=1
[ -n "${_VERBOSE_}" ] && echo "-- INCLUDE: echo.sh"

# if [ -n "$SCRIPTS_DIR" ]; then
#   source "$SCRIPTS_DIR/common/echo.sh"
# else
#   source "$(dirname "$0")/echo.sh"
# fi

if [ "$OS" = "Windows_NT" ]; then
  ECHO="echo -e"
else
  ECHO="echo"
fi

# task colors
COLOR_STRONG="1;35"  # Magenta
COLOR_INFO="1;34"    # Blue
COLOR_DONE="1;32"    # Green
COLOR_ERROR="1;31"   # Red
# action colors
COLOR_STRONG_NORMAL="35"
COLOR_INFO_NORMAL="34"
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

_echo_dn() {
  _echo_ "$1" "$COLOR_DONE_NORMAL"
}

_echo_en() {
  _echo_e_ "$1" "$COLOR_ERROR_NORMAL"
}
