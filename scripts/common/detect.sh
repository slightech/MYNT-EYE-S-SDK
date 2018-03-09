#!/usr/bin/env sh

[ -n "${_DETECT_SH_}" ] && return || readonly _DETECT_SH_=1
[ -n "${_VERBOSE_}" ] && echo "-- INCLUDE: detect.sh"

_detect() {
  cmd="$1"; optional="$2"; verbose="$3";
  [ -n "$verbose" ] || [ -z "${_VERBOSE_}" ] || verbose=1;
  if ! type "$cmd" &> /dev/null; then
    [ -z "$verbose" ] || echo "-- DETECT: $cmd not found"
    if [ -z "$optional" ]; then
      echo >&2 "-- DETECT: $cmd not found, but required"
      exit 1
    fi
  else
    [ -z "$verbose" ] || echo "-- DETECT: $cmd found"
    eval "${cmd}_FOUND=1"
  fi
}

_detect_cmd() {
  [ -x "$(command -v $1)" ]
}

_detect_fn() {
  [ `type -t $1`"" == "function" ]
}

if [ -n "${_TEST_}" ]; then
  echo "-- TEST_BEG: _detect"
  _CMDS=(ls none)
  for _cmd in "${_CMDS[@]}"; do
    _detect "$_cmd" 1

    _CMD_FOUND="${_cmd}_FOUND"
    if [ -n "${!_CMD_FOUND}" ]; then
      echo "--   ${_CMD_FOUND} set"
    else
      echo "--   ${_CMD_FOUND} unset"
    fi
  done
  echo "-- TEST_END: _detect"

  echo "-- TEST_BEG: _detect_fn"
  _FUNCS=(_detect none)
  for _fn in "${_FUNCS[@]}"; do
    if _detect_fn "$_fn"; then
      echo "--   $_fn function exists"
    else
      echo "--   $_fn function not exist"
    fi
  done
  echo "-- TEST_END: _detect_fn"
fi
