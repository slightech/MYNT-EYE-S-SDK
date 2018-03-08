#!/usr/bin/env sh

[ -n "${_MKDIR_SH_}" ] && return || readonly _MKDIR_SH_=1
[ -n "${_VERBOSE_}" ] && echo "-- INCLUDE: mkdir.sh"

MKDIR="mkdir -p"

_mkdir() {
  dir="$1"; required="$2"; verbose="$3";
  [ -n "$verbose" ] || [ -z "${_VERBOSE_}" ] || verbose=1;
  if [ -e "$dir" ]; then
    if [ -d "$dir" ]; then
      [ -z "$verbose" ] || echo "-- MKDIR: $dir exists"
    else
      [ -z "$verbose" ] || echo >&2 "-- MKDIR: $dir not directory"
      if [ -n "$required" ]; then
        echo >&2 "-- MKDIR: $dir not directory, but required"
        exit 1
      fi
    fi
  else
    [ -z "$verbose" ] || echo "-- MKDIR: $dir creates"
    $MKDIR "$dir"
  fi
}
