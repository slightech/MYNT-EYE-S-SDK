#!/usr/bin/env bash

[ -n "${_STRING_SH_}" ] && return || readonly _STRING_SH_=1
[ -n "${_VERBOSE_}" ] && echo "-- INCLUDE: string.sh"

_startswith() { [ "$1" != "${1#$2}" ]; }
_endswith() { [ "$1" != "${1%$2}" ]; }
