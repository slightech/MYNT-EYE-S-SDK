#!/usr/bin/env sh

BASE_DIR=$(cd "$(dirname "$0")" && pwd)

source "$BASE_DIR/common/echo.sh"
source "$BASE_DIR/common/host.sh"

if [ "$HOST_OS" = "Linux" ]; then
  OPEN="xdg-open"
elif [ "$HOST_OS" = "Mac" ]; then
  OPEN="open"
elif [ "$HOST_OS" = "Win" ]; then
  OPEN="start"
else
  _echo_e "Unknown host os :("
  exit 1
fi

[ $# -gt 0 ] && $OPEN "$@"
