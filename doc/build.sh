#!/usr/bin/env bash
# _VERBOSE_=1
# _TEST_=1

BASE_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(realpath "$BASE_DIR/..")
SCRIPTS_DIR="$ROOT_DIR/scripts"

source "$SCRIPTS_DIR/common/echo.sh"
source "$SCRIPTS_DIR/common/mkdir.sh"
source "$SCRIPTS_DIR/common/detect.sh"

_detect "doxygen"
_detect "pdflatex" 1

source "$BASE_DIR/langs.sh"
DOXYFILE="api.doxyfile"
OUTPUT="$BASE_DIR/output"

for lang in "${LANGS[@]}"; do
  _echo_s "Build doc $lang"
  [ -d "$BASE_DIR/$lang" ] || continue
  cd "$BASE_DIR/$lang"
  if [ -f "$DOXYFILE" ]; then
    _mkdir "$OUTPUT/$lang"
    _echo_i "doxygen $DOXYFILE"
    doxygen $DOXYFILE
    if [ $pdflatex_FOUND ] && [ -f "$OUTPUT/$lang/latex/Makefile" ]; then
      _echo_in "doxygen make latex"
      cd "$OUTPUT/$lang/latex" && make
      [ -f "refman.pdf" ] && mv "refman.pdf" "../mynteye-apidoc.pdf"
    fi
    _echo_d "doxygen completed"
  else
    _echo_e "$DOXYFILE not found"
  fi
done
