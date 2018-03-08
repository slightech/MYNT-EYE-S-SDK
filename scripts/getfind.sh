#!/usr/bin/env sh

FIND="find"

if type where &> /dev/null; then
  for i in `where find`; do
    # find on MSYS instead of Windows
    if [ `echo $i | grep -c "msys"` -gt 0 ]; then
      FIND=`echo "${i%.exe}" | tr -d "[:space:]" | sed 's:\\\:/:g'`
    fi
  done
fi

echo "$FIND"
