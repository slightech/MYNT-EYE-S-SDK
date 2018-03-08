#!/usr/bin/env sh

if type where &> /dev/null; then
  for i in `where find`; do
    # find on MSYS instead of Windows
    if [ `echo $i | grep -c "msys"` -gt 0 ]; then
      echo "${i%.exe}" | tr -d "[:space:]" | sed -e 's/\\/\//g'
      exit
    fi
  done
fi

echo "find"
