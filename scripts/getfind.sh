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
