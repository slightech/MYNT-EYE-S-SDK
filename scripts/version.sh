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

BASE_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(dirname "$BASE_DIR")
CONFIG_FILE="$ROOT_DIR/CMakeLists.txt"

version=$(cat "$CONFIG_FILE" | grep -m1 "mynteye VERSION")
version=$(echo "${version%LANGUAGES*}")
version=$(echo "${version#*VERSION}" | tr -d '[:space:]')

echo "$version"
