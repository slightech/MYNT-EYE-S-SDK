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

option(WITH_API "Build with API layer, need OpenCV" ON)

message(STATUS "Options:")
message(STATUS "  WITH_API: ${WITH_API}")

# How to install glog?
#   Ubuntu: `sudo apt-get install libgoogle-glog-dev`
option(WITH_GLOG "Include glog support" OFF)
