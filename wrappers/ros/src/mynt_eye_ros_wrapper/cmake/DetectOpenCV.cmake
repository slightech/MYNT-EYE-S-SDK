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

include(${CMAKE_CURRENT_LIST_DIR}/IncludeGuard.cmake)
cmake_include_guard()

find_package(OpenCV REQUIRED)

message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

set(WITH_OPENCV TRUE)
add_definitions(-DWITH_OPENCV)

if(OpenCV_VERSION VERSION_LESS 3.0)
  add_definitions(-DWITH_OPENCV2)
elseif(OpenCV_VERSION VERSION_LESS 4.0)
  add_definitions(-DWITH_OPENCV3)
else()
  add_definitions(-DWITH_OPENCV4)
endif()

list(FIND OpenCV_LIBS "opencv_world" __index)
if(${__index} GREATER -1)
  set(WITH_OPENCV_WORLD TRUE)
endif()

if(MSVC OR MSYS OR MINGW)
  get_filename_component(OpenCV_LIB_SEARCH_PATH "${OpenCV_LIB_PATH}/../bin" ABSOLUTE)
else()
  set(OpenCV_LIB_SEARCH_PATH "${OpenCV_LIB_PATH}")
endif()
