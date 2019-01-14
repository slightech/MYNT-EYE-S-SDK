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

if(OpenCV_FIND_QUIET)
  find_package(OpenCV QUIET)
else()
  find_package(OpenCV REQUIRED)
endif()

if(OpenCV_FOUND)

#message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

set(WITH_OPENCV TRUE)

if(OpenCV_VERSION VERSION_LESS 3.0)
  set(WITH_OPENCV2 TRUE)
elseif(OpenCV_VERSION VERSION_LESS 4.0)
  set(WITH_OPENCV3 TRUE)
else()
  set(WITH_OPENCV4 TRUE)
endif()

list(FIND OpenCV_LIBS "opencv_world" __index)
if(${__index} GREATER -1)
  set(WITH_OPENCV_WORLD TRUE)
endif()

if(NOT OpenCV_LIB_PATH)
  list(LENGTH OpenCV_INCLUDE_DIRS __length)
  if(${__length} GREATER 0)
    list(GET OpenCV_INCLUDE_DIRS 0 __include_dir)
    string(REGEX REPLACE "include.*$" "lib" __lib_dir "${__include_dir}")
    find_library(__opencv_lib
      NAMES opencv_core3 opencv_core opencv_world
      PATHS "${__lib_dir}" "${__lib_dir}/x86_64-linux-gnu"
      NO_DEFAULT_PATH)
    #message(STATUS "__opencv_lib: ${__opencv_lib}")
    if(__opencv_lib)
      get_filename_component(OpenCV_LIB_PATH "${__opencv_lib}" DIRECTORY)
    else()
      set(OpenCV_LIB_PATH "${__lib_dir}")
    endif()
    #message(STATUS "OpenCV_LIB_PATH: ${OpenCV_LIB_PATH}")
  endif()
endif()

if(MSVC OR MSYS OR MINGW)
  get_filename_component(OpenCV_LIB_SEARCH_PATH "${OpenCV_LIB_PATH}/../bin" ABSOLUTE)
else()
  set(OpenCV_LIB_SEARCH_PATH "${OpenCV_LIB_PATH}")
endif()

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

else()

set(WITH_OPENCV FALSE)

endif()
