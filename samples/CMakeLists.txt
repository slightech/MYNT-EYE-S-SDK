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

cmake_minimum_required(VERSION 3.0)

project(mynteye_samples VERSION 2.0.0 LANGUAGES C CXX)

get_filename_component(PRO_DIR ${PROJECT_SOURCE_DIR} DIRECTORY)

include(${PRO_DIR}/cmake/Common.cmake)
include(${PRO_DIR}/cmake/Utils.cmake)

# options

include(${PRO_DIR}/cmake/Option.cmake)

# flags

if(OS_WIN)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
else()
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

include(${PRO_DIR}/cmake/DetectCXX11.cmake)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

string(STRIP "${CMAKE_C_FLAGS}" CMAKE_C_FLAGS)
string(STRIP "${CMAKE_CXX_FLAGS}" CMAKE_CXX_FLAGS)
message(STATUS "C_FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

# packages

LIST(APPEND CMAKE_PREFIX_PATH ${PRO_DIR}/_install/lib/cmake)
find_package(mynteye REQUIRED)
message(STATUS "Found mynteye: ${mynteye_VERSION}")

include(${PRO_DIR}/cmake/DetectOpenCV.cmake)

#LIST(APPEND CMAKE_MODULE_PATH ${PRO_DIR}/cmake)

# targets

set(OUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/_output")

# samples above api layer

if(WITH_API)
  add_subdirectory(api)
endif()

# samples above device layer

add_subdirectory(device)

# samples above uvc layer

add_subdirectory(uvc)

# tutorials

if(WITH_API)
  add_subdirectory(tutorials)
endif()
