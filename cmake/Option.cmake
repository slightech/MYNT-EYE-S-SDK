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

include(${CMAKE_CURRENT_LIST_DIR}/Utils.cmake)


# build components

option(WITH_API "Build with API layer, need OpenCV" ON)

# 3rdparty components

option(WITH_BOOST "Include Boost support" ON)


# packages

if(WITH_API)
  include(${CMAKE_CURRENT_LIST_DIR}/DetectOpenCV.cmake)
endif()

if(WITH_BOOST)
  find_package(Boost COMPONENTS filesystem)
  if(Boost_FOUND)
    set(Boost_VERSION_STRING "${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_SUBMINOR_VERSION}")
    set(WITH_FILESYSTEM TRUE)
    set(WITH_BOOST_FILESYSTEM TRUE)
    add_definitions(-DWITH_FILESYSTEM)
    add_definitions(-DWITH_BOOST_FILESYSTEM)
  endif()
endif()

if(NOT WITH_FILESYSTEM)
  if(MSVC OR MSYS OR MINGW)  # win
    set(WITH_FILESYSTEM TRUE)
    set(WITH_NATIVE_FILESYSTEM TRUE)
    add_definitions(-DWITH_FILESYSTEM)
    add_definitions(-DWITH_NATIVE_FILESYSTEM)
  endif()
endif()

find_package(CUDA QUIET)

# set default int values for yaml file (build.info)
set_version_values(CMAKE_CXX_COMPILER_VERSION)
set_default_value(CMAKE_CXX_COMPILER_VERSION_MAJOR 0)
set_default_value(CMAKE_CXX_COMPILER_VERSION_MINOR 0)
set_default_value(CMAKE_CXX_COMPILER_VERSION_PATCH 0)
set_default_value(CMAKE_CXX_COMPILER_VERSION_TWEAK 0)
set_default_value(CUDA_VERSION_MAJOR 0)
set_default_value(CUDA_VERSION_MINOR 0)
set_default_value(OpenCV_VERSION_MAJOR 0)
set_default_value(OpenCV_VERSION_MINOR 0)
set_default_value(OpenCV_VERSION_PATCH 0)
set_default_value(OpenCV_VERSION_TWEAK 0)
set_default_value(${PROJECT_NAME}_VERSION_MAJOR 0)
set_default_value(${PROJECT_NAME}_VERSION_MINOR 0)
set_default_value(${PROJECT_NAME}_VERSION_PATCH 0)
set_default_value(${PROJECT_NAME}_VERSION_TWEAK 0)

# summary

status("")
status("Platform:")
status("  HOST_OS: ${HOST_OS}")
status("  HOST_NAME: ${HOST_NAME}")
status("  HOST_ARCH: ${HOST_ARCH}")
status("  HOST_COMPILER: ${CMAKE_CXX_COMPILER_ID}")
status("    COMPILER_VERSION: ${CMAKE_CXX_COMPILER_VERSION}")
status("    COMPILER_VERSION_MAJOR: ${CMAKE_CXX_COMPILER_VERSION_MAJOR}")
status("    COMPILER_VERSION_MINOR: ${CMAKE_CXX_COMPILER_VERSION_MINOR}")
status("    COMPILER_VERSION_PATCH: ${CMAKE_CXX_COMPILER_VERSION_PATCH}")
status("    COMPILER_VERSION_TWEAK: ${CMAKE_CXX_COMPILER_VERSION_TWEAK}")
status("  CUDA_VERSION: ${CUDA_VERSION}")
status("    CUDA_VERSION_MAJOR: ${CUDA_VERSION_MAJOR}")
status("    CUDA_VERSION_MINOR: ${CUDA_VERSION_MINOR}")
status("    CUDA_VERSION_STRING: ${CUDA_VERSION_STRING}")
status("  OpenCV_VERSION: ${OpenCV_VERSION}")
status("    OpenCV_VERSION_MAJOR: ${OpenCV_VERSION_MAJOR}")
status("    OpenCV_VERSION_MINOR: ${OpenCV_VERSION_MINOR}")
status("    OpenCV_VERSION_PATCH: ${OpenCV_VERSION_PATCH}")
status("    OpenCV_VERSION_TWEAK: ${OpenCV_VERSION_TWEAK}")
status("    OpenCV_VERSION_STATUS: ${OpenCV_VERSION_STATUS}")
status("    OpenCV_WITH_WORLD: ${WITH_OPENCV_WORLD}")
status("  MYNTEYE_VERSION: ${mynteye_VERSION}")
status("    MYNTEYE_VERSION_MAJOR: ${mynteye_VERSION_MAJOR}")
status("    MYNTEYE_VERSION_MINOR: ${mynteye_VERSION_MINOR}")
status("    MYNTEYE_VERSION_PATCH: ${mynteye_VERSION_PATCH}")
status("    MYNTEYE_VERSION_TWEAK: ${mynteye_VERSION_TWEAK}")

status("")
status("Options:")
status("  WITH_API: ${WITH_API}")
if(WITH_API)
  if(OpenCV_FOUND)
    status("    OpenCV: YES")
    status("    OpenCV_VERSION: ${OpenCV_VERSION}")
    status("    OpenCV_WORLD: " IF WITH_OPENCV_WORLD "YES" ELSE "NO")
  else()
    status("    OpenCV: NO")
  endif()
endif()

status("  WITH_BOOST: ${WITH_BOOST}")
if(WITH_BOOST)
  if(Boost_FOUND)
    status("    Boost: YES")
    status("    Boost_VERSION: ${Boost_VERSION_STRING}")
    #status("    Boost_LIBRARIES: ${Boost_LIBRARIES}")
  else()
    status("    Boost: NO")
  endif()
endif()

status("")
status("Features:")
status("  Filesystem: "
  IF WITH_BOOST_FILESYSTEM "boost"
  ELIF WITH_NATIVE_FILESYSTEM "native"
  ELSE "none"
)

status("")
