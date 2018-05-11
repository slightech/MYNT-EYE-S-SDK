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

if(MSVC)

# Support For C++11/14/17 Features (Modern C++)
#   https://msdn.microsoft.com/en-us/library/hh567368.aspx
# MSVC_VERSION:
#   https://cmake.org/cmake/help/latest/variable/MSVC_VERSION.html

if(NOT (MSVC_VERSION LESS 1600))
  message(STATUS "Visual Studio >= 2010, MSVC >= 10.0")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

else()

set(CXX_FLAGS_EXTRA "")

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CXX_FLAGS_EXTRA "-std=c++11")
  message(STATUS "Using flag -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CXX_FLAGS_EXTRA "-std=c++0x")
  message(STATUS "Using flag -std=c++0x")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_FLAGS_EXTRA}")

# Ensure access this in sub directories
set(CXX_FLAGS_EXTRA "${CXX_FLAGS_EXTRA}" CACHE STRING "Value of the extra cxx flags.")

endif()
