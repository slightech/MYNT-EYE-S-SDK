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

include(CMakeParseArguments)

set(CUR_DIR ${CMAKE_CURRENT_LIST_DIR})

if(MSVC OR MSYS OR MINGW)
  set(OS_WIN TRUE)
  set(HOST_OS Win)
elseif(APPLE)
  set(OS_MAC TRUE)
  set(HOST_OS Mac)
elseif(UNIX)
  set(OS_LINUX TRUE)
  set(HOST_OS Linux)
else()
  message(FATAL_ERROR "Unsupported OS.")
endif()

set(HOST_NAME "${HOST_OS}")

if(OS_LINUX)
  execute_process(COMMAND uname -a COMMAND tr -d '\n' OUTPUT_VARIABLE UNAME_A)
  string(TOLOWER "${UNAME_A}" UNAME_A)
  if(${UNAME_A} MATCHES ".*(tegra|jetsonbot).*")
    set(OS_TEGRA TRUE)
    set(HOST_NAME Tegra)
  elseif(${UNAME_A} MATCHES ".*ubuntu.*")
    set(OS_UBUNTU TRUE)
    set(HOST_NAME Ubuntu)
  endif()
endif()

include(${CMAKE_CURRENT_LIST_DIR}/TargetArch.cmake)
target_architecture(HOST_ARCH)
message(STATUS "HOST_ARCH: ${HOST_ARCH}")

if(CMAKE_COMPILER_IS_GNUCC)
  execute_process(COMMAND gcc -dumpversion COMMAND cut -c 1-3 COMMAND tr -d '\n' OUTPUT_VARIABLE GCC_VERSION)
  string(REGEX MATCHALL "[0-9]+" GCC_VERSION_COMPONMENTS ${GCC_VERSION})
  list(GET GCC_VERSION_COMPONMENTS 0 GCC_VERSION_MAJOR)
  list(GET GCC_VERSION_COMPONMENTS 1 GCC_VERSION_MINOR)
  message(STATUS "GCC_VERSION: ${GCC_VERSION}")
  #message(STATUS "GCC_VERSION_MAJOR: ${GCC_VERSION_MAJOR}")
  #message(STATUS "GCC_VERSION_MINOR: ${GCC_VERSION_MINOR}")
endif()

# set_outdir(ARCHIVE_OUTPUT_DIRECTORY
#            LIBRARY_OUTPUT_DIRECTORY
#            RUNTIME_OUTPUT_DIRECTORY)
macro(set_outdir ARCHIVE_OUTPUT_DIRECTORY LIBRARY_OUTPUT_DIRECTORY RUNTIME_OUTPUT_DIRECTORY)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ARCHIVE_OUTPUT_DIRECTORY})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_DIRECTORY})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RUNTIME_OUTPUT_DIRECTORY})
  foreach(CONFIG ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${CONFIG} CONFIG)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIG} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIG} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIG} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  endforeach()
endmacro()

macro(exe2bat exe_name exe_dir dll_search_paths)
  message(STATUS "Generating ${exe_name}.bat")
  set(__exe_name ${exe_name})
  set(__dll_relative_search_paths "")
  foreach(path ${dll_search_paths})
    file(RELATIVE_PATH __relative_path "${exe_dir}" "${path}")
    file(TO_NATIVE_PATH ${__relative_path} __relative_path)
    list(APPEND __dll_relative_search_paths ${__relative_path})
  endforeach()
  #message(STATUS __dll_relative_search_paths: "${__dll_relative_search_paths}")
  set(__dll_search_paths "${__dll_relative_search_paths}")
  configure_file(
    "${CUR_DIR}/templates/exe.bat.in"
    "${exe_dir}/${__exe_name}.bat"
  )
endmacro()

# target_create_scripts(NAME
#                       [BIN_DIR bin_dir]
#                       [DLL_SEARCH_PATHS path1 path2 ...])
macro(target_create_scripts NAME)
  set(options)
  set(oneValueArgs BIN_DIR)
  set(multiValueArgs DLL_SEARCH_PATHS)
  cmake_parse_arguments(THIS "${options}" "${oneValueArgs}"
                        "${multiValueArgs}" ${ARGN})
  if(NOT THIS_BIN_DIR)
    set(THIS_BIN_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  endif()

  #message(STATUS "NAME: ${NAME}")
  #message(STATUS "THIS_BIN_DIR: ${THIS_BIN_DIR}")
  #message(STATUS "THIS_DLL_SEARCH_PATHS: ${THIS_DLL_SEARCH_PATHS}")

  if(OS_WIN)
    exe2bat("${NAME}" "${THIS_BIN_DIR}" "${THIS_DLL_SEARCH_PATHS}")
  endif()
endmacro()
