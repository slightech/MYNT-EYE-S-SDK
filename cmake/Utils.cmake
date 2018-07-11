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

include(${CMAKE_CURRENT_LIST_DIR}/Common.cmake)

# make_executable(NAME
#                 [SRCS src1 src2 ...]
#                 [LINK_LIBS lib1 lib2 ...]
#                 [DLL_SEARCH_PATHS path1 path2 ...]
#                 [WITH_THREAD])
macro(make_executable NAME)
  set(options WITH_THREAD)
  set(oneValueArgs)
  set(multiValueArgs SRCS LINK_LIBS DLL_SEARCH_PATHS)
  cmake_parse_arguments(THIS "${options}" "${oneValueArgs}"
                        "${multiValueArgs}" ${ARGN})

  add_executable(${NAME} ${THIS_SRCS})
  target_link_libraries(${NAME} ${THIS_LINK_LIBS})
  target_create_scripts(${NAME} DLL_SEARCH_PATHS ${THIS_DLL_SEARCH_PATHS})

  if(OS_WIN)
    target_compile_definitions(${NAME}
      PUBLIC GLOG_NO_ABBREVIATED_SEVERITIES
    )
  endif()

  if(THIS_WITH_THREAD)
    #find_package(Threads REQUIRED)
    if(THREADS_HAVE_PTHREAD_ARG)
      target_compile_options(PUBLIC ${NAME} "-pthread")
    endif()
    if(CMAKE_THREAD_LIBS_INIT)
      target_link_libraries(${NAME} "${CMAKE_THREAD_LIBS_INIT}")
    endif()
  endif()
endmacro()

# set_default_value(VARIABLE DEFAULT)
macro(set_default_value VARIABLE DEFAULT)
  if(NOT ${VARIABLE})
    set(${VARIABLE} ${DEFAULT})
  endif()
endmacro()

# set_version_values(VARIABLE)
macro(set_version_values VARIABLE)
  string(REPLACE "." ";" __version_list "${${VARIABLE}}")
  list(LENGTH __version_list __len)
  if(${__len} GREATER 0)
    list(GET __version_list 0 ${VARIABLE}_MAJOR)
  endif()
  if(${__len} GREATER 1)
    list(GET __version_list 1 ${VARIABLE}_MINOR)
  endif()
  if(${__len} GREATER 2)
    list(GET __version_list 2 ${VARIABLE}_PATCH)
  endif()
  if(${__len} GREATER 3)
    list(GET __version_list 3 ${VARIABLE}_TWEAK)
  endif()
endmacro()

# status(TEXT [IF cond text [ELIF cond text] [ELSE cond text]])
macro(status TEXT)
  set(options)
  set(oneValueArgs)
  set(multiValueArgs IF ELIF ELSE)
  cmake_parse_arguments(THIS "${options}" "${oneValueArgs}"
                        "${multiValueArgs}" ${ARGN})

  #message(STATUS "TEXT: ${TEXT}")
  #message(STATUS "THIS_IF: ${THIS_IF}")
  #message(STATUS "THIS_ELIF: ${THIS_ELIF}")
  #message(STATUS "THIS_ELSE: ${THIS_ELSE}")

  set(__msg_list "${TEXT}")
  set(__continue TRUE)

  if(__continue AND DEFINED THIS_IF)
    #message(STATUS "-- THIS_IF: ${THIS_IF}")
    list(LENGTH THIS_IF __if_len)
    if(${__if_len} GREATER 1)
      list(GET THIS_IF 0 __if_cond)
      if(${__if_cond})
        list(REMOVE_AT THIS_IF 0)
        list(APPEND __msg_list ${THIS_IF})
        set(__continue FALSE)
      endif()
    else()
      message(FATAL_ERROR "status() IF must have cond and text, >= 2 items")
    endif()
  endif()

  if(__continue AND DEFINED THIS_ELIF)
    #message(STATUS "-- THIS_ELIF: ${THIS_ELIF}")
    list(LENGTH THIS_ELIF __elif_len)
    if(${__elif_len} GREATER 1)
      list(GET THIS_ELIF 0 __elif_cond)
      if(${__elif_cond})
        list(REMOVE_AT THIS_ELIF 0)
        list(APPEND __msg_list ${THIS_ELIF})
        set(__continue FALSE)
      endif()
    else()
      message(FATAL_ERROR "status() ELIF must have cond and text, >= 2 items")
    endif()
  endif()

  if(__continue AND DEFINED THIS_ELSE)
    #message(STATUS "-- THIS_ELSE: ${THIS_ELSE}")
    list(LENGTH THIS_ELSE __else_len)
    if(${__else_len} GREATER 0)
      list(APPEND __msg_list ${THIS_ELSE})
    else()
      message(FATAL_ERROR "status() ELSE must have text, >= 1 items")
    endif()
  endif()

  string(REPLACE ";" "" __msg_list "${__msg_list}")
  message(STATUS "${__msg_list}")
endmacro()
