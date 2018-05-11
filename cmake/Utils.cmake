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
