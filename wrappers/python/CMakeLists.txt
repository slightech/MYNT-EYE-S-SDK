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

project(mynteye_python VERSION 2.0.0 LANGUAGES C CXX)

get_filename_component(PRO_DIR "${PROJECT_SOURCE_DIR}/../.." ABSOLUTE)

include(${PRO_DIR}/cmake/Common.cmake)

# configs

#set(BOOST_ROOT)
#set(BOOST_FIND_VERSION 1.67)

# Boost Python 2.7
set(PYTHON_FIND_VERSION 2.7)
set(PYTHON_BOOST_SUFFIX 27)
set(PYTHON_BOOST_CODE)
# Boost Python 3.6
#set(PYTHON_FIND_VERSION 3.6)
#set(PYTHON_BOOST_SUFFIX 36)
#set(PYTHON_BOOST_CODE 3)

if(OS_MAC)
  execute_process(
    COMMAND brew --prefix python@2
    OUTPUT_VARIABLE PYTHON_INSTALL_PREFIX
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(PYTHON_INSTALL_PREFIX)
    message(STATUS "PYTHON_INSTALL_PREFIX: ${PYTHON_INSTALL_PREFIX}")
    set(PYTHON_INCLUDE_DIR "${PYTHON_INSTALL_PREFIX}/Frameworks/Python.framework/Headers")
    set(PYTHON_LIBRARY "${PYTHON_INSTALL_PREFIX}/Frameworks/Python.framework/Python")
  endif()
endif()

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

if(NOT mynteye_WITH_API)
  message(FATAL_ERROR "Must with API layer :(")
endif()

include(${PRO_DIR}/cmake/DetectOpenCV.cmake)

if(mynteye_WITH_GLOG)
  include(${PRO_DIR}/cmake/DetectGLog.cmake)
endif()

## boost & python

if(CMAKE_VERSION VERSION_LESS "3.10" OR CMAKE_VERSION VERSION_EQUAL "3.10")
  find_package(Boost ${BOOST_FIND_VERSION} REQUIRED
    COMPONENTS python${PYTHON_BOOST_CODE}  # numpy${PYTHON_BOOST_CODE}
  )
else()
  find_package(Boost ${BOOST_FIND_VERSION} REQUIRED COMPONENTS
    python${PYTHON_BOOST_SUFFIX}
  )
endif()
message(STATUS "Found Boost: ${Boost_VERSION}")
message(STATUS "  Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message(STATUS "  Boost_LIBRARIES: ${Boost_LIBRARIES}")

find_package(PythonLibs ${PYTHON_FIND_VERSION} REQUIRED)
message(STATUS "Found PythonLibs: ${PYTHONLIBS_VERSION_STRING}")
message(STATUS "  PYTHON_INCLUDE_DIRS: ${PYTHON_INCLUDE_DIRS}")
message(STATUS "  PYTHON_LIBRARIES: ${PYTHON_LIBRARIES}")

#LIST(APPEND CMAKE_MODULE_PATH ${PRO_DIR}/cmake)

# targets

set(OUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/_output")

set_outdir(
  "${OUT_DIR}/lib"
  "${OUT_DIR}/lib"
  "${OUT_DIR}/bin"
)

set(DEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/_install")
message(STATUS "DEST_DIR: ${DEST_DIR}")

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${DEST_DIR}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if(OS_WIN)
  add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})
endif()

# add_library_py(NAME
#                [SRCS src1 src2 ...]
#                [LINK_LIBS lib1 lib2 ...])
macro(add_library_py NAME)
  set(options)
  set(oneValueArgs)
  set(multiValueArgs SRCS LINK_LIBS)
  cmake_parse_arguments(THIS "${options}" "${oneValueArgs}"
                        "${multiValueArgs}" ${ARGN})

  add_library(${NAME} SHARED ${THIS_SRCS})
  target_link_libraries(${NAME}
    ${THIS_LINK_LIBS} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES}
  )
  target_include_directories(${NAME} PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIR}
  )
  set_target_properties(${NAME} PROPERTIES PREFIX "")
  if(OS_MAC)
    set_target_properties(${NAME} PROPERTIES SUFFIX ".so")
  endif()
endmacro()

## mynteye_py

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/array
)

#set(PBCVT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pyboostcvconverter")
#
#include_directories(
#  ${PBCVT_DIR}/include
#)
#
#set(PBCVT_SRCS
#  ${PBCVT_DIR}/src/pyboost_cv2_converter.cpp
#  ${PBCVT_DIR}/src/pyboost_cv3_converter.cpp
#  ${PBCVT_DIR}/include/pyboostcvconverter/pyboostcvconverter.hpp
#)

set(NPCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/numpy-opencv-converter")

include_directories(
  ${NPCV_DIR}
)

set(NPCV_SRCS
  ${NPCV_DIR}/np_opencv_converter.cpp
  ${NPCV_DIR}/utils/conversion.cpp
)

add_library_py(mynteye_py
  SRCS src/mynteye_py.cc ${NPCV_SRCS}
  LINK_LIBS mynteye ${OpenCV_LIBS}
)

# insall

install(TARGETS mynteye_py
  ARCHIVE DESTINATION "${DEST_DIR}/lib"
  LIBRARY DESTINATION "${DEST_DIR}/lib"
  RUNTIME DESTINATION "${DEST_DIR}/bin"
)
