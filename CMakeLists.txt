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

project(mynteye VERSION 2.0.0 LANGUAGES C CXX)

include(cmake/Common.cmake)

# options

include(cmake/Option.cmake)

# flags

if(OS_WIN)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
else()
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

include(cmake/DetectCXX11.cmake)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

string(STRIP "${CMAKE_C_FLAGS}" CMAKE_C_FLAGS)
string(STRIP "${CMAKE_CXX_FLAGS}" CMAKE_CXX_FLAGS)
message(STATUS "C_FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

# packages

LIST(APPEND CMAKE_PREFIX_PATH third_party/glog/_build)
find_package(glog REQUIRED)
message(STATUS "Found glog: ${glog_VERSION}")

if(WITH_API)
  find_package(OpenCV REQUIRED)
  message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

  if(OpenCV_VERSION VERSION_LESS 3.0)
    add_definitions(-DUSE_OPENCV2)
  else()
    add_definitions(-DUSE_OPENCV3)
  endif()
endif()

LIST(APPEND CMAKE_MODULE_PATH cmake)

include(CMakePackageConfigHelpers)

# config

set(MYNTEYE_NAME ${PROJECT_NAME})

set(MYNTEYE_NAMESPACE "mynteye")
message(STATUS "Namespace: ${MYNTEYE_NAMESPACE}")

configure_file(
  include/mynteye/mynteye.h.in
  include/mynteye/mynteye.h @ONLY
)

# targets

add_definitions(-DMYNTEYE_EXPORTS)

set(OUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/_output")
set_outdir(
  "${OUT_DIR}/lib"
  "${OUT_DIR}/lib"
  "${OUT_DIR}/bin"
)

set(MYNTEYE_CMAKE_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include")
set(MYNTEYE_CMAKE_BINDIR "${CMAKE_INSTALL_PREFIX}/bin")
set(MYNTEYE_CMAKE_LIBDIR "${CMAKE_INSTALL_PREFIX}/lib")
set(MYNTEYE_CMAKE_INSTALLDIR "${MYNTEYE_CMAKE_LIBDIR}/cmake/${MYNTEYE_NAME}")

## main

add_executable(main src/main.cc)
target_link_libraries(main glog::glog)
target_include_directories(main PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/include
)

## libmynteye

set(MYNTEYE_PUBLIC_H
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/callbacks.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/global.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/glog_init.h
  ${CMAKE_CURRENT_BINARY_DIR}/include/mynteye/mynteye.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/types.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/utils.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/device/context.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/device/device.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/internal/files.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/internal/strings.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/internal/times.h
)
if(WITH_API)
  list(APPEND MYNTEYE_PUBLIC_H
    ${CMAKE_CURRENT_SOURCE_DIR}/src/api/api.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/api/plugin.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/api/processor/object.h
  )
endif()

if(OS_WIN)
  set(UVC_SRC src/uvc/uvc-wmf.cc)
elseif(OS_MAC)
  set(UVC_SRC src/uvc/uvc-libuvc.cc)

  find_package(libuvc REQUIRED)
  set(UVC_LIB ${libuvc_LIBRARIES})

  include_directories(${libuvc_INCLUDE_DIRS})
elseif(OS_LINUX)
  set(UVC_SRC src/uvc/uvc-v4l2.cc)
else()
  message(FATAL_ERROR "Unsupported OS.")
endif()

set(MYNTEYE_SRCS
  ${UVC_SRC}
  src/internal/channels.cc
  src/internal/config.cc
  src/internal/dl.cc
  src/internal/files.cc
  src/internal/motions.cc
  src/internal/streams.cc
  src/internal/strings.cc
  src/internal/types.cc
  src/public/types.cc
  src/public/utils.cc
  src/device/context.cc
  src/device/device.cc
  src/device/device_s.cc
)
if(WITH_API)
  list(APPEND MYNTEYE_SRCS
    src/api/api.cc
    src/api/synthetic.cc
    src/api/processor/processor.cc
    src/api/processor/rectify_processor.cc
    src/api/processor/disparity_processor.cc
    src/api/processor/disparity_normalized_processor.cc
    src/api/processor/depth_processor.cc
    src/api/processor/points_processor.cc
  )
endif()

set(MYNTEYE_LINKLIBS
  glog::glog
  ${UVC_LIB}
)
if(WITH_API)
  list(APPEND MYNTEYE_LINKLIBS ${OpenCV_LIBS})
endif()
#message(STATUS "MYNTEYE_LINKLIBS: ${MYNTEYE_LINKLIBS}")

add_library(${MYNTEYE_NAME} SHARED ${MYNTEYE_SRCS})
target_link_libraries(${MYNTEYE_NAME} ${MYNTEYE_LINKLIBS})

if(OS_WIN)
  target_compile_definitions(${MYNTEYE_NAME}
    PUBLIC GLOG_NO_ABBREVIATED_SEVERITIES
  )
endif()

target_include_directories(${MYNTEYE_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
  "$<INSTALL_INTERFACE:${MYNTEYE_CMAKE_INCLUDE_DIR}>"
)

set_target_properties(${MYNTEYE_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
)
set_target_properties(${MYNTEYE_NAME} PROPERTIES
  PUBLIC_HEADER "${MYNTEYE_PUBLIC_H}"
)

# install

#message(STATUS "MYNTEYE_CMAKE_INCLUDE_DIR: ${MYNTEYE_CMAKE_INCLUDE_DIR}")
#message(STATUS "MYNTEYE_CMAKE_BINDIR: ${MYNTEYE_CMAKE_BINDIR}")
#message(STATUS "MYNTEYE_CMAKE_LIBDIR: ${MYNTEYE_CMAKE_LIBDIR}")
#message(STATUS "MYNTEYE_CMAKE_INSTALLDIR: ${MYNTEYE_CMAKE_INSTALLDIR}")

install(TARGETS ${MYNTEYE_NAME}
  EXPORT ${MYNTEYE_NAME}-targets
  PUBLIC_HEADER DESTINATION ${MYNTEYE_CMAKE_INCLUDE_DIR}/${MYNTEYE_NAME}
  RUNTIME DESTINATION ${MYNTEYE_CMAKE_BINDIR}
  LIBRARY DESTINATION ${MYNTEYE_CMAKE_LIBDIR}
  ARCHIVE DESTINATION ${MYNTEYE_CMAKE_LIBDIR}
)

configure_package_config_file(mynteye-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/mynteye-config.cmake
  INSTALL_DESTINATION ${MYNTEYE_CMAKE_INSTALLDIR}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(mynteye-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/mynteye-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/mynteye-config-version.cmake
  DESTINATION ${MYNTEYE_CMAKE_INSTALLDIR}
)

install(EXPORT ${MYNTEYE_NAME}-targets
  DESTINATION ${MYNTEYE_CMAKE_INSTALLDIR}
)
