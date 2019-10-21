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

project(mynteye VERSION 2.5.0 LANGUAGES C CXX)

include(cmake/Common.cmake)

if(NOT CMAKE_DEBUG_POSTFIX)
  set(CMAKE_DEBUG_POSTFIX d)
endif()

LIST(APPEND CMAKE_MODULE_PATH cmake)

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

find_package(Threads QUIET)

macro(target_link_threads NAME)
  if(THREADS_HAVE_PTHREAD_ARG)
    target_compile_options(PUBLIC ${NAME} "-pthread")
  endif()
  if(CMAKE_THREAD_LIBS_INIT)
    target_link_libraries(${NAME} PUBLIC "${CMAKE_THREAD_LIBS_INIT}")
  endif()
endmacro()

include(CMakePackageConfigHelpers)

# config

set(MYNTEYE_NAME ${PROJECT_NAME})

set(MYNTEYE_NAMESPACE "mynteye")
message(STATUS "Namespace: ${MYNTEYE_NAMESPACE}")

if(MSVC)
  string(REPLACE "/" "\\\\" MYNTEYE_SDK_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
  string(REPLACE "/" "\\\\" MYNTEYE_SDK_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}")
else()
  file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" MYNTEYE_SDK_ROOT_DIR)
  file(TO_NATIVE_PATH "${CMAKE_INSTALL_PREFIX}" MYNTEYE_SDK_INSTALL_DIR)
endif()

configure_file(
  include/mynteye/mynteye.h.in
  include/mynteye/mynteye.h @ONLY
)

configure_file(
  cmake/templates/pkginfo.sh.in
  ${CMAKE_CURRENT_SOURCE_DIR}/pkginfo.sh @ONLY
)
if(OS_WIN)
  configure_file(
    scripts/win/nsis/winpack.nsi.in
    ${CMAKE_CURRENT_SOURCE_DIR}/winpack.nsi @ONLY
  )
endif()

# rpath

set(CMAKE_MACOSX_RPATH 1)
set(MYNTEYE_CMAKE_RPATH "")
if(WITH_OPENCV)
  list(APPEND MYNTEYE_CMAKE_RPATH ${OpenCV_LIB_PATH})
endif()
if(MYNTEYE_CMAKE_RPATH)
  message(STATUS "RPATH: ${MYNTEYE_CMAKE_RPATH}")
  set(CMAKE_INSTALL_RPATH "${MYNTEYE_CMAKE_RPATH}")
endif()

# targets

add_definitions(-DMYNTEYE_EXPORTS)

set(OUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/_output")
set_outdir(
  "${OUT_DIR}/lib"
  "${OUT_DIR}/lib"
  "${OUT_DIR}/bin"
)

## main

if(WITH_GLOG)
  add_executable(main src/main.cc)
  target_link_libraries(main glog::glog)
  target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}/include
  )
endif()

## camera_models

if(WITH_CAM_MODELS)

  include_directories(
    include
    ${CMAKE_CURRENT_BINARY_DIR}/include
    src/mynteye/api/camera_models
    src/mynteye
  )

  add_library(camera_models STATIC
    src/mynteye/api/camera_models/camera.cc
    src/mynteye/api/camera_models/equidistant_camera.cc
    src/mynteye/api/camera_models/util/gpl.cc
  )

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

## libmynteye

if(NOT WITH_GLOG AND NOT OS_WIN)
  set(__MINIGLOG_FLAGS "-Wno-unused-parameter -Wno-format -Wno-return-type -Wno-comment")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${__MINIGLOG_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${__MINIGLOG_FLAGS}")
  unset(__MINIGLOG_FLAGS)
endif()
if(NOT WITH_GLOG)
  list(APPEND MYNTEYE_PUBLIC_H
    ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/miniglog.h
  )
endif()

if(OS_WIN)
  set(UVC_SRC src/mynteye/uvc/win/uvc-wmf.cc)
elseif(OS_MAC)
  add_compile_options(-x objective-c++ -Wno-unused-command-line-argument -Wno-missing-method-return-type -Wno-sign-compare)

  INCLUDE_DIRECTORIES(src/mynteye/uvc/macosx/USBBusProber.framework/Headers)
  INCLUDE_DIRECTORIES(src/mynteye/uvc/macosx/VVUVCKit.framework/Headers)
  find_library(VVUVCKIT_LIBRARY VVUVCKit PATHS src/mynteye/uvc/macosx)
  find_library(USB_LIBRARY USBBusProber PATHS src/mynteye/uvc/macosx)
  MARK_AS_ADVANCED (VVUVCKIT_LIBRARY USB_LIBRARY)
  SET(OSX_EXTRA_LIBS ${VVUVCKIT_LIBRARY} ${USB_LIBRARY})

  set(UVC_SRC src/mynteye/uvc/macosx/CameraEngine.cpp src/mynteye/uvc/macosx/AVfoundationCamera.mm src/mynteye/uvc/macosx/uvc-vvuvckit.cc )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -framework CoreFoundation -framework AVFoundation -framework IOKit -framework AppKit -framework Cocoa -framework CoreMedia -framework CoreData -framework Foundation -framework CoreVideo ${__MACUVCLOG_FLAGS}")
elseif(OS_LINUX)
  set(UVC_SRC src/mynteye/uvc/linux/uvc-v4l2.cc)
else()
  message(FATAL_ERROR "Unsupported OS.")
endif()

set(MYNTEYE_SRCS
  ${UVC_SRC}
  src/mynteye/types.cc
  src/mynteye/util/files.cc
  src/mynteye/util/strings.cc
  src/mynteye/device/channel/bytes.cc
  src/mynteye/device/channel/channels.cc
  src/mynteye/device/channel/file_channel.cc
  src/mynteye/device/config.cc
  src/mynteye/device/context.cc
  src/mynteye/device/device.cc
  src/mynteye/device/motions.cc
  src/mynteye/device/standard/channels_adapter_s.cc
  src/mynteye/device/standard/device_s.cc
  src/mynteye/device/standard/streams_adapter_s.cc
  src/mynteye/device/standard2/channels_adapter_s2.cc
  src/mynteye/device/standard2/device_s2.cc
  src/mynteye/device/standard2/streams_adapter_s2.cc
  src/mynteye/device/streams.cc
  src/mynteye/device/types.cc
  src/mynteye/device/utils.cc
)
if(WITH_API)
  list(APPEND MYNTEYE_SRCS
    src/mynteye/api/api.cc
    src/mynteye/api/dl.cc
    src/mynteye/api/processor.cc
    src/mynteye/api/synthetic.cc
    src/mynteye/api/processor/disparity_processor.cc
    src/mynteye/api/processor/disparity_normalized_processor.cc
    src/mynteye/api/processor/root_camera_processor.cc
    src/mynteye/api/processor/rectify_processor_ocv.cc
    src/mynteye/api/correspondence.cc
    src/mynteye/api/version_checker.cc
    src/mynteye/api/data_tools.cc
  )
  if(WITH_CAM_MODELS)
    list(APPEND MYNTEYE_SRCS
      src/mynteye/api/processor/depth_processor.cc
      src/mynteye/api/processor/points_processor.cc
      src/mynteye/api/processor/rectify_processor.cc
    )
  endif()
endif()

if(NOT WITH_GLOG)
  list(APPEND MYNTEYE_SRCS src/mynteye/miniglog.cc)
endif()

set(MYNTEYE_LINKLIBS ${UVC_LIB})
if(WITH_API)
  list(APPEND MYNTEYE_LINKLIBS ${OpenCV_LIBS})
endif()
if(WITH_BOOST_FILESYSTEM)
  list(APPEND MYNTEYE_LINKLIBS ${Boost_LIBRARIES})
endif()
if(WITH_GLOG)
  list(APPEND MYNTEYE_LINKLIBS glog::glog)
endif()

#message(STATUS "MYNTEYE_LINKLIBS: ${MYNTEYE_LINKLIBS}")

add_library(${MYNTEYE_NAME} SHARED ${MYNTEYE_SRCS})
target_link_libraries(${MYNTEYE_NAME} PUBLIC ${MYNTEYE_LINKLIBS})
if(OS_MAC)
  target_link_libraries(${MYNTEYE_NAME} PUBLIC ${OSX_EXTRA_LIBS} )
endif()
if(WITH_CAM_MODELS)
  target_link_libraries(${MYNTEYE_NAME} PRIVATE camera_models)
endif()
target_link_threads(${MYNTEYE_NAME})

if(OS_WIN)
  target_compile_definitions(${MYNTEYE_NAME}
    PUBLIC GLOG_NO_ABBREVIATED_SEVERITIES
  )
endif()

target_include_directories(${MYNTEYE_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
  "$<INSTALL_INTERFACE:include>"
)

set_target_properties(${MYNTEYE_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
)

# install

set(MYNTEYE_CMAKE_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include/${MYNTEYE_NAME}")
set(MYNTEYE_CMAKE_BINDIR "${CMAKE_INSTALL_PREFIX}/bin")
set(MYNTEYE_CMAKE_LIBDIR "${CMAKE_INSTALL_PREFIX}/lib")
set(MYNTEYE_CMAKE_INSTALLDIR "${MYNTEYE_CMAKE_LIBDIR}/cmake/${MYNTEYE_NAME}")

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/include/mynteye/mynteye.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/global.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/logger.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/types.h
  DESTINATION ${MYNTEYE_CMAKE_INCLUDE_DIR}
)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/device/callbacks.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/device/context.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/device/device.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/device/types.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/device/utils.h
  DESTINATION ${MYNTEYE_CMAKE_INCLUDE_DIR}/device
)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/util/files.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/util/strings.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/util/times.h
  DESTINATION ${MYNTEYE_CMAKE_INCLUDE_DIR}/util
)
if(WITH_API)
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/api/api.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/api/plugin.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/api/object.h
    DESTINATION ${MYNTEYE_CMAKE_INCLUDE_DIR}/api
  )
endif()
if(NOT WITH_GLOG)
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/miniglog.h
    DESTINATION ${MYNTEYE_CMAKE_INCLUDE_DIR}
  )
endif()

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/deprecated/mynteye/callbacks.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/deprecated/mynteye/context.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/deprecated/mynteye/device.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/deprecated/mynteye/files.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/deprecated/mynteye/glog_init.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/deprecated/mynteye/strings.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/deprecated/mynteye/times.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/deprecated/mynteye/utils.h
  DESTINATION ${MYNTEYE_CMAKE_INCLUDE_DIR}
)
if(WITH_API)
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/deprecated/mynteye/api.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/deprecated/mynteye/plugin.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/deprecated/mynteye/object.h
    DESTINATION ${MYNTEYE_CMAKE_INCLUDE_DIR}
  )
endif()

install(TARGETS ${MYNTEYE_NAME}
  EXPORT ${MYNTEYE_NAME}-targets
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

## build.info

# set default int values for yaml file (build.info)
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

configure_file(
  cmake/templates/build.info.in
  build.info @ONLY
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/build.info
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/mynteye
)
