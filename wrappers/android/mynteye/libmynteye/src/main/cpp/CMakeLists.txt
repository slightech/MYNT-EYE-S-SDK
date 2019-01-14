# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.4.1)

get_filename_component(MYNTETE_ROOT "${PROJECT_SOURCE_DIR}/../../../../../../.." ABSOLUTE)
message(STATUS "MYNTETE_ROOT: ${MYNTETE_ROOT}")

set(MYNTEYE_NAME "mynteye")

set(MYNTEYE_NAMESPACE "${MYNTEYE_NAME}")
message(STATUS "Namespace: ${MYNTEYE_NAMESPACE}")

configure_file(
  ${MYNTETE_ROOT}/include/mynteye/mynteye.h.in
  include/mynteye/mynteye.h @ONLY
)

## libs

find_library(log-lib log)

## targets

# libmynteye

add_definitions(-DMYNTEYE_EXPORTS)

set(MYNTEYE_SRCS
  ${MYNTETE_ROOT}/src/mynteye/uvc/linux/uvc-v4l2.cc
  ${MYNTETE_ROOT}/src/mynteye/types.cc
  ${MYNTETE_ROOT}/src/mynteye/util/files.cc
  ${MYNTETE_ROOT}/src/mynteye/util/strings.cc
  ${MYNTETE_ROOT}/src/mynteye/device/channel/bytes.cc
  ${MYNTETE_ROOT}/src/mynteye/device/channel/channels.cc
  ${MYNTETE_ROOT}/src/mynteye/device/channel/file_channel.cc
  ${MYNTETE_ROOT}/src/mynteye/device/config.cc
  ${MYNTETE_ROOT}/src/mynteye/device/context.cc
  ${MYNTETE_ROOT}/src/mynteye/device/device.cc
  ${MYNTETE_ROOT}/src/mynteye/device/motions.cc
  ${MYNTETE_ROOT}/src/mynteye/device/standard/channels_adapter_s.cc
  ${MYNTETE_ROOT}/src/mynteye/device/standard/device_s.cc
  ${MYNTETE_ROOT}/src/mynteye/device/standard/streams_adapter_s.cc
  ${MYNTETE_ROOT}/src/mynteye/device/standard2/channels_adapter_s2.cc
  ${MYNTETE_ROOT}/src/mynteye/device/standard2/device_s2.cc
  ${MYNTETE_ROOT}/src/mynteye/device/standard2/streams_adapter_s2.cc
  ${MYNTETE_ROOT}/src/mynteye/device/standard2/channels_adapter_s210a.cc
  ${MYNTETE_ROOT}/src/mynteye/device/standard2/device_s210a.cc
  ${MYNTETE_ROOT}/src/mynteye/device/standard2/streams_adapter_s210a.cc
  ${MYNTETE_ROOT}/src/mynteye/device/streams.cc
  ${MYNTETE_ROOT}/src/mynteye/device/types.cc
  ${MYNTETE_ROOT}/src/mynteye/device/utils.cc
)

list(APPEND MYNTEYE_SRCS ${MYNTETE_ROOT}/src/mynteye/miniglog.cc)

add_library(${MYNTEYE_NAME} SHARED ${MYNTEYE_SRCS})
target_link_libraries(${MYNTEYE_NAME} ${log-lib})

target_include_directories(${MYNTEYE_NAME} PUBLIC
  "$<BUILD_INTERFACE:${MYNTETE_ROOT}/include>"
  "$<BUILD_INTERFACE:${MYNTETE_ROOT}/src>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
