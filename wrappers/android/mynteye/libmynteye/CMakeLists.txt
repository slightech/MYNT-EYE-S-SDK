# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.4.1)

get_filename_component(MYNTETE_ROOT "${PROJECT_SOURCE_DIR}/../../../.." ABSOLUTE)
message(STATUS "MYNTETE_ROOT: ${MYNTETE_ROOT}")

get_filename_component(PRO_ROOT "${PROJECT_SOURCE_DIR}/.." ABSOLUTE)
message(STATUS "PRO_ROOT: ${PRO_ROOT}")

set(LIB_ROOT "${PROJECT_SOURCE_DIR}")
message(STATUS "LIB_ROOT: ${LIB_ROOT}")

if(NOT DJINNI_DIR)
  if(DEFINED ENV{DJINNI_DIR})
    set(DJINNI_DIR $ENV{DJINNI_DIR})
  else()
    set(DJINNI_DIR "${PRO_ROOT}/third_party/djinni")
  endif()
endif()

# libs

## log

find_library(log-lib log)

## jnigraphics

find_library(jnigraphics-lib jnigraphics)

## djinni_jni

include_directories(
  ${DJINNI_DIR}/support-lib/jni
)
add_library(djinni_jni STATIC
  ${DJINNI_DIR}/support-lib/jni/djinni_support.cpp
)

## mynteye_internal

add_library(mynteye_internal SHARED IMPORTED)
set_target_properties(mynteye_internal PROPERTIES
  IMPORTED_LOCATION "${LIB_ROOT}/src/main/jniLibs/${ANDROID_ABI}/libmynteye_internal.so"
)

# targets

## libmynteye

add_definitions(-DMYNTEYE_EXPORTS)

set(MYNTEYE_NAMESPACE "mynteye")
#message(STATUS "Namespace: ${MYNTEYE_NAMESPACE}")

configure_file(
  ${MYNTETE_ROOT}/include/mynteye/mynteye.h.in
  include/mynteye/mynteye.h @ONLY
)

set(MYNTEYE_SRCS
  #${MYNTETE_ROOT}/src/mynteye/uvc/linux/uvc-v4l2.cc
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
  ${MYNTETE_ROOT}/src/mynteye/device/streams.cc
  ${MYNTETE_ROOT}/src/mynteye/device/types.cc
  ${MYNTETE_ROOT}/src/mynteye/device/utils.cc
)

list(APPEND MYNTEYE_SRCS ${MYNTETE_ROOT}/src/mynteye/miniglog.cc)

add_library(mynteye STATIC ${MYNTEYE_SRCS})
target_link_libraries(mynteye ${log-lib})

target_include_directories(mynteye PUBLIC
  "$<BUILD_INTERFACE:${MYNTETE_ROOT}/include>"
  "$<BUILD_INTERFACE:${MYNTETE_ROOT}/src>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

## libmynteye_jni

set(CPP_DIR "${PROJECT_SOURCE_DIR}/src/main/cpp")

include_directories(
  ${CPP_DIR}/mynteye/cpp
  ${CPP_DIR}/mynteye/impl
  ${CPP_DIR}/mynteye/jni
)

set(MYNTEYE_JNI_SRCS "")
foreach(__dir cpp impl jni)
  file(GLOB __srcs "${CPP_DIR}/mynteye/${__dir}/*.cpp")
  list(APPEND MYNTEYE_JNI_SRCS ${__srcs})
endforeach()
#message(STATUS "MYNTEYE_JNI_SRCS: ${MYNTEYE_JNI_SRCS}")

add_library(mynteye_jni SHARED
  ${DJINNI_DIR}/support-lib/jni/djinni_main.cpp
  ${CPP_DIR}/mynteye/impl/util/jni_util.cpp
  ${MYNTEYE_JNI_SRCS}
)
target_link_libraries(mynteye_jni ${log-lib} ${jnigraphics-lib} djinni_jni mynteye mynteye_internal)
