cmake_minimum_required(VERSION 3.0)

project(mynteye VERSION 2.0.0 LANGUAGES C CXX)

# options

# flags

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

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

LIST(APPEND CMAKE_MODULE_PATH cmake)

include(CMakePackageConfigHelpers)

# config

set(MYNTEYE_NAMESPACE "mynteye")
message(STATUS "Namespace: ${MYNTEYE_NAMESPACE}")

configure_file(
  include/mynteye/mynteye.h.in
  include/mynteye/mynteye.h @ONLY
)

# targets

include(cmake/Common.cmake)

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

set(MYNTEYE_NAME ${PROJECT_NAME})

set(MYNTEYE_PUBLIC_H
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/global.h
  ${CMAKE_CURRENT_BINARY_DIR}/include/mynteye/mynteye.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mynteye/types.h
)

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
  src/internal/types.cc
  src/public/types.cc
)

set(MYNTEYE_LINKLIBS
  glog::glog
  ${UVC_LIB}
)
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
