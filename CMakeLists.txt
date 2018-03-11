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

# config

set(MYNTEYE_NAMESPACE "mynteye")
message(STATUS "Namespace: ${MYNTEYE_NAMESPACE}")

configure_file(
  include/mynteye/mynteye.h.in
  include/mynteye/mynteye.h @ONLY
)

# targets

include(cmake/Common.cmake)

set(OUT_DIR "${PROJECT_SOURCE_DIR}/_output")
set_outdir(
  "${OUT_DIR}/lib"
  "${OUT_DIR}/lib"
  "${OUT_DIR}/bin"
)

include_directories(
  include
  ${CMAKE_BINARY_DIR}/include
)

add_executable(main src/main.cc)
target_link_libraries(main glog::glog)
