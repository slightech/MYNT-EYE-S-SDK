cmake_minimum_required(VERSION 3.0)

project(mynteye_samples VERSION 2.0.0 LANGUAGES C CXX)

get_filename_component(PRO_DIR ${PROJECT_SOURCE_DIR} DIRECTORY)

include(${PRO_DIR}/cmake/Common.cmake)

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

LIST(APPEND CMAKE_PREFIX_PATH ${PRO_DIR}/third_party/glog/_build)
find_package(glog REQUIRED)
message(STATUS "Found glog: ${glog_VERSION}")

LIST(APPEND CMAKE_PREFIX_PATH ${PRO_DIR}/_install/lib/cmake)
find_package(mynteye REQUIRED)
message(STATUS "Found mynteye: ${mynteye_VERSION}")

LIST(APPEND CMAKE_MODULE_PATH ${PRO_DIR}/cmake)

find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

# targets

set(OUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/_output")

# samples above device layer

add_subdirectory(device)

# samples above uvc layer

add_subdirectory(uvc)
