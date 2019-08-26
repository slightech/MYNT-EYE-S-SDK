cmake_minimum_required(VERSION 3.0)

project(mynteye_demo VERSION 1.0.0 LANGUAGES C CXX)

# flags

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")

# packages

if(MSVC)
  set(SDK_ROOT "$ENV{MYNTEYES_SDK_ROOT}")
  if(SDK_ROOT)
    message(STATUS "MYNTEYES_SDK_ROOT: ${SDK_ROOT}")
    list(APPEND CMAKE_PREFIX_PATH
      "${SDK_ROOT}/lib/cmake"
      "${SDK_ROOT}/3rdparty/opencv/build"
    )
  else()
    message(FATAL_ERROR "MYNTEYES_SDK_ROOT not found, please install SDK firstly")
  endif()
endif()

## mynteye

find_package(mynteye REQUIRED)
message(STATUS "Found mynteye: ${mynteye_VERSION}")

# When SDK build with OpenCV, we can add WITH_OPENCV macro to enable some
# features depending on OpenCV, such as ToMat().
if(mynteye_WITH_OPENCV)
  add_definitions(-DWITH_OPENCV)
endif()

## OpenCV

# Set where to find OpenCV
#set(OpenCV_DIR "/usr/share/OpenCV")

# When SDK build with OpenCV, we must find the same version here.
find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

# targets

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

## mynteye_demo

add_executable(mynteye_demo mynteye_demo.cc)
target_link_libraries(mynteye_demo mynteye ${OpenCV_LIBS})


# Build
#   mkdir _build
#   cd _build
#
#   # win 
#   cmake -G "Visual Studio 15 2017 Win64" ..
#   msbuild.exe ALL_BUILD.vcxproj /property:Configuration=Release
#
#   .\Release\mynteye_demo.exe
#   
#   # unix
#   cmake ..
#   make
#
#   ./mynteye_demo
