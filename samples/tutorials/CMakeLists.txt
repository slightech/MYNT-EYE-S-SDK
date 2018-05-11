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

get_filename_component(DIR_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

set_outdir(
  "${OUT_DIR}/lib/${DIR_NAME}"
  "${OUT_DIR}/lib/${DIR_NAME}"
  "${OUT_DIR}/bin/${DIR_NAME}"
)

# make_executable(NAME
#                 [SRCS src1 src2 ...])
#                 [WITH_OPENCV]
#                 [WITH_PCL])
macro(make_executable NAME)
  set(options WITH_OPENCV WITH_PCL)
  set(oneValueArgs)
  set(multiValueArgs SRCS)
  cmake_parse_arguments(THIS "${options}" "${oneValueArgs}"
                        "${multiValueArgs}" ${ARGN})

  set(__link_libs mynteye)
  set(__dll_search_paths ${PRO_DIR}/_install/bin)
  if(THIS_WITH_OPENCV)
    list(APPEND __link_libs ${OpenCV_LIBS})
    list(APPEND __dll_search_paths ${OpenCV_LIB_SEARCH_PATH})
  endif()
  if(THIS_WITH_PCL)
    list(APPEND __link_libs ${PCL_LIBRARIES})
    #list(APPEND __link_libs pcl::pcl)
    list(APPEND __dll_search_paths ${PCL_LIB_SEARCH_PATH})
  endif()

  add_executable(${NAME} ${THIS_SRCS})
  target_link_libraries(${NAME} ${__link_libs})
  target_create_scripts(${NAME} DLL_SEARCH_PATHS ${__dll_search_paths})

  if(THIS_WITH_PCL)
    target_include_directories(${NAME} PRIVATE ${PCL_INCLUDE_DIRS})
    #target_compile_definitions(${NAME} PRIVATE ${PCL_DEFINITIONS})
    #target_compile_options(${NAME} PRIVATE ${PCL_COMPILE_OPTIONS})
  endif()
  if(OS_WIN)
    target_compile_definitions(${NAME}
      PUBLIC GLOG_NO_ABBREVIATED_SEVERITIES
    )
  endif()
endmacro()

if(WITH_API)

# If you install PCL to different directory, please set CMAKE_PREFIX_PATH to find it.
#LIST(APPEND CMAKE_PREFIX_PATH /usr/local/share)
find_package(PCL)
if(PCL_FOUND)
  message(STATUS "Found PCL: ${PCL_VERSION}")

  #message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")
  #message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
  #message(STATUS "PCL_LIBRARY_DIRS: ${PCL_LIBRARY_DIRS}")
  #message(STATUS "PCL_DEFINITIONS: ${PCL_DEFINITIONS}")
  #message(STATUS "PCL_COMPILE_OPTIONS: ${PCL_COMPILE_OPTIONS}")

  if(OS_WIN)
    get_filename_component(PCL_LIB_SEARCH_PATH "${PCL_LIBRARY_DIRS}/../bin" ABSOLUTE)
  else()
    set(PCL_LIB_SEARCH_PATH "${PCL_LIBRARY_DIRS}")
  endif()
else()
  message(WARNING "PCL not found :(")
endif()

# data

make_executable(get_device_info SRCS data/get_device_info.cc)
make_executable(get_img_params SRCS data/get_img_params.cc)
make_executable(get_imu_params SRCS data/get_imu_params.cc)

make_executable(get_stereo SRCS data/get_stereo.cc WITH_OPENCV)
make_executable(get_stereo_rectified SRCS data/get_stereo_rectified.cc WITH_OPENCV)
make_executable(get_disparity SRCS data/get_disparity.cc WITH_OPENCV)
make_executable(get_depth SRCS data/get_depth.cc WITH_OPENCV)
if(PCL_FOUND)
  make_executable(get_points
    SRCS data/get_points.cc data/pc_viewer.cc
    WITH_OPENCV WITH_PCL
  )
endif()
make_executable(get_imu SRCS data/get_imu.cc data/cv_painter.cc WITH_OPENCV)
make_executable(get_with_plugin SRCS data/get_with_plugin.cc WITH_OPENCV)

# control

make_executable(ctrl_infrared SRCS control/infrared.cc WITH_OPENCV)

endif()
