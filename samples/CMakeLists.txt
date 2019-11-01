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

project(mynteye_samples VERSION 2.0.0 LANGUAGES C CXX)

get_filename_component(PRO_DIR ${PROJECT_SOURCE_DIR} DIRECTORY)

include(${PRO_DIR}/cmake/Common.cmake)
include(${PRO_DIR}/cmake/Utils.cmake)

# options

include(${PRO_DIR}/cmake/Option.cmake)

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

include(${PRO_DIR}/cmake/DetectOpenCV.cmake)

if(mynteye_WITH_GLOG)
  include(${PRO_DIR}/cmake/DetectGLog.cmake)
endif()

#LIST(APPEND CMAKE_MODULE_PATH ${PRO_DIR}/cmake)




# targets

set(OUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/_output")


include_directories(
  ${PRO_DIR}/src
)

set_outdir(
  "${OUT_DIR}/lib"
  "${OUT_DIR}/lib"
  "${OUT_DIR}/bin"
)

if(OS_MAC)
# samples above uvc layer

  make_executable(camera_u
  SRCS uvc_camera.cc
  LINK_LIBS mynteye ${OpenCV_LIBS}
  DLL_SEARCH_PATHS ${PRO_DIR}/_install/bin ${OpenCV_LIB_SEARCH_PATH}
  )
else()
  # samples above device layer

  make_executable(camera_with_junior_device_api
    SRCS camera_with_junior_device_api.cc
    LINK_LIBS mynteye ${OpenCV_LIBS}
    DLL_SEARCH_PATHS ${PRO_DIR}/_install/bin ${OpenCV_LIB_SEARCH_PATH}
  )

  # samples above api layer

  if(WITH_API)
    ## camera_a

    make_executable(camera_with_senior_api
    SRCS camera_with_senior_api.cc
    LINK_LIBS mynteye ${OpenCV_LIBS}
    DLL_SEARCH_PATHS ${PRO_DIR}/_install/bin ${OpenCV_LIB_SEARCH_PATH}
    )

    ## get_depth_with_region

    make_executable(get_depth_with_region
    SRCS get_depth_with_region.cc
    LINK_LIBS mynteye ${OpenCV_LIBS}
    DLL_SEARCH_PATHS ${PRO_DIR}/_install/bin ${OpenCV_LIB_SEARCH_PATH}
    )
  endif()
endif()



# targets

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)


# tutorials

if(WITH_API)

  # make_executable2(NAME
  #                 [SRCS src1 src2 ...]
  #                 [WITH_OPENCV]
  #                 [WITH_PCL])
  macro(make_executable2 NAME)
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

  # packages

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

  # beginner level

  ## data

  make_executable2(get_device_info SRCS get_device_info.cc WITH_OPENCV)
  make_executable2(get_img_params SRCS get_img_params.cc WITH_OPENCV)
  make_executable2(get_imu_params SRCS get_imu_params.cc WITH_OPENCV)

  make_executable2(get_stereo_rectified SRCS get_stereo_rectified.cc WITH_OPENCV)
  make_executable2(get_disparity SRCS get_disparity.cc WITH_OPENCV)
  make_executable2(get_depth SRCS get_depth.cc WITH_OPENCV)

  make_executable2(get_data_without_select SRCS get_data_without_select.cc WITH_OPENCV)
  make_executable2(get_imu_correspondence
  SRCS get_imu_correspondence.cc util_cv.cc
  WITH_OPENCV
  )
  make_executable2(get_imu SRCS get_imu.cc util_cv.cc WITH_OPENCV)
  make_executable2(save_single_image SRCS save_single_image.cc WITH_OPENCV)
  make_executable2(get_from_callbacks
  SRCS get_from_callbacks.cc util_cv.cc
  WITH_OPENCV
  )
  make_executable2(get_with_plugin SRCS get_with_plugin.cc WITH_OPENCV)

  ## control

  make_executable2(ctrl_framerate SRCS ctrl_framerate.cc WITH_OPENCV)
  make_executable2(ctrl_imu_low_pass_filter SRCS ctrl_imu_low_pass_filter.cc WITH_OPENCV)
  make_executable2(ctrl_imu_range SRCS ctrl_imu_range.cc util_cv.cc WITH_OPENCV)
  make_executable2(ctrl_infrared SRCS ctrl_infrared.cc WITH_OPENCV)
  make_executable2(ctrl_iic_address SRCS ctrl_iic_address.cc WITH_OPENCV)
  make_executable2(ctrl_sync_timestamp SRCS ctrl_sync_timestamp.cc WITH_OPENCV)
  make_executable2(ctrl_auto_exposure
  SRCS ctrl_auto_exposure.cc util_cv.cc
  WITH_OPENCV
  )
  make_executable2(ctrl_manual_exposure
  SRCS ctrl_manual_exposure.cc util_cv.cc
  WITH_OPENCV
  )

  if(PCL_FOUND)

    make_executable2(get_depth_and_points
    SRCS get_depth_and_points.cc util_cv.cc util_pcl.cc
    WITH_OPENCV WITH_PCL
    )

  endif()
endif()

if(OS_WIN)
else()
  ## device_writer

  add_library(device_writer STATIC device_writer.cc)
  target_link_libraries(device_writer mynteye ${OpenCV_LIBS})

  ## device_info_writer

  make_executable(write_device_info
    SRCS write_device_info.cc
    LINK_LIBS device_writer
    DLL_SEARCH_PATHS ${PRO_DIR}/_install/bin ${OpenCV_LIB_SEARCH_PATH}
  )

  ## img_params_writer

  make_executable(write_img_params
    SRCS write_img_params.cc
    LINK_LIBS device_writer
    DLL_SEARCH_PATHS ${PRO_DIR}/_install/bin ${OpenCV_LIB_SEARCH_PATH}
  )

  ## imu_params_writer

  make_executable(write_imu_params
    SRCS write_imu_params.cc
    LINK_LIBS device_writer
    DLL_SEARCH_PATHS ${PRO_DIR}/_install/bin ${OpenCV_LIB_SEARCH_PATH}
  )

  ## save_all_infos

  make_executable(save_all_infos
    SRCS save_all_infos.cc
    LINK_LIBS device_writer
    DLL_SEARCH_PATHS ${PRO_DIR}/_install/bin ${OpenCV_LIB_SEARCH_PATH}
  )

endif()

## record

make_executable(record
  SRCS record.cc dataset.cc
  LINK_LIBS mynteye ${OpenCV_LIBS}
  DLL_SEARCH_PATHS ${PRO_DIR}/_install/bin ${OpenCV_LIB_SEARCH_PATH}
)

