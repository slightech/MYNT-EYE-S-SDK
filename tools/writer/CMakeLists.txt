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

set_outdir(
  "${OUT_DIR}/lib/${DIR_NAME}"
  "${OUT_DIR}/lib/${DIR_NAME}"
  "${OUT_DIR}/bin/${DIR_NAME}"
)

include_directories(
  ${PRO_DIR}/src
)

## device_writer

add_library(device_writer STATIC device_writer.cc)
target_link_libraries(device_writer mynteye ${OpenCV_LIBS})

set(LINK_LIBS device_writer)

## device_info_writer

add_executable(device_info_writer device_info_writer.cc)
target_link_libraries(device_info_writer ${LINK_LIBS})
target_create_scripts(device_info_writer DLL_SEARCH_PATHS
  ${PRO_DIR}/_install/bin
  ${OpenCV_LIB_SEARCH_PATH}
)

if(OS_WIN)
  target_compile_definitions(device_info_writer
    PUBLIC GLOG_NO_ABBREVIATED_SEVERITIES
  )
endif()

## img_params_writer

add_executable(img_params_writer img_params_writer.cc)
target_link_libraries(img_params_writer ${LINK_LIBS})
target_create_scripts(img_params_writer DLL_SEARCH_PATHS
  ${PRO_DIR}/_install/bin
  ${OpenCV_LIB_SEARCH_PATH}
)

if(OS_WIN)
  target_compile_definitions(img_params_writer
    PUBLIC GLOG_NO_ABBREVIATED_SEVERITIES
  )
endif()

## imu_params_writer

add_executable(imu_params_writer imu_params_writer.cc)
target_link_libraries(imu_params_writer ${LINK_LIBS})
target_create_scripts(imu_params_writer DLL_SEARCH_PATHS
  ${PRO_DIR}/_install/bin
  ${OpenCV_LIB_SEARCH_PATH}
)

if(OS_WIN)
  target_compile_definitions(imu_params_writer
    PUBLIC GLOG_NO_ABBREVIATED_SEVERITIES
  )
endif()
