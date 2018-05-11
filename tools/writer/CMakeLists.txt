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

## device_info_writer

make_executable(device_info_writer
  SRCS device_info_writer.cc
  LINK_LIBS device_writer
  DLL_SEARCH_PATHS ${PRO_DIR}/_install/bin ${OpenCV_LIB_SEARCH_PATH}
)

## img_params_writer

make_executable(img_params_writer
  SRCS img_params_writer.cc
  LINK_LIBS device_writer
  DLL_SEARCH_PATHS ${PRO_DIR}/_install/bin ${OpenCV_LIB_SEARCH_PATH}
)

## imu_params_writer

make_executable(imu_params_writer
  SRCS imu_params_writer.cc
  LINK_LIBS device_writer
  DLL_SEARCH_PATHS ${PRO_DIR}/_install/bin ${OpenCV_LIB_SEARCH_PATH}
)

## save_all_infos

make_executable(save_all_infos
  SRCS save_all_infos.cc
  LINK_LIBS device_writer
  DLL_SEARCH_PATHS ${PRO_DIR}/_install/bin ${OpenCV_LIB_SEARCH_PATH}
)
