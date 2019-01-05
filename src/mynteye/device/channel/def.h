// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef MYNTEYE_DEVICE_CHANNEL_DEF_H_
#define MYNTEYE_DEVICE_CHANNEL_DEF_H_
#pragma once

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

typedef enum Channel {
  CHANNEL_CAM_CTRL = 1,
  CHANNEL_HALF_DUPLEX = 2,
  CHANNEL_IMU_WRITE = 3,
  CHANNEL_IMU_READ = 4,
  CHANNEL_FILE = 5,
  CHANNEL_LAST
} channel_t;

typedef enum FileId {
  FID_DEVICE_INFO = 1,  // device info
  FID_IMG_PARAMS = 2,   // image intrinsics & extrinsics
  FID_IMU_PARAMS = 4,   // imu intrinsics & extrinsics
  FID_LAST,
} file_id_t;

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_CHANNEL_DEF_H_
