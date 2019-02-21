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
#include "mynteye/api/correspondence.h"

#include "mynteye/device/device.h"
#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

Correspondence::Correspondence(const std::shared_ptr<Device> &device,
    const Stream &stream)
  : device_(device), stream_(stream) {
  VLOG(2) << __func__;
}

Correspondence::~Correspondence() {
  VLOG(2) << __func__;
}

void Correspondence::OnStreamDataCallback(
    const Stream &stream, const api::StreamData &data) {
  // LOG(INFO) << __func__ << ", " << stream
  //     << ", id: " << data.frame_id << ", stamp: " << data.img->timestamp;
}

void Correspondence::OnMotionDataCallback(const device::MotionData &data) {
  // LOG(INFO) << __func__ << ", id: " << data.imu->frame_id
  //     << ", stamp: " << data.imu->timestamp;
  {
    std::lock_guard<std::mutex> _(mtx_motion_datas_);
    motion_datas_.push_back(data);
  }
  if (motion_callback_) {
    motion_callback_({data.imu});
  }
}

void Correspondence::SetMotionCallback(API::motion_callback_t callback) {
  // LOG(INFO) << __func__;
  motion_callback_ = callback;
}

std::vector<api::MotionData> Correspondence::GetMotionDatas() {
  std::lock_guard<std::mutex> _(mtx_motion_datas_);
  std::vector<api::MotionData> datas;
  for (auto &&data : motion_datas_) {
    datas.push_back({data.imu});
  }
  motion_datas_.clear();
  return datas;
}

MYNTEYE_END_NAMESPACE
