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
#ifndef MYNTEYE_API_CONFIG_H_
#define MYNTEYE_API_CONFIG_H_
#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <vector>

#include "mynteye/api/api.h"
#include "mynteye/device/callbacks.h"

MYNTEYE_BEGIN_NAMESPACE

class Correspondence {
 public:
  Correspondence(const std::shared_ptr<Device> &device, const Stream &stream);
  ~Correspondence();

  bool Watch(const Stream &stream) const;
  void KeepAccelThenGyro(bool enabled);

  void OnStreamDataCallback(const Stream &stream, const api::StreamData &data);
  void OnMotionDataCallback(const device::MotionData &data);

  void SetMotionCallback(API::motion_callback_t callback);

  void WaitForStreams();
  api::StreamData GetStreamData(const Stream &stream);
  std::vector<api::StreamData> GetStreamDatas(const Stream &stream);
  std::vector<api::MotionData> GetMotionDatas();

 private:
  void EnableStreamMatch();
  void DisableStreamMatch();

  void WaitStreamDataReady();
  void NotifyStreamDataReady();

  bool IsStreamDataReady();

  std::vector<api::StreamData> GetReadyStreamData(bool matched);
  std::vector<api::MotionData> GetReadyMotionDatas();

  void KeepAccelThenGyro(std::vector<api::MotionData> &datas);  // NOLINT

  std::shared_ptr<Device> device_;
  Stream stream_;
  Stream stream_match_;
  std::atomic_bool stream_match_enabled_;

  float stream_interval_us_;
  float stream_interval_us_half_;

  API::motion_callback_t motion_callback_;
  std::vector<device::MotionData> motion_datas_;
  std::recursive_mutex mtx_motion_datas_;

  std::vector<api::StreamData> stream_datas_;
  std::vector<api::StreamData> stream_datas_match_;
  std::recursive_mutex mtx_stream_datas_;
  std::condition_variable_any cond_stream_datas_;

  std::uint64_t ready_image_timestamp_;

  bool keep_accel_then_gyro_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_API_CONFIG_H_
