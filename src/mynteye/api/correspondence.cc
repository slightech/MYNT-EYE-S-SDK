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
  // set matched stream to be watched too,
  // aim to make stream and matched stream correspondence
  if (stream_ == Stream::LEFT) {
    stream_match_ = Stream::RIGHT;
  } else if (stream_ == Stream::RIGHT) {
    stream_match_ = Stream::LEFT;
  } else if (stream_ == Stream::LEFT_RECTIFIED) {
    stream_match_ = Stream::RIGHT_RECTIFIED;
  } else if (stream_ == Stream::RIGHT_RECTIFIED) {
    stream_match_ = Stream::LEFT_RECTIFIED;
  } else {
    stream_match_ = Stream::LAST;
  }
  EnableStreamMatch();

  auto framerate = device_->GetOptionValue(Option::FRAME_RATE);
  stream_interval_us_ = 1000000.f / framerate;
  stream_interval_us_half_ = 0.5f * stream_interval_us_;
  VLOG(2) << "framerate: " << framerate
      << ", interval_us: " << stream_interval_us_;
}

Correspondence::~Correspondence() {
  VLOG(2) << __func__;
}

bool Correspondence::Watch(const Stream &stream) const {
  if (stream == stream_) return true;
  if (stream_match_enabled_ && stream == stream_match_) return true;
  return false;
}

void Correspondence::OnStreamDataCallback(
    const Stream &stream, const api::StreamData &data) {
  // LOG(INFO) << __func__ << ", " << stream
  //     << ", id: " << data.frame_id << ", stamp: " << data.img->timestamp;
  if (!Watch(stream)) {
    return;  // unwatched
  }
  std::lock_guard<std::recursive_mutex> _(mtx_stream_datas_);
  if (stream == stream_) {
    stream_datas_.push_back(data);
  } else if (/*stream_match_enabled_ && */stream == stream_match_) {
    stream_datas_match_.push_back(data);
  }
  NotifyStreamDataReady();
}

void Correspondence::OnMotionDataCallback(const device::MotionData &data) {
  // LOG(INFO) << __func__ << ", id: " << data.imu->frame_id
  //     << ", stamp: " << data.imu->timestamp;
  {
    std::lock_guard<std::recursive_mutex> _(mtx_motion_datas_);
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

void Correspondence::WaitForStreams() {
  if (stream_ == Stream::LEFT || stream_ == Stream::RIGHT) {
    // Wait native stream ready, avoid get these stream empty
    // Todo: determine native stream according to device
    WaitStreamDataReady();
    return;
  }
  device_->WaitForStreams();
}

api::StreamData Correspondence::GetStreamData(const Stream &stream) {
  auto datas = GetStreamDatas(stream);
  return datas.empty() ? api::StreamData{} : datas.back();
}

std::vector<api::StreamData> Correspondence::GetStreamDatas(
    const Stream &stream) {
  if (!Watch(stream)) {
    LOG(ERROR) << "Get unwatched stream data of " << stream;
    return {};
  }

  std::lock_guard<std::recursive_mutex> _(mtx_stream_datas_);
  static std::uint32_t stream_count_ = 0;
  static std::uint32_t stream_match_count_ = 0;

  if (stream == stream_) {
    auto datas = std::move(stream_datas_);

    if (stream_count_ < 10) {
      ++stream_count_;
    } else {
      // get stream, but not get matched stream, disable it
      if (stream_match_count_ == 0) {
        DisableStreamMatch();
      }
    }

    return datas;
  } else if (/*stream_match_enabled_ && */stream == stream_match_) {
    auto datas = std::move(stream_datas_match_);

    if (stream_match_count_ < 10) {
      ++stream_match_count_;
    }

    return datas;
  }

  return {};
}

std::vector<api::MotionData> Correspondence::GetMotionDatas() {
  std::lock_guard<std::recursive_mutex> _(mtx_motion_datas_);
  std::vector<api::MotionData> datas;
  for (auto &&data : motion_datas_) {
    datas.push_back({data.imu});
  }
  motion_datas_.clear();
  return datas;
}

void Correspondence::EnableStreamMatch() {
  stream_match_enabled_ = true;
}

void Correspondence::DisableStreamMatch() {
  stream_match_enabled_ = false;
  stream_datas_match_.clear();
}

void Correspondence::WaitStreamDataReady() {
  std::unique_lock<std::recursive_mutex> lock(mtx_stream_datas_);
  auto ready = std::bind(&Correspondence::IsStreamDataReady, this);
  bool ok = cond_stream_datas_.wait_for(lock, std::chrono::seconds(3), ready);
  if (!ok) {
    LOG(FATAL) << "Timeout waiting for key frames. Please use USB 3.0, and not "
                  "in virtual machine.";
  }
}

void Correspondence::NotifyStreamDataReady() {
  cond_stream_datas_.notify_one();
}

bool Correspondence::IsStreamDataReady() {
  if (stream_datas_.empty()) return false;
  if (motion_datas_.empty()) return false;

  std::uint64_t img_stamp = 0;
  {
    std::lock_guard<std::recursive_mutex> _(mtx_stream_datas_);
    auto data = stream_datas_.front();
    if (data.img == nullptr) {
      LOG(FATAL) << "stream data image info is empty!";
    }
    img_stamp = data.img->timestamp;
  }
  std::uint64_t imu_stamp = 0;
  {
    std::lock_guard<std::recursive_mutex> _(mtx_motion_datas_);
    auto data = motion_datas_.back();
    if (data.imu == nullptr) {
      LOG(FATAL) << "motion data imu info is empty!";
    }
    imu_stamp = data.imu->timestamp;
  }

  return img_stamp + stream_interval_us_half_ < imu_stamp;
}

MYNTEYE_END_NAMESPACE
