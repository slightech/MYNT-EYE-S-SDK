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
#include "mynteye/api/processor/root_camera_processor.h"

#include <utility>
#include <vector>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "mynteye/logger.h"
#include "mynteye/api/synthetic.h"
#include "mynteye/device/device.h"
#include "mynteye/api/data_tools.h"
MYNTEYE_BEGIN_NAMESPACE
const char RootProcessor::NAME[] = "RootProcessor";

RootProcessor::RootProcessor(std::shared_ptr<Device> device,
    std::int32_t proc_period)
    : Processor(std::move(proc_period)),
    device_(device) {}
RootProcessor::~RootProcessor() {
  VLOG(2) << __func__;
}

std::string RootProcessor::Name() {
  return NAME;
}

s1s2Processor::s1s2Processor(std::shared_ptr<Device> device,
    std::int32_t proc_period)
    : RootProcessor(device, std::move(proc_period)) {}
s1s2Processor::~s1s2Processor() {
  VLOG(2) << __func__;
}

Object *s1s2Processor::OnCreateOutput() {
    return new ObjMat2();
}
bool s1s2Processor::OnProcess(
    Object *const in, Object *const out,
    std::shared_ptr<Processor> const parent) {
  const ObjMat2 *input = Object::Cast<ObjMat2>(in);
  ObjMat2 *output = Object::Cast<ObjMat2>(out);
  output->second = input->second;
  output->first = input->first;
  output->first_id = input->first_id;
  output->first_data = input->first_data;
  output->second_id = input->second_id;
  output->second_data = input->second_data;
  MYNTEYE_UNUSED(parent)
  return true;
}

void s1s2Processor::ProcessNativeStream(
    const Stream &stream, const api::StreamData &data) {
  std::unique_lock<std::mutex> lk(mtx_left_right_ready_);
  static api::StreamData left_data, right_data;
  if (stream == Stream::LEFT) {
    left_data = data;
  } else if (stream == Stream::RIGHT) {
    right_data = data;
  }
  if (left_data.img && right_data.img &&
      left_data.img->frame_id == right_data.img->frame_id) {
    Process(std::make_shared<ObjMat2>(data_obj(left_data, right_data)));
  }
  return;
}

void s1s2Processor::StartVideoStreaming() {
  Activate();
  auto streams = getTargetStreams();
  for (unsigned int j =0; j< streams.size(); j++) {
    auto stream = streams[j].stream;
    auto callback = streams[j].stream_callback;
    target_streams_[j].enabled_mode_ = Synthetic::MODE_ON;
    device_->SetStreamCallback(
      stream,
      [this, stream, callback](const device::StreamData &data) {
        auto &&stream_data = data2api(data);
        ProcessNativeStream(stream, stream_data);
      },
      true);
  }
  device_->Start(Source::VIDEO_STREAMING);
}

void s1s2Processor::StopVideoStreaming() {
  Deactivate();
  auto streams = getTargetStreams();
  for (unsigned int j =0; j< streams.size(); j++) {
    auto stream = streams[j].stream;
    target_streams_[j].enabled_mode_ = Synthetic::MODE_OFF;
    device_->SetStreamCallback(stream, nullptr);
  }
  device_->Stop(Source::VIDEO_STREAMING);
}
api::StreamData s1s2Processor::GetStreamData(const Stream &stream) {
  Synthetic::Mode enable_mode = Synthetic::MODE_OFF;
  auto streams = getTargetStreams();
  for (auto it_s : streams) {
    if (it_s.stream == stream) {
      enable_mode = it_s.enabled_mode_;
      break;
    }
  }
  if (enable_mode == Synthetic::MODE_ON) {
    auto res = data2api(device_->GetStreamData(stream));
    if (res.img == nullptr ||
        res.img->timestamp == last_frame_id_cd ||
        res.frame.empty()) {
      return {};
    }
    last_frame_id_cd = res.img->timestamp;
    return res;
    // return data2api(device_->GetStreamData(stream));
  }
  LOG(ERROR) << "Failed to get device stream data of " << stream
               << ", unsupported or disabled";
  LOG(ERROR) << "Make sure you have enable " << stream;
  return {};
}

std::vector<api::StreamData> s1s2Processor::GetStreamDatas(
    const Stream &stream) {
  Synthetic::Mode enable_mode = Synthetic::MODE_OFF;
  auto streams = getTargetStreams();
  for (auto it_s : streams) {
    if (it_s.stream == stream) {
      enable_mode = it_s.enabled_mode_;
      break;
    }
  }
  if (enable_mode == Synthetic::MODE_ON) {
    std::vector<api::StreamData> datas;
    for (auto &&data : device_->GetStreamDatas(stream)) {
      datas.push_back(data2api(data));
    }
    return datas;
  }
}

MYNTEYE_END_NAMESPACE
