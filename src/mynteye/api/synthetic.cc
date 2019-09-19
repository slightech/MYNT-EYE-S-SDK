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
#include "mynteye/api/synthetic.h"

#include <algorithm>
#include <functional>
#include <stdexcept>

#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye/logger.h"
#include "mynteye/api/object.h"
#include "mynteye/api/plugin.h"
#include "mynteye/api/processor.h"
#include "mynteye/api/processor/disparity_normalized_processor.h"
#include "mynteye/api/processor/disparity_processor.h"
#include "mynteye/api/processor/root_camera_processor.h"
#include "mynteye/api/processor/rectify_processor_ocv.h"
#ifdef WITH_CAM_MODELS
#include "mynteye/api/processor/depth_processor.h"
#include "mynteye/api/processor/points_processor.h"
#include "mynteye/api/processor/rectify_processor.h"
#endif
#include "mynteye/device/device.h"
#include "mynteye/api/data_tools.h"

#define RECTIFY_PROC_PERIOD 0
#define DISPARITY_PROC_PERIOD 0
#define DISPARITY_NORM_PROC_PERIOD 0
#define POINTS_PROC_PERIOD 0
#define DEPTH_PROC_PERIOD 0
#define ROOT_PROC_PERIOD 0

MYNTEYE_BEGIN_NAMESPACE

void Synthetic::InitCalibInfo() {
  if (calib_model_ == CalibrationModel::PINHOLE) {
    LOG(INFO) << "camera calib model: pinhole";
#ifdef WITH_CAM_MODELS
  } else if (calib_model_ == CalibrationModel::KANNALA_BRANDT) {
    LOG(INFO) << "camera calib model: kannala_brandt";
#endif
  }

  intr_left_ = api_->GetIntrinsicsBase(Stream::LEFT);
  intr_right_ = api_->GetIntrinsicsBase(Stream::RIGHT);
  extr_ = std::make_shared<Extrinsics>(
      api_->GetExtrinsics(Stream::RIGHT, Stream::LEFT));
}

Synthetic::Synthetic(API *api, CalibrationModel calib_model)
    : api_(api),
      plugin_(nullptr),
      calib_model_(calib_model),
      calib_default_tag_(false),
      stream_data_listener_(nullptr) {
  VLOG(2) << __func__;
  CHECK_NOTNULL(api_);
  InitCalibInfo();
  InitProcessors();
}

Synthetic::~Synthetic() {
  VLOG(2) << __func__;
  processors_.clear();
  if (processor_) {
    processor_->Deactivate(true);
    processor_ = nullptr;
  }
}

void Synthetic::SetStreamDataListener(stream_data_listener_t listener) {
  stream_data_listener_ = listener;
}

void Synthetic::NotifyImageParamsChanged() {
  if (!calib_default_tag_) {
    intr_left_ = api_->GetIntrinsicsBase(Stream::LEFT);
    intr_right_ = api_->GetIntrinsicsBase(Stream::RIGHT);
    extr_ =  std::make_shared<Extrinsics>(
        api_->GetExtrinsics(Stream::RIGHT, Stream::LEFT));
  }
  auto processor = getProcessorWithStream(Stream::LEFT_RECTIFIED);

  if (processor && calib_model_ ==  CalibrationModel::PINHOLE) {
    auto proc = static_cast<RectifyProcessorOCV*>(&(*processor));
    proc->ReloadImageParams(intr_left_, intr_right_, extr_);
#ifdef WITH_CAM_MODELS
  } else if (processor && calib_model_ == CalibrationModel::KANNALA_BRANDT) {
    auto proc = static_cast<RectifyProcessor*>(&(*processor));
    proc->ReloadImageParams(intr_left_, intr_right_, extr_);
#endif
  } else {
    LOG(ERROR) << "Unknow calib model type in device" << std::endl;
  }
}

bool Synthetic::ConfigDisparityFromFile(const std::string& config_file) {
  auto processor = getProcessorWithStream(Stream::DISPARITY);
  auto proc = static_cast<DisparityProcessor*>(&(*processor));
  return proc->ConfigFromFile(config_file);
}

const struct Synthetic::stream_control_t Synthetic::getControlDateWithStream(
    const Stream& stream) const {
  for (auto &&it : processors_) {
    for (auto it_s : it->getTargetStreams()) {
      if (it_s.stream == stream) {
        return it_s;
      }
    }
  }
  LOG(ERROR) << "ERROR: no suited processor for stream "<< stream;
  return {};
}

std::shared_ptr<Processor> Synthetic::getProcessorWithStream(
    const Stream& stream) {
  for (auto &&it : processors_) {
    for (auto it_s : it->getTargetStreams()) {
      if (it_s.stream == stream) {
        return it;
      }
    }
  }
  LOG(ERROR) << "ERROR: no suited processor for stream "<< stream;
}

void Synthetic::setControlDateCallbackWithStream(
    const struct stream_control_t& ctr_data) {
  for (auto &&it : processors_) {
    int i = 0;
    for (auto it_s : it->getTargetStreams()) {
      if (it_s.stream == ctr_data.stream) {
        it->target_streams_[i].stream_callback = ctr_data.stream_callback;
        return;
      }
      i++;
    }
  }
  LOG(ERROR) << "ERROR: no suited processor for stream "<< ctr_data.stream;
}

bool Synthetic::checkControlDateWithStream(const Stream& stream) const {
  for (auto &&it : processors_) {
    for (auto it_s : it->getTargetStreams()) {
      if (it_s.stream == stream) {
        return true;
      }
    }
  }
  return false;
}

bool Synthetic::Supports(const Stream &stream) const {
  return checkControlDateWithStream(stream);
}

void Synthetic::setDuplicate(bool isEnable) {
  for (auto it : processors_) {
    it->setDupEnable(isEnable);
  }
}

void Synthetic::EnableStreamData(
    const Stream &stream, stream_switch_callback_t callback,
    bool try_tag) {
  // Activate processors of synthetic stream
  auto processor = getProcessorWithStream(stream);
  iterate_processor_CtoP_before(processor,
      [callback, try_tag](std::shared_ptr<Processor> proce){
        if (proce->Name() == "RootProcessor") {
          return;
        }
        auto streams = proce->getTargetStreams();
        int act_tag = 0;
        for (unsigned int i = 0; i < proce->getStreamsSum() ; i++) {
          if (proce->target_streams_[i].enabled_mode_ == MODE_OFF) {
            callback(proce->target_streams_[i].stream);
            if (!try_tag) {
              act_tag++;
              proce->target_streams_[i].enabled_mode_ = MODE_ON;
            }
          }
        }
        if (act_tag > 0 && !proce->IsActivated()) {
          // std::cout << proce->Name() << " Active now" << std::endl;
          proce->Activate();
        }
      });
}
void Synthetic::DisableStreamData(
    const Stream &stream, stream_switch_callback_t callback,
    bool try_tag) {
  auto processor = getProcessorWithStream(stream);
  iterate_processor_PtoC_before(processor,
      [callback, try_tag](std::shared_ptr<Processor> proce){
        if (proce->Name() == "RootProcessor") {
          return;
        }
        auto streams = proce->getTargetStreams();
        int act_tag = 0;
        for (unsigned int i = 0; i < proce->getStreamsSum() ; i++) {
          if (proce->target_streams_[i].enabled_mode_ == MODE_ON) {
            callback(proce->target_streams_[i].stream);
            if (!try_tag) {
              act_tag++;
              proce->target_streams_[i].enabled_mode_ = MODE_OFF;
            }
          }
        }
        if (act_tag > 0 && proce->IsActivated()) {
          // std::cout << proce->Name() << "Deactive now" << std::endl;
          proce->Deactivate();
        }
      });
}

void Synthetic::EnableStreamData(const Stream &stream) {
  EnableStreamData(stream, [](const Stream &stream){
        // std::cout << stream << "enabled in callback" << std::endl;
        MYNTEYE_UNUSED(stream);
      }, false);
}

void Synthetic::DisableStreamData(const Stream &stream) {
  DisableStreamData(stream, [](const Stream &stream){
        // std::cout << stream << "disabled in callback" << std::endl;
        MYNTEYE_UNUSED(stream);
      }, false);
}

bool Synthetic::IsStreamDataEnabled(const Stream &stream) const {
  if (checkControlDateWithStream(stream)) {
    auto data = getControlDateWithStream(stream);
    return data.enabled_mode_ == MODE_ON;
  }
  return false;
}

void Synthetic::SetStreamCallback(
    const Stream &stream, stream_callback_t callback) {
  stream_control_t data;
  data.stream = stream;
  if (callback == nullptr) {
    data.stream_callback = nullptr;
  } else {
    data.stream_callback = callback;
  }
  setControlDateCallbackWithStream(data);
}

bool Synthetic::HasStreamCallback(const Stream &stream) const {
  if (checkControlDateWithStream(stream)) {
    auto data = getControlDateWithStream(stream);
    if (data.stream_callback != nullptr) {
      return true;
    }
  }
  return false;
}

void Synthetic::StartVideoStreaming() {
  processor_->StartVideoStreaming();
}

void Synthetic::StopVideoStreaming() {
  processor_->StopVideoStreaming();
}

void Synthetic::WaitForStreams() {
  api_->device()->WaitForStreams();
}

api::StreamData Synthetic::GetStreamData(const Stream &stream) {
  return getProcessorWithStream(stream)->GetStreamData(stream);
}

std::vector<api::StreamData> Synthetic::GetStreamDatas(const Stream &stream) {
  return getProcessorWithStream(stream)->GetStreamDatas(stream);
}

void Synthetic::SetPlugin(std::shared_ptr<Plugin> plugin) {
  plugin_ = plugin;
}

bool Synthetic::HasPlugin() const {
  return plugin_ != nullptr;
}

Synthetic::mode_t Synthetic::GetStreamEnabledMode(const Stream &stream) const {
  if (checkControlDateWithStream(stream)) {
    auto data = getControlDateWithStream(stream);
    return data.enabled_mode_;
  }
  return MODE_OFF;
}

void Synthetic::InitProcessors() {
  std::shared_ptr<Processor> rectify_processor = nullptr;
  std::shared_ptr<Processor> points_processor = nullptr;
  std::shared_ptr<Processor> depth_processor = nullptr;
  std::shared_ptr<Processor> disparity_processor = nullptr;

  auto &&disparitynormalized_processor =
      std::make_shared<DisparityNormalizedProcessor>(
          DISPARITY_NORM_PROC_PERIOD);

  auto root_processor =
      std::make_shared<s1s2Processor>(api_->device(), ROOT_PROC_PERIOD);

  if (calib_model_ == CalibrationModel::PINHOLE) {
    // PINHOLE
    auto &&rectify_processor_ocv =
        std::make_shared<RectifyProcessorOCV>(intr_left_, intr_right_, extr_,
                                              RECTIFY_PROC_PERIOD);
    rectify_processor = rectify_processor_ocv;
    points_processor = std::make_shared<PointsProcessor>(
        rectify_processor_ocv -> getCameraROSMsgInfoPair(),
        POINTS_PROC_PERIOD);
    auto disparity_processor_imp =
      std::make_shared<DisparityProcessor>(DisparityComputingMethod::BM,
                                           nullptr,
                                           DISPARITY_PROC_PERIOD);
    depth_processor = std::make_shared<DepthProcessor>(
        rectify_processor_ocv -> getCameraROSMsgInfoPair(),
        disparity_processor_imp->GetMinDisparity(),
        disparity_processor_imp->GetMaxDisparity(),
        DEPTH_PROC_PERIOD);
    disparity_processor = disparity_processor_imp;
#ifdef WITH_CAM_MODELS
  } else if (calib_model_ == CalibrationModel::KANNALA_BRANDT) {
    // KANNALA_BRANDT
    auto rectify_processor_imp =
        std::make_shared<RectifyProcessor>(intr_left_, intr_right_, extr_,
                                           RECTIFY_PROC_PERIOD);
    rectify_processor = rectify_processor_imp;
    points_processor = std::make_shared<PointsProcessor>(
        rectify_processor_imp -> getCameraROSMsgInfoPair(),
        POINTS_PROC_PERIOD);
    auto disparity_processor_imp =
      std::make_shared<DisparityProcessor>(DisparityComputingMethod::BM,
          rectify_processor_imp -> getCameraROSMsgInfoPair(),
          DISPARITY_PROC_PERIOD);
    depth_processor = std::make_shared<DepthProcessor>(
        rectify_processor_imp -> getCameraROSMsgInfoPair(),
        disparity_processor_imp->GetMinDisparity(),
        disparity_processor_imp->GetMaxDisparity(),
        DEPTH_PROC_PERIOD);
    disparity_processor = disparity_processor_imp;
#endif
  } else {
    // UNKNOW
    LOG(ERROR) << "Unknow calib model type in device: "
               << calib_model_;
    return;
  }

  root_processor->AddChild(rectify_processor);
  rectify_processor->AddChild(disparity_processor);
  disparity_processor->AddChild(disparitynormalized_processor);
  disparity_processor->AddChild(depth_processor);
  depth_processor->AddChild(points_processor);

  root_processor->addTargetStreams(
      {Stream::LEFT, Mode::MODE_OFF, nullptr});
  root_processor->addTargetStreams(
      {Stream::RIGHT, Mode::MODE_OFF, nullptr});
  rectify_processor->addTargetStreams(
      {Stream::LEFT_RECTIFIED, Mode::MODE_OFF, nullptr});
  rectify_processor->addTargetStreams(
      {Stream::RIGHT_RECTIFIED, Mode::MODE_OFF, nullptr});
  disparity_processor->addTargetStreams(
      {Stream::DISPARITY, Mode::MODE_OFF, nullptr});
  disparitynormalized_processor->addTargetStreams(
      {Stream::DISPARITY_NORMALIZED, Mode::MODE_OFF, nullptr});
  points_processor->addTargetStreams(
      {Stream::POINTS, Mode::MODE_OFF, nullptr});
  depth_processor->addTargetStreams(
      {Stream::DEPTH, Mode::MODE_OFF, nullptr});

  processors_.push_back(root_processor);
  processors_.push_back(rectify_processor);
  processors_.push_back(disparity_processor);
  processors_.push_back(disparitynormalized_processor);
  processors_.push_back(points_processor);
  processors_.push_back(depth_processor);
  using namespace std::placeholders;  // NOLINT
  root_processor->SetProcessCallback(
      std::bind(&Synthetic::OnDeviceProcess, this, _1, _2, _3));
  rectify_processor->SetProcessCallback(
      std::bind(&Synthetic::OnRectifyProcess, this, _1, _2, _3));
  disparity_processor->SetProcessCallback(
      std::bind(&Synthetic::OnDisparityProcess, this, _1, _2, _3));
  disparitynormalized_processor->SetProcessCallback(
      std::bind(&Synthetic::OnDisparityNormalizedProcess, this, _1, _2, _3));
  points_processor->SetProcessCallback(
      std::bind(&Synthetic::OnPointsProcess, this, _1, _2, _3));
  depth_processor->SetProcessCallback(
      std::bind(&Synthetic::OnDepthProcess, this, _1, _2, _3));

  root_processor->SetPostProcessCallback(
      std::bind(&Synthetic::OnDevicePostProcess, this, _1));
  rectify_processor->SetPostProcessCallback(
      std::bind(&Synthetic::OnRectifyPostProcess, this, _1));
  disparity_processor->SetPostProcessCallback(
      std::bind(&Synthetic::OnDisparityPostProcess, this, _1));
  disparitynormalized_processor->SetPostProcessCallback(
      std::bind(&Synthetic::OnDisparityNormalizedPostProcess, this, _1));
  points_processor->SetPostProcessCallback(
      std::bind(&Synthetic::OnPointsPostProcess, this, _1));
  depth_processor->SetPostProcessCallback(
      std::bind(&Synthetic::OnDepthPostProcess, this, _1));

  processor_ = root_processor;
}

bool Synthetic::OnDeviceProcess(
    Object *const in, Object *const out,
    std::shared_ptr<Processor> const parent) {
  MYNTEYE_UNUSED(parent)
  return GetStreamEnabledMode(Stream::LEFT) != MODE_ON
      || GetStreamEnabledMode(Stream::RIGHT) != MODE_ON;
}

bool Synthetic::OnRectifyProcess(
    Object *const in, Object *const out,
    std::shared_ptr<Processor> const parent) {
  MYNTEYE_UNUSED(parent)
  if (plugin_ && plugin_->OnRectifyProcess(in, out)) {
    return true;
  }
  return GetStreamEnabledMode(Stream::LEFT_RECTIFIED) != MODE_ON
      && GetStreamEnabledMode(Stream::RIGHT_RECTIFIED) != MODE_ON;
}

bool Synthetic::OnDisparityProcess(
    Object *const in, Object *const out,
    std::shared_ptr<Processor> const parent) {
  MYNTEYE_UNUSED(parent)
  if (plugin_ && plugin_->OnDisparityProcess(in, out)) {
    return true;
  }
  return GetStreamEnabledMode(Stream::DISPARITY) != MODE_ON;
}

bool Synthetic::OnDisparityNormalizedProcess(
    Object *const in, Object *const out,
    std::shared_ptr<Processor> const parent) {
  MYNTEYE_UNUSED(parent)
  if (plugin_ && plugin_->OnDisparityNormalizedProcess(in, out)) {
    return true;
  }
  return GetStreamEnabledMode(Stream::DISPARITY_NORMALIZED) != MODE_ON;
}

bool Synthetic::OnPointsProcess(
    Object *const in, Object *const out,
    std::shared_ptr<Processor> const parent) {
  MYNTEYE_UNUSED(parent)
  if (plugin_ && plugin_->OnPointsProcess(in, out)) {
    return true;
  }
  return GetStreamEnabledMode(Stream::POINTS) != MODE_ON;
}

bool Synthetic::OnDepthProcess(
    Object *const in, Object *const out,
    std::shared_ptr<Processor> const parent) {
  MYNTEYE_UNUSED(parent)
  if (plugin_ && plugin_->OnDepthProcess(in, out)) {
    return true;
  }
  return GetStreamEnabledMode(Stream::DEPTH) != MODE_ON;
}

void Synthetic::OnDevicePostProcess(Object *const out) {
  const ObjMat2 *output = Object::Cast<ObjMat2>(out);
  NotifyStreamData(Stream::LEFT, obj_data_first(output));
  NotifyStreamData(Stream::RIGHT, obj_data_second(output));
  if (HasStreamCallback(Stream::LEFT)) {
    auto data = getControlDateWithStream(Stream::LEFT);
    data.stream_callback(obj_data_first(output));
  }
  if (HasStreamCallback(Stream::RIGHT)) {
    auto data = getControlDateWithStream(Stream::RIGHT);
    if (data.stream_callback)
      data.stream_callback(obj_data_second(output));
  }
}

void Synthetic::OnRectifyPostProcess(Object *const out) {
  const ObjMat2 *output = Object::Cast<ObjMat2>(out);
  NotifyStreamData(Stream::LEFT_RECTIFIED, obj_data_first(output));
  NotifyStreamData(Stream::RIGHT_RECTIFIED, obj_data_second(output));
  if (HasStreamCallback(Stream::LEFT_RECTIFIED)) {
    auto data = getControlDateWithStream(Stream::LEFT_RECTIFIED);
    data.stream_callback(obj_data_first(output));
  }
  if (HasStreamCallback(Stream::RIGHT_RECTIFIED)) {
    auto data = getControlDateWithStream(Stream::RIGHT_RECTIFIED);
    data.stream_callback(obj_data_second(output));
  }
}

void Synthetic::OnDisparityPostProcess(Object *const out) {
  const ObjMat *output = Object::Cast<ObjMat>(out);
  NotifyStreamData(Stream::DISPARITY, obj_data(output));
  if (HasStreamCallback(Stream::DISPARITY)) {
    auto data = getControlDateWithStream(Stream::DISPARITY);
    data.stream_callback(obj_data(output));
  }
}

void Synthetic::OnDisparityNormalizedPostProcess(Object *const out) {
  const ObjMat *output = Object::Cast<ObjMat>(out);
  NotifyStreamData(Stream::DISPARITY_NORMALIZED, obj_data(output));
  if (HasStreamCallback(Stream::DISPARITY_NORMALIZED)) {
    auto data = getControlDateWithStream(Stream::DISPARITY_NORMALIZED);
    data.stream_callback(obj_data(output));
  }
}

void Synthetic::OnPointsPostProcess(Object *const out) {
  const ObjMat *output = Object::Cast<ObjMat>(out);
  NotifyStreamData(Stream::POINTS, obj_data(output));
  if (HasStreamCallback(Stream::POINTS)) {
    auto data = getControlDateWithStream(Stream::POINTS);
    data.stream_callback(obj_data(output));
  }
}

void Synthetic::OnDepthPostProcess(Object *const out) {
  const ObjMat *output = Object::Cast<ObjMat>(out);
  NotifyStreamData(Stream::DEPTH, obj_data(output));
  if (HasStreamCallback(Stream::DEPTH)) {
    auto data = getControlDateWithStream(Stream::DEPTH);
    data.stream_callback(obj_data(output));
  }
}

void Synthetic::SetDisparityComputingMethodType(
    const DisparityComputingMethod &MethodType) {
  if (checkControlDateWithStream(Stream::LEFT_RECTIFIED)) {
    auto processor = find_processor<DisparityProcessor>(processor_);
    if (processor)
      processor->SetDisparityComputingMethodType(MethodType);
    return;
  }
  LOG(ERROR) << "ERROR: no suited processor for disparity computing.";
}

bool Synthetic::SetRectifyAlpha(const double &alpha) {
  if (checkControlDateWithStream(Stream::LEFT_RECTIFIED)) {
#ifdef WITH_CAM_MODELS
    auto processor = find_processor<RectifyProcessor>(processor_);
    if (processor)
      processor->SetRectifyAlpha(alpha);
    return true;
#endif
  }
  LOG(ERROR) << "ERROR: no suited processor for rectify.";
  return false;
}

void Synthetic::NotifyStreamData(
    const Stream &stream, const api::StreamData &data) {
  if (stream_data_listener_) {
    stream_data_listener_(stream, data);
  }
}

std::shared_ptr<struct CameraROSMsgInfoPair>
    Synthetic::GetCameraROSMsgInfoPair() {
  if (calib_model_ == CalibrationModel::PINHOLE) {
    auto processor = getProcessorWithStream(Stream::LEFT_RECTIFIED);
    auto proc = static_cast<RectifyProcessorOCV*>(&(*processor));
    return proc->getCameraROSMsgInfoPair();
#ifdef WITH_CAM_MODELS
  } else if (calib_model_ == CalibrationModel::KANNALA_BRANDT) {
    auto processor = getProcessorWithStream(Stream::LEFT_RECTIFIED);
    auto proc = static_cast<RectifyProcessor*>(&(*processor));
    return proc->getCameraROSMsgInfoPair();
#endif
  }
  return nullptr;
}

MYNTEYE_END_NAMESPACE
