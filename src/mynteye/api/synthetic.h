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
#ifndef MYNTEYE_API_SYNTHETIC_H_
#define MYNTEYE_API_SYNTHETIC_H_
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "mynteye/api/api.h"

MYNTEYE_BEGIN_NAMESPACE

class API;
class Plugin;
class Processor;
class RootProcessor;

struct Object;

class Synthetic {
 public:
  using stream_callback_t = API::stream_callback_t;
  using stream_data_listener_t =
      std::function<void(const Stream &stream, const api::StreamData &data)>;
  using stream_switch_callback_t = API::stream_switch_callback_t;

  typedef enum Mode {
    MODE_ON,  // On
    MODE_OFF  // Off
  } mode_t;

  struct stream_control_t {
    Stream stream;
    mode_t enabled_mode_;
    stream_callback_t stream_callback;
  };

  explicit Synthetic(API *api, CalibrationModel calib_model);
  ~Synthetic();

  void SetStreamDataListener(stream_data_listener_t listener);

  void NotifyImageParamsChanged();

  bool Supports(const Stream &stream) const;

  void EnableStreamData(const Stream &stream);
  void DisableStreamData(const Stream &stream);

  void EnableStreamData(
      const Stream &stream, stream_switch_callback_t callback, bool try_tag);
  void DisableStreamData(
      const Stream &stream, stream_switch_callback_t callback, bool try_tag);
  bool IsStreamDataEnabled(const Stream &stream) const;

  void SetStreamCallback(const Stream &stream, stream_callback_t callback);
  bool HasStreamCallback(const Stream &stream) const;

  void StartVideoStreaming();
  void StopVideoStreaming();

  void WaitForStreams();

  api::StreamData GetStreamData(const Stream &stream);
  std::vector<api::StreamData> GetStreamDatas(const Stream &stream);

  void SetPlugin(std::shared_ptr<Plugin> plugin);
  bool HasPlugin() const;

  void setDuplicate(bool isEnable);

  const struct stream_control_t getControlDateWithStream(
      const Stream& stream) const;
  void setControlDateCallbackWithStream(
      const struct stream_control_t& ctr_data);
  bool checkControlDateWithStream(const Stream& stream) const;
  std::shared_ptr<Processor> getProcessorWithStream(const Stream& stream);
  void SetDisparityComputingMethodType(
      const DisparityComputingMethod &MethoType);
  std::shared_ptr<struct CameraROSMsgInfoPair> GetCameraROSMsgInfoPair();
  bool ConfigDisparityFromFile(const std::string& config_file);
  bool SetRectifyAlpha(const double &alpha);

 private:
  void InitCalibInfo();

  mode_t GetStreamEnabledMode(const Stream &stream) const;

  void EnableStreamData(const Stream &stream, std::uint32_t depth);
  void DisableStreamData(const Stream &stream, std::uint32_t depth);

  void InitProcessors();

  template <class T>
  bool ActivateProcessor(bool tree = false);
  template <class T>
  bool DeactivateProcessor(bool tree = false);

  bool OnDeviceProcess(
      Object *const in, Object *const out,
      std::shared_ptr<Processor> const parent);
  bool OnRectifyProcess(
      Object *const in, Object *const out,
      std::shared_ptr<Processor> const parent);
  bool OnDisparityProcess(
      Object *const in, Object *const out,
      std::shared_ptr<Processor> const parent);
  bool OnDisparityNormalizedProcess(
      Object *const in, Object *const out,
      std::shared_ptr<Processor> const parent);
  bool OnPointsProcess(
      Object *const in, Object *const out,
      std::shared_ptr<Processor> const parent);
  bool OnDepthProcess(
      Object *const in, Object *const out,
      std::shared_ptr<Processor> const parent);

  void OnDevicePostProcess(Object *const out);
  void OnRectifyPostProcess(Object *const out);
  void OnDisparityPostProcess(Object *const out);
  void OnDisparityNormalizedPostProcess(Object *const out);
  void OnPointsPostProcess(Object *const out);
  void OnDepthPostProcess(Object *const out);

  void NotifyStreamData(const Stream &stream, const api::StreamData &data);

  API *api_;

  std::shared_ptr<RootProcessor> processor_;
  std::vector<std::shared_ptr<Processor>> processors_;
  std::shared_ptr<Plugin> plugin_;

  CalibrationModel calib_model_;

  std::shared_ptr<IntrinsicsBase> intr_left_;
  std::shared_ptr<IntrinsicsBase> intr_right_;
  std::shared_ptr<Extrinsics> extr_;
  bool calib_default_tag_;

  stream_data_listener_t stream_data_listener_;
};

class SyntheticProcessorPart {
 protected:
  inline std::vector<Synthetic::stream_control_t> getTargetStreams() {
    return target_streams_;
  }
  inline void addTargetStreams(const Synthetic::stream_control_t& strm) {
    target_streams_.push_back(strm);
  }

  std::vector<Synthetic::stream_control_t> target_streams_;

  inline unsigned int getStreamsSum() {return target_streams_.size();}
  friend Synthetic;
};

template <class T, class P>
std::shared_ptr<T> find_processor(const P &processor) {
  return find_processor<T>(processor, T::NAME);
}

template <class T, class P>
std::shared_ptr<T> find_processor(const P &processor, const std::string &name) {
  if (processor->Name() == name) {
    return std::dynamic_pointer_cast<T>(processor);
  }
  auto &&childs = processor->GetChilds();
  return find_processor<T>(std::begin(childs), std::end(childs), name);
}

template <class T, class InputIt>
std::shared_ptr<T> find_processor(
    InputIt first, InputIt last, const std::string &name) {
  if (first == last)
    return nullptr;
  for (auto it = first; it != last; ++it) {
    if ((*it)->Name() == name) {
      return std::dynamic_pointer_cast<T>(*it);
    }
  }
  for (auto it = first; it != last; ++it) {
    auto &&childs = (*it)->GetChilds();
    if (childs.empty())
      continue;
    auto &&result =
        find_processor<T>(std::begin(childs), std::end(childs), name);
    if (result == nullptr)
      continue;
    return result;
  }
  return nullptr;
}

template <class T>
bool Synthetic::ActivateProcessor(bool parents) {
  auto &&processor = find_processor<T>(processor_);
  if (processor == nullptr)
    return false;
  processor->Activate(parents);
  return true;
}

template <class T>
bool Synthetic::DeactivateProcessor(bool childs) {
  auto &&processor = find_processor<T>(processor_);
  if (processor == nullptr)
    return false;
  processor->Deactivate(childs);
  return true;
}

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_API_SYNTHETIC_H_

