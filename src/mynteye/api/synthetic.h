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

#include "mynteye/api/api.h"
#include "mynteye/api/config.h"

MYNTEYE_BEGIN_NAMESPACE

class API;
class Plugin;
class Processor;

struct Object;

class Synthetic {
 public:
  using stream_callback_t = API::stream_callback_t;

  typedef enum Mode {
    MODE_NATIVE,     // Native stream
    MODE_SYNTHETIC,  // Synthetic stream
    MODE_LAST        // Unsupported
  } mode_t;

  explicit Synthetic(API *api, CalibrationModel calib_model);
  ~Synthetic();

  void NotifyImageParamsChanged(bool is_from_dev = false);

  bool Supports(const Stream &stream) const;
  mode_t SupportsMode(const Stream &stream) const;

  void EnableStreamData(const Stream &stream);
  void DisableStreamData(const Stream &stream);
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

 private:
  void InitCalibInfo();
  void InitStreamSupports();

  mode_t GetStreamEnabledMode(const Stream &stream) const;
  bool IsStreamEnabledNative(const Stream &stream) const;
  bool IsStreamEnabledSynthetic(const Stream &stream) const;

  void EnableStreamData(const Stream &stream, std::uint32_t depth);
  void DisableStreamData(const Stream &stream, std::uint32_t depth);

  void InitProcessors();

  template <class T>
  bool ActivateProcessor(bool tree = false);
  template <class T>
  bool DeactivateProcessor(bool tree = false);

  void ProcessNativeStream(const Stream &stream, const api::StreamData &data);

  bool OnRectifyProcess(
      Object *const in, Object *const out, Processor *const parent);
  bool OnDisparityProcess(
      Object *const in, Object *const out, Processor *const parent);
  bool OnDisparityNormalizedProcess(
      Object *const in, Object *const out, Processor *const parent);
  bool OnPointsProcess(
      Object *const in, Object *const out, Processor *const parent);
  bool OnDepthProcess(
      Object *const in, Object *const out, Processor *const parent);

  void OnRectifyPostProcess(Object *const out);
  void OnDisparityPostProcess(Object *const out);
  void OnDisparityNormalizedPostProcess(Object *const out);
  void OnPointsPostProcess(Object *const out);
  void OnDepthPostProcess(Object *const out);

  API *api_;

  std::map<Stream, mode_t> stream_supports_mode_;
  std::map<Stream, mode_t> stream_enabled_mode_;

  std::map<Stream, stream_callback_t> stream_callbacks_;

  std::shared_ptr<Processor> processor_;

  std::shared_ptr<Plugin> plugin_;

  CalibrationModel calib_model_;

  std::shared_ptr<IntrinsicsBase> intr_left_;
  std::shared_ptr<IntrinsicsBase> intr_right_;
  std::shared_ptr<Extrinsics> extr_;
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
