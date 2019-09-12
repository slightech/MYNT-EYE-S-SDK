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
#ifndef MYNTEYE_API_PROCESSOR_ROOT_CAMERA_PROCESSOR_H_
#define MYNTEYE_API_PROCESSOR_ROOT_CAMERA_PROCESSOR_H_
#pragma once

#include <string>
#include <memory>
#include <opencv2/core/core.hpp>

#include "mynteye/api/processor.h"
#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

class RootProcessor : public Processor {
 public:
  static const char NAME[];

  explicit RootProcessor(std::shared_ptr<Device> device,
      std::int32_t proc_period = 0);
  virtual ~RootProcessor();

  virtual std::string Name();

  virtual void StartVideoStreaming() = 0;
  virtual void StopVideoStreaming() = 0;
  virtual api::StreamData GetStreamData(const Stream &stream) = 0;
  virtual std::vector<api::StreamData> GetStreamDatas(const Stream &stream) = 0; // NOLINT
 protected:
  virtual Object *OnCreateOutput() = 0;
  virtual bool OnProcess(
      Object *const in, Object *const out,
      std::shared_ptr<Processor> const parent) = 0;
  std::shared_ptr<Device> device_;
};

class s1s2Processor : public RootProcessor {
 public:
  explicit s1s2Processor(std::shared_ptr<Device> device,
      std::int32_t proc_period = 0);
  virtual ~s1s2Processor();
  void StartVideoStreaming();
  void StopVideoStreaming();
  api::StreamData GetStreamData(const Stream &stream) override;
  std::vector<api::StreamData> GetStreamDatas(const Stream &stream) override; // NOLINT
 protected:
  // inline Processor::process_type ProcessOutputConnection() override {
  //   return Processor::WITHOUT_CLONE;
  // }
  // inline Processor::process_type ProcessInputConnection() override {
  //   return Processor::WITHOUT_CLONE;
  // }
  Object *OnCreateOutput() override;
  bool OnProcess(
      Object *const in, Object *const out,
      std::shared_ptr<Processor> const parent) override;
 private:
  void ProcessNativeStream(
    const Stream &stream, const api::StreamData &data);
  std::mutex mtx_left_right_ready_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_API_PROCESSOR_ROOT_CAMERA_PROCESSOR_H_
