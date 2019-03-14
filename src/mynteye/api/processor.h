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
#ifndef MYNTEYE_API_PROCESSOR_H_
#define MYNTEYE_API_PROCESSOR_H_
#pragma once

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "mynteye/api/synthetic.h"

#include "mynteye/mynteye.h"
#include "mynteye/api/object.h"

MYNTEYE_BEGIN_NAMESPACE

class Processor :
    public std::enable_shared_from_this<Processor>,
    public SyntheticProcessorPart {
 public:
  using PreProcessCallback = std::function<void(Object *const)>;
  using PostProcessCallback = std::function<void(Object *const)>;
  using ProcessCallback = std::function<bool(
      Object *const in, Object *const out,
      std::shared_ptr<Processor> const parent)>;

  explicit Processor(std::int32_t proc_period = 0);
  virtual ~Processor();

  virtual std::string Name();

  void AddChild(const std::shared_ptr<Processor> &child);

  void RemoveChild(const std::shared_ptr<Processor> &child);

  std::shared_ptr<Processor> GetParent();
  std::list<std::shared_ptr<Processor>> GetChilds();

  void SetPreProcessCallback(PreProcessCallback callback);
  void SetPostProcessCallback(PostProcessCallback callback);
  void SetProcessCallback(ProcessCallback callback);

  void Activate(bool parents = false);
  void Deactivate(bool childs = false);
  bool IsActivated();

  bool IsIdle();

  /** Returns dropped or not. */
  bool Process(std::shared_ptr<Object> in);

  virtual api::StreamData GetStreamData(const Stream &stream);

  virtual std::vector<api::StreamData> GetStreamDatas(const Stream &stream);

  /**
   * Returns the last output.
   * @note Returns null if not output now.
   */
  std::shared_ptr<Object> GetOutput();

  std::uint64_t GetDroppedCount();
  inline void setDupEnable(bool isEnable) {
    is_enable_cd = isEnable;
  }

 protected:
  virtual Object *OnCreateOutput() = 0;
  virtual bool OnProcess(
      Object *const in, Object *const out,
      std::shared_ptr<Processor> const parent) = 0;
  enum process_type{
    WITH_CLONE,
    WITHOUT_CLONE
  };

  virtual process_type ProcessOutputConnection();
  virtual process_type ProcessInputConnection();
  std::uint16_t last_frame_id_cd;
  std::uint16_t last_frame_id_cd_vice;
  bool is_enable_cd;
 private:
  /** Run in standalone thread. */
  void Run();

  void SetIdle(bool idle);

  std::int32_t proc_period_;

  bool activated_;

  bool input_ready_;
  std::mutex mtx_input_ready_;
  std::condition_variable cond_input_ready_;

  bool idle_;
  std::uint64_t dropped_count_;
  std::mutex mtx_state_;

  std::shared_ptr<Object> input_;
  std::shared_ptr<Object> output_;

  std::shared_ptr<Object> output_result_;
  std::mutex mtx_result_;

  PreProcessCallback pre_callback_;
  PostProcessCallback post_callback_;
  ProcessCallback callback_;

  // Processor *parent_;
  std::shared_ptr<Processor> parent_;
  std::list<std::shared_ptr<Processor>> childs_;

  std::thread thread_;
};

template <typename T>
void iterate_processors_PtoC_after(
    const T &processors, std::function<void(std::shared_ptr<Processor>)> fn) {
  for (auto &&proc : processors) {
    fn(proc);
    iterate_processors_PtoC_after(proc->GetChilds(), fn);
  }
}
template <typename T>
void iterate_processors_PtoC_before(
    const T &processors, std::function<void(std::shared_ptr<Processor>)> fn) {
  for (auto &&proc : processors) {
    iterate_processors_PtoC_before(proc->GetChilds(), fn);
    fn(proc);
  }
}

template <typename T>
void iterate_processor_PtoC_after(
    T processor, std::function<void(std::shared_ptr<Processor>)> fn) {
  fn(processor);
  auto chids = processor->GetChilds();
  for (auto it : chids) {
    iterate_processor_PtoC_after(it, fn);
  }
}
template <typename T>
void iterate_processor_PtoC_before(
    T processor, std::function<void(std::shared_ptr<Processor>)> fn) {
  auto chids = processor->GetChilds();
  for (auto it : chids) {
    iterate_processor_PtoC_before(it, fn);
  }
  fn(processor);
}
template <typename T>
void iterate_processor_CtoP_before(
    T processor, std::function<void(std::shared_ptr<Processor>)> fn) {
  if (processor->GetParent() != nullptr)
    iterate_processor_CtoP_before(processor->GetParent(), fn);
  fn(processor);
}
template <typename T>
void iterate_processor_CtoP_after(
    T processor, std::function<void(std::shared_ptr<Processor>)> fn) {
  fn(processor);
  if (processor->GetParent() != nullptr)
    iterate_processor_CtoP_after(processor->GetParent(), fn);
}

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_API_PROCESSOR_H_
