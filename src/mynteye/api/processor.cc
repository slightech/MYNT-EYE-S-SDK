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
#include "mynteye/api/processor.h"

#include <exception>
#include <utility>

#include "mynteye/logger.h"
#include "mynteye/util/strings.h"
#include "mynteye/util/times.h"
#include "mynteye/api/data_tools.h"

MYNTEYE_BEGIN_NAMESPACE

Processor::Processor(std::int32_t proc_period)
    : last_frame_id_cd(0),
      last_frame_id_cd_vice(0),
      is_enable_cd(false),
      proc_period_(std::move(proc_period)),
      activated_(false),
      input_ready_(false),
      idle_(true),
      dropped_count_(0),
      input_(nullptr),
      output_(nullptr),
      output_result_(nullptr),
      pre_callback_(nullptr),
      post_callback_(nullptr),
      callback_(nullptr),
      parent_(nullptr) {
  VLOG(2) << __func__;
}

Processor::~Processor() {
  VLOG(2) << __func__;
  Deactivate();
  input_ = nullptr;
  output_ = nullptr;
  output_result_ = nullptr;
  childs_.clear();
}

std::string Processor::Name() {
  return "Processor";
}

void Processor::AddChild(const std::shared_ptr<Processor> &child) {
  child->parent_ = shared_from_this();
  childs_.push_back(child);
}

void Processor::RemoveChild(const std::shared_ptr<Processor> &child) {
  childs_.remove(child);
}

std::list<std::shared_ptr<Processor>> Processor::GetChilds() {
  return childs_;
}

void Processor::SetPreProcessCallback(PreProcessCallback callback) {
  pre_callback_ = std::move(callback);
}

void Processor::SetPostProcessCallback(PostProcessCallback callback) {
  post_callback_ = std::move(callback);
}

void Processor::SetProcessCallback(ProcessCallback callback) {
  callback_ = std::move(callback);
}

void Processor::Activate(bool parents) {
  if (activated_)
    return;
  if (parents) {
    // Activate all parents
    auto parent = parent_;
    // Processor *parent = parent_;
    while (parent != nullptr) {
      parent->Activate();
      parent = parent->parent_;
    }
  }
  activated_ = true;
  thread_ = std::thread(&Processor::Run, this);
  // thread_.detach();
}

void Processor::Deactivate(bool childs) {
  if (!activated_)
    return;
  if (childs) {
    // Deactivate all childs
    iterate_processors_PtoC_after(GetChilds(),
        [](std::shared_ptr<Processor> proc) {
      proc->Deactivate();
    });
  }
  activated_ = false;
  {
    std::lock_guard<std::mutex> lk(mtx_input_ready_);
    input_ready_ = true;
  }
  cond_input_ready_.notify_all();
  thread_.join();
}

bool Processor::IsActivated() {
  return activated_;
}

bool Processor::IsIdle() {
  std::lock_guard<std::mutex> lk(mtx_state_);
  return idle_;
}

bool Processor::Process(std::shared_ptr<Object> in) {
  if (!activated_)
    return false;
  if (!idle_) {
    std::lock_guard<std::mutex> lk(mtx_state_);
    if (!idle_) {
      ++dropped_count_;
      return false;
    }
  }
  if (in && !in->DecValidity()) {
    LOG(WARNING) << Name() << " process with invalid input";
    return false;
  }
  {
    std::lock_guard<std::mutex> lk(mtx_input_ready_);
    if (ProcessInputConnection() == WITH_CLONE) {
      input_.reset(in->Clone());
    } else {
      input_ = in;
    }
    input_ready_ = true;
  }
  cond_input_ready_.notify_all();
  return true;
}

std::shared_ptr<Object> Processor::GetOutput() {
  std::lock_guard<std::mutex> lk(mtx_result_);
  return std::shared_ptr<Object>(std::move(output_result_));
}

std::uint64_t Processor::GetDroppedCount() {
  std::lock_guard<std::mutex> lk(mtx_state_);
  return dropped_count_;
}

std::shared_ptr<Processor> Processor::GetParent() {
  return parent_;
}

void Processor::Run() {
  VLOG(2) << Name() << " thread start";

  auto sleep = [this](const times::system_clock::time_point &time_beg) {
    if (proc_period_ > 0) {
      static times::system_clock::time_point time_prev = time_beg;
      auto &&time_elapsed_ms =
          times::count<times::milliseconds>(times::now() - time_prev);
      time_prev = time_beg;

      if (time_elapsed_ms < proc_period_) {
        VLOG(2) << Name() << " process cost "
                << times::count<times::milliseconds>(times::now() - time_beg)
                << " ms, sleep " << (proc_period_ - time_elapsed_ms) << " ms";
        std::this_thread::sleep_for(
            std::chrono::milliseconds(proc_period_ - time_elapsed_ms));
        return;
      }
    }
    VLOG(2) << Name() << " process cost "
            << times::count<times::milliseconds>(times::now() - time_beg)
            << " ms";
  };

  while (true) {
    std::unique_lock<std::mutex> lk(mtx_input_ready_);
    cond_input_ready_.wait(lk, [this] { return input_ready_; });

    auto &&time_beg = times::now();

    if (!activated_) {
      SetIdle(true);
      input_ready_ = false;
      break;
    }
    SetIdle(false);

    if (!output_) {
      output_.reset(OnCreateOutput());
    }

    if (pre_callback_) {
      pre_callback_(input_.get());
    }
    bool ok = false;
    try {
      if (callback_) {
        if (callback_(input_.get(), output_.get(), parent_)) {
          ok = true;
        } else {
          ok = OnProcess(input_.get(), output_.get(), parent_);
        }
      } else {
        ok = OnProcess(input_.get(), output_.get(), parent_);
      }
      // CV_Assert(false);
    } catch (const std::exception &e) {
      std::string msg(e.what());
      strings::rtrim(msg);
      LOG(ERROR) << Name() << " process error \"" << msg << "\"";
    }
    if (!ok) {
      VLOG(2) << Name() << " process failed";
      continue;
    }
    if (post_callback_) {
      post_callback_(output_.get());
    }
    {
      std::unique_lock<std::mutex> lk(mtx_result_);
      if (ProcessOutputConnection() == WITH_CLONE) {
        output_result_.reset(output_->Clone());
      } else {
        output_result_ = output_;
      }
    }

    if (!childs_.empty()) {
      for (auto child : childs_) {
        child->Process(output_);
      }
    }

    SetIdle(true);
    input_ready_ = false;

    sleep(time_beg);
  }
  VLOG(2) << Name() << " thread end";
}

Processor::process_type Processor::ProcessOutputConnection() {
  return WITH_CLONE;
}

Processor::process_type Processor::ProcessInputConnection() {
  return WITH_CLONE;
}

api::StreamData Processor::GetStreamData(const Stream &stream) {
  auto sum = getStreamsSum();
  auto &&out = GetOutput();
  Synthetic::Mode enable_mode = Synthetic::MODE_OFF;
  auto streams = getTargetStreams();
  for (auto it_s : streams) {
    if (it_s.stream == stream) {
      enable_mode = it_s.enabled_mode_;
      break;
    }
  }
  if (enable_mode == Synthetic::MODE_ON) {
    if (sum == 1) {
      if (out != nullptr) {
        auto output = Object::Cast<ObjMat>(out);
        if (output != nullptr) {
          if (!is_enable_cd) {
            if (output->data &&
                last_frame_id_cd == output->data->frame_id) {
              // cut the duplicate frame.
              return {};
            }
            last_frame_id_cd = output->data->frame_id;
          }
          return obj_data(output);
        }
        VLOG(2) << "Frame not ready now";
      }
    } else if (sum == 2) {
      static std::shared_ptr<ObjMat2> output = nullptr;
      if (out != nullptr) {
        output = Object::Cast<ObjMat2>(out);
      }
      auto streams = getTargetStreams();
      if (output != nullptr) {
        for (auto it : streams) {
          if (it.stream == stream) {
            if (it.stream == Stream::LEFT ||
                it.stream == Stream::LEFT_RECTIFIED) {
              if (!is_enable_cd) {
                if (output->first_data &&
                  last_frame_id_cd == output->first_data->frame_id) {
                  // cut the duplicate frame.
                  return {};
                }
                last_frame_id_cd = output->first_data->frame_id;
              }
              return obj_data_first(output);
            } else if (it.stream == Stream::RIGHT ||
                       it.stream == Stream::RIGHT_RECTIFIED) {
              // last_frame_id_cd = output->second_data->frame_id;
              if (!is_enable_cd) {
                if (output->second_data &&
                    last_frame_id_cd_vice == output->second_data->frame_id) {
                  return {};
                }
                last_frame_id_cd_vice = output->second_data->frame_id;
              }
              return obj_data_second(output);
            }
          }
        }
      }
      VLOG(2) << "Frame not ready now";
    } else {
      LOG(ERROR) << "error: invalid sum!";
    }
    return {};  // frame.empty() == true
  }
  LOG(ERROR) << "Failed to get stream data of " << stream
               << ", unsupported or disabled";
  return {};  // frame.empty() == true
}

std::vector<api::StreamData> Processor::GetStreamDatas(const Stream &stream) {
  Synthetic::Mode enable_mode = Synthetic::MODE_OFF;
  auto streams = getTargetStreams();
  for (auto it_s : streams) {
    if (it_s.stream == stream) {
      enable_mode = it_s.enabled_mode_;
      break;
    }
  }
  if (enable_mode == Synthetic::MODE_ON) {
    return {GetStreamData(stream)};
  } else {
    LOG(ERROR) << "Failed to get stream data of " << stream
               << ", unsupported or disabled";
  }
  return {};
}

void Processor::SetIdle(bool idle) {
  std::lock_guard<std::mutex> lk(mtx_state_);
  idle_ = idle;
}

MYNTEYE_END_NAMESPACE
