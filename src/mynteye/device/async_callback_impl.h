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
#ifndef MYNTEYE_DEVICE_ASYNC_CALLBACK_IMPL_H_
#define MYNTEYE_DEVICE_ASYNC_CALLBACK_IMPL_H_
#pragma once

#include <string>
#include <utility>

#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

template <class Data>
AsyncCallback<Data>::AsyncCallback(
    std::string name, callback_t callback, std::size_t max_data_size)
    : name_(std::move(name)),
      callback_(std::move(callback)),
      count_(0),
      max_data_size_(max_data_size) {
  VLOG(2) << __func__;
  running_ = true;
  thread_ = std::thread(&AsyncCallback<Data>::Run, this);
}

template <class Data>
AsyncCallback<Data>::~AsyncCallback() {
  VLOG(2) << __func__;
  {
    std::lock_guard<std::mutex> _(mtx_);
    running_ = false;
    ++count_;
  }
  cv_.notify_one();
  if (thread_.joinable()) {
    thread_.join();
  }
}

template <class Data>
void AsyncCallback<Data>::PushData(Data data) {
  std::lock_guard<std::mutex> _(mtx_);
  if (max_data_size_ <= 0) {
    datas_.clear();
  } else if (max_data_size_ == datas_.size()) {  // >= 1
    datas_.erase(datas_.begin());
  }
  datas_.push_back(data);
  ++count_;
  cv_.notify_one();
}

template <class Data>
void AsyncCallback<Data>::Run() {
  VLOG(2) << "AsyncCallback(" << name_ << ") thread start";
  while (true) {
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [this] { return count_ > 0; });

    if (!running_)
      break;

    if (callback_) {
      for (auto &&data : datas_) {
        callback_(data);
      }
    }

    if (VLOG_IS_ON(2) && count_ > datas_.size()) {
      VLOG(2) << "AsyncCallback(" << name_ << ") dropped "
              << (count_ - datas_.size());
    }
    count_ = 0;
    datas_.clear();
  }
  VLOG(2) << "AsyncCallback(" << name_ << ") thread end";
}

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_ASYNC_CALLBACK_IMPL_H_
