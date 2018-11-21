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
#ifndef MYNTEYE_DEVICE_ASYNC_CALLBACK_H_
#define MYNTEYE_DEVICE_ASYNC_CALLBACK_H_
#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

template <class Data>
class AsyncCallback {
 public:
  using callback_t = std::function<void(Data data)>;

  AsyncCallback(
      std::string name, callback_t callback, std::size_t max_data_size = 0);
  ~AsyncCallback();

  void PushData(Data data);

 private:
  void Run();

  std::string name_;

  callback_t callback_;

  std::mutex mtx_;
  std::condition_variable cv_;

  bool running_;
  std::thread thread_;

  std::uint32_t count_;
  std::vector<Data> datas_;
  std::size_t max_data_size_;
};

MYNTEYE_END_NAMESPACE

#include "mynteye/device/async_callback_impl.h"

#endif  // MYNTEYE_DEVICE_ASYNC_CALLBACK_H_
