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
#ifndef MYNTEYE_TOOLS_DATASET_H_  // NOLINT
#define MYNTEYE_TOOLS_DATASET_H_
#pragma once

#include <fstream>
#include <map>
#include <memory>
#include <string>

#include "mynteye/mynteye.h"
#include "mynteye/api/api.h"
#include "mynteye/device/callbacks.h"

MYNTEYE_BEGIN_NAMESPACE

namespace tools {

class Dataset {
 public:
  struct Writer {
    std::ofstream ofs;
    std::string outdir;
    std::string outfile;
  };

  using writer_t = std::shared_ptr<Writer>;

  explicit Dataset(std::string outdir);
  ~Dataset();

  void SaveStreamData(const Stream &stream, const device::StreamData &data);
  void SaveMotionData(const device::MotionData &data);

  void SaveStreamData(const Stream &stream, const api::StreamData &data);
  void SaveMotionData(const api::MotionData &data);

 private:
  writer_t GetStreamWriter(const Stream &stream);
  writer_t GetMotionWriter();

  std::string outdir_;

  std::map<Stream, writer_t> stream_writers_;
  writer_t motion_writer_;

  std::map<Stream, std::size_t> stream_counts_;
  std::size_t motion_count_;
  std::size_t accel_count_;
  std::size_t gyro_count_;
};

}  // namespace tools

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_TOOLS_DATASET_H_ NOLINT
