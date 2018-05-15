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
#ifndef MYNTEYE_API_H_  // NOLINT
#define MYNTEYE_API_H_
#pragma once

#include <opencv2/core/core.hpp>

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "mynteye/mynteye.h"
#include "mynteye/types.h"

MYNTEYE_BEGIN_NAMESPACE

class Device;
class Synthetic;

namespace api {

struct MYNTEYE_API StreamData {
  std::shared_ptr<ImgData> img;
  cv::Mat frame;
};

struct MYNTEYE_API MotionData {
  std::shared_ptr<ImuData> imu;
};

}  // namespace api

class MYNTEYE_API API {
 public:
  using stream_callback_t = std::function<void(const api::StreamData &data)>;
  using motion_callback_t = std::function<void(const api::MotionData &data)>;

  explicit API(std::shared_ptr<Device> device);
  /*virtual*/ ~API();

  /**
   * Create the API instance.
   * @return the API instance.
   * @note This will call device::select() to select a device.
   */
  static std::shared_ptr<API> Create();
  /**
   * Create the API instance.
   * @param device the selected device.
   * @return the API instance.
   */
  static std::shared_ptr<API> Create(std::shared_ptr<Device> device);
  /**
   * Create the API instance.
   * @param argc the arg count.
   * @param argv the arg values.
   * @return the API instance.
   * @note This will init glog with args and call device::select() to select a
   * device.
   */
  static std::shared_ptr<API> Create(int argc, char *argv[]);
  /**
   * Create the API instance.
   * @param argc the arg count.
   * @param argv the arg values.
   * @param device the selected device.
   * @return the API instance.
   * @note This will init glog with args.
   */
  static std::shared_ptr<API> Create(
      int argc, char *argv[], std::shared_ptr<Device> device);

  Model GetModel() const;

  bool Supports(const Stream &stream) const;
  bool Supports(const Capabilities &capability) const;
  bool Supports(const Option &option) const;
  bool Supports(const AddOns &addon) const;

  const std::vector<StreamRequest> &GetStreamRequests(
      const Capabilities &capability) const;
  void ConfigStreamRequest(
      const Capabilities &capability, const StreamRequest &request);

  std::string GetInfo(const Info &info) const;

  Intrinsics GetIntrinsics(const Stream &stream) const;
  Extrinsics GetExtrinsics(const Stream &from, const Stream &to) const;
  MotionIntrinsics GetMotionIntrinsics() const;
  Extrinsics GetMotionExtrinsics(const Stream &from) const;

  void LogOptionInfos() const;
  OptionInfo GetOptionInfo(const Option &option) const;

  std::int32_t GetOptionValue(const Option &option) const;
  void SetOptionValue(const Option &option, std::int32_t value);

  bool RunOptionAction(const Option &option) const;

  void SetStreamCallback(const Stream &stream, stream_callback_t callback);
  void SetMotionCallback(motion_callback_t callback);

  bool HasStreamCallback(const Stream &stream) const;
  bool HasMotionCallback() const;

  void Start(const Source &source);
  void Stop(const Source &source);

  void WaitForStreams();

  void EnableStreamData(const Stream &stream);
  void DisableStreamData(const Stream &stream);

  api::StreamData GetStreamData(const Stream &stream);
  std::vector<api::StreamData> GetStreamDatas(const Stream &stream);

  void EnableMotionDatas(
      std::size_t max_size = std::numeric_limits<std::size_t>::max());
  std::vector<api::MotionData> GetMotionDatas();

  void EnablePlugin(const std::string &path);

  std::shared_ptr<Device> device();

 private:
  std::shared_ptr<Device> device_;

  std::unique_ptr<Synthetic> synthetic_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_API_H_ NOLINT
