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
#ifndef MYNTEYE_API_API_H_
#define MYNTEYE_API_API_H_
#pragma once

#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include "mynteye/mynteye.h"
#include "mynteye/types.h"

MYNTEYE_BEGIN_NAMESPACE

struct DeviceInfo;

class Correspondence;
class Device;
class Synthetic;

namespace device {

class Frame;

}  // namespace device

namespace api {

/**
 * @ingroup datatypes
 * API stream data.
 */
struct MYNTEYE_API StreamData {
  /** ImgData. */
  std::shared_ptr<ImgData> img;
  /** Frame. */
  cv::Mat frame;
  /** Raw frame. */
  std::shared_ptr<device::Frame> frame_raw;
  /** Frame ID. */
  std::uint16_t frame_id;

  bool operator==(const StreamData &other) const {
    if (img && other.img) {
      return img->frame_id == other.img->frame_id &&
             img->timestamp == other.img->timestamp;
    }
    return false;
  }
};

/**
 * @ingroup datatypes
 * API motion data.
 */
struct MYNTEYE_API MotionData {
  /** ImuData. */
  std::shared_ptr<ImuData> imu;

  bool operator==(const MotionData &other) const {
    if (imu && other.imu) {
      return imu->timestamp == other.imu->timestamp;
    }
    return false;
  }
};

}  // namespace api

/**
 * The API class to communicate with MYNT® EYE device.
 */
class MYNTEYE_API API {
 public:
  /** The api::StreamData callback. */
  using stream_callback_t = std::function<void(const api::StreamData &data)>;
  /** The api::MotionData callback. */
  using motion_callback_t = std::function<void(const api::MotionData &data)>;
  /** The enable/disable switch callback. */
  using stream_switch_callback_t = std::function<void(const Stream &stream)>;

  explicit API(std::shared_ptr<Device> device, CalibrationModel calib_model);
  virtual ~API();

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
      int argc, char *argv[], const std::shared_ptr<Device> &device);
  /**
   * Create the API instance.
   * @param device the selected device.
   * @return the API instance.
   */
  static std::shared_ptr<API> Create(const std::shared_ptr<Device> &device);

  /**
   * Get the model.
   */
  Model GetModel() const;

  /**
   * Supports the stream or not.
   */
  bool Supports(const Stream &stream) const;
  /**
   * Supports the capability or not.
   */
  bool Supports(const Capabilities &capability) const;
  /**
   * Supports the option or not.
   */
  bool Supports(const Option &option) const;
  /**
   * Supports the addon or not.
   */
  bool Supports(const AddOns &addon) const;

  /**
   * Log all stream requests and prompt user to select one.
   */
  StreamRequest SelectStreamRequest(bool *ok) const;

  /**
   * Get all stream requests of the capability.
   */
  const std::vector<StreamRequest> &GetStreamRequests(
      const Capabilities &capability) const;
  /**
   * Config the stream request to the capability.
   */
  void ConfigStreamRequest(
      const Capabilities &capability, const StreamRequest &request);
  /**
   * Get the config stream requests of the capability.
   */
  const StreamRequest &GetStreamRequest(const Capabilities &capability) const;

  /**
   * Get all stream requests of the key stream capability.
   */
  const std::vector<StreamRequest> &GetStreamRequests() const;
  /**
   * Config the stream request to the key stream capability.
   */
  void ConfigStreamRequest(const StreamRequest &request);
  /**
   * Get the config stream requests of the key stream capability.
   */
  const StreamRequest &GetStreamRequest() const;

  /**
   * Get the device info.
   */
  std::shared_ptr<DeviceInfo> GetInfo() const;
  /**
   * Get the device info.
   */
  std::string GetInfo(const Info &info) const;
  /**
   * Get the sdk version.
   */
  std::string GetSDKVersion() const;
  /**
   * @deprecated Get the intrinsics (pinhole) of stream.
   */
  IntrinsicsPinhole GetIntrinsics(const Stream &stream) const;
  /**
   * Get the intrinsics of stream.
   */
  template <typename T>
  T GetIntrinsics(const Stream &stream) const;
  /**
   * Get the intrinsics base of stream.
   */
  std::shared_ptr<IntrinsicsBase> GetIntrinsicsBase(const Stream &stream) const;
  /**
   * Get the extrinsics from one stream to another.
   */
  Extrinsics GetExtrinsics(const Stream &from, const Stream &to) const;
  /**
   * Get the intrinsics of motion.
   */
  MotionIntrinsics GetMotionIntrinsics() const;
  /**
   * Get the extrinsics from one stream to motion.
   */
  Extrinsics GetMotionExtrinsics(const Stream &from) const;

  /**
   * Log all option infos.
   */
  void LogOptionInfos() const;
  /**
   * Get the option info.
   */
  OptionInfo GetOptionInfo(const Option &option) const;

  /**
   * Get the option value.
   */
  std::int32_t GetOptionValue(const Option &option) const;

  /**
   * Set the disparity computing method.
   */
  void SetDisparityComputingMethodType(
      const DisparityComputingMethod &MethodType);

  /**
   * Set the rectify bord cut alpha.
   */
  void SetRectifyAlpha(const double &alpha);

  /**
   * Set if the duplicate frames is enable.
   */
  void setDuplicate(bool isEnable);

  /**
   * Set the option value.
   */
  void SetOptionValue(const Option &option, std::int32_t value);

  /**
   * Set the option value.
   */
  bool SetOptionValue(const Option &option, std::uint64_t value);

  /**
   * Run the option action.
   */
  bool RunOptionAction(const Option &option) const;

  /**
   * Set the callback of stream.
   */
  void SetStreamCallback(const Stream &stream, stream_callback_t callback);
  /**
   * Set the callback of motion.
   */
  void SetMotionCallback(motion_callback_t callback);

  /**
   * Has the callback of stream.
   */
  bool HasStreamCallback(const Stream &stream) const;
  /**
   * Has the callback of motion.
   */
  bool HasMotionCallback() const;

  /**
   * Start capturing the source.
   */
  void Start(const Source &source);
  /**
   * Stop capturing the source.
   */
  void Stop(const Source &source);

  /**
   * Wait the streams are ready.
   */
  void WaitForStreams();

  /**
   * Enable the data of stream.
   * @note must enable the stream if it's a synthetic one. This means the stream
   * in not native, the device has the capability to provide this stream, but
   * still support this stream.
   */
  void EnableStreamData(const Stream &stream);

  /**
   * Enable the data of stream.
   * callback function will call before the father processor enable.
   * when try_tag is true, the function will do nothing except callback.
   */
  void EnableStreamData(
      const Stream &stream,
      stream_switch_callback_t callback,
      bool try_tag = false);
  /**
   * Disable the data of stream.
   */
  void DisableStreamData(const Stream &stream);

  /**
   * Disable the data of stream.
   * callback function will call before the children processor disable.
   * when try_tag is true, the function will do nothing except callback.
   */
  void DisableStreamData(
      const Stream &stream,
      stream_switch_callback_t callback,
      bool try_tag = false);

  /**
   * Get the latest data of stream.
   */
  api::StreamData GetStreamData(const Stream &stream);
  /**
   * Get the datas of stream.
   * @note default cache 4 datas at most.
   */
  std::vector<api::StreamData> GetStreamDatas(const Stream &stream);

  /**
   * Enable cache motion datas.
   */
  void EnableMotionDatas(
      std::size_t max_size = std::numeric_limits<std::size_t>::max());
  /**
   * Get the motion datas.
   */
  std::vector<api::MotionData> GetMotionDatas();

  /**
   * enable motion datas timestamp correspondence in device.
   */
  void EnableImuTimestampCorrespondence(bool is_enable);

  /**
   * Enable motion datas with timestamp correspondence of some stream in api.
   */
  void EnableTimestampCorrespondence(const Stream &stream,
        bool keep_accel_then_gyro = true);

  /**
   * Enable the plugin.
   */
  void EnablePlugin(const std::string &path);

  std::shared_ptr<Device> device();

  /** Enable process mode, e.g. imu assembly, temp_drift */
  void EnableProcessMode(const ProcessMode& mode);
  /** Enable process mode, e.g. imu assembly, temp_drift */
  void EnableProcessMode(const std::int32_t& mode);

  /** 
   * Get ROS need camera info struct
   */
  std::shared_ptr<struct CameraROSMsgInfoPair> GetCameraROSMsgInfoPair();

  /**
   * Load disparity config from file.
   */
  bool ConfigDisparityFromFile(const std::string& config_file);

  bool IsDefaultIntrinsics();

 private:
  std::shared_ptr<Device> device_;

  std::unique_ptr<Synthetic> synthetic_;

  std::unique_ptr<Correspondence> correspondence_;

  motion_callback_t callback_;

  bool api_correspondence_enable_;
  bool dev_correspondence_enable_;

  void CheckImageParams();
};

template <typename T>
T API::GetIntrinsics(const Stream &stream) const {
  auto in = GetIntrinsicsBase(stream);
  return *std::dynamic_pointer_cast<T>(in);
}

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_API_API_H_
