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
#ifndef MYNTEYE_DEVICE_DEVICE_H_
#define MYNTEYE_DEVICE_DEVICE_H_
#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "mynteye/mynteye.h"
#include "mynteye/types.h"
#include "mynteye/device/callbacks.h"

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

struct device;

}  // namespace uvc

namespace tools {

class DeviceWriter;

}  // namespace tools

struct DeviceInfo;

class API;
class Channels;
class Motions;
class Streams;

template <class Data>
class AsyncCallback;

/**
 * The Device class to communicate with MYNT® EYE device.
 */
class MYNTEYE_API Device {
 public:
  /** The device::StreamData callback. */
  using stream_callback_t = device::StreamCallback;
  /** The device::MotionData callback. */
  using motion_callback_t = device::MotionCallback;

  using stream_callbacks_t = std::map<Stream, stream_callback_t>;

  using stream_async_callback_t = AsyncCallback<device::StreamData>;
  using motion_async_callback_t = AsyncCallback<device::MotionData>;
  using stream_async_callback_ptr_t = std::shared_ptr<stream_async_callback_t>;
  using motion_async_callback_ptr_t = std::shared_ptr<motion_async_callback_t>;

  Device(const Model &model, std::shared_ptr<uvc::device> device);
  virtual ~Device();

  /**
   * Create the Device instance.
   * @param name the device name.
   * @param device the device from uvc.
   * @return the Device instance.
   */
  static std::shared_ptr<Device> Create(
      const std::string &name, std::shared_ptr<uvc::device> device);

  /**
   * Get the model.
   */
  Model GetModel() const {
    return model_;
  }

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
   * Get the device info.
   */
  std::shared_ptr<DeviceInfo> GetInfo() const;
  /**
   * Get the device info of a field.
   */
  std::string GetInfo(const Info &info) const;

  /**
   * Get the intrinsics of stream.
   */
  Intrinsics GetIntrinsics(const Stream &stream) const;
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
   * Get the intrinsics of stream.
   */
  Intrinsics GetIntrinsics(const Stream &stream, bool *ok) const;
  /**
   * Get the extrinsics from one stream to another.
   */
  Extrinsics GetExtrinsics(
      const Stream &from, const Stream &to, bool *ok) const;
  /**
   * Get the intrinsics of motion.
   */
  MotionIntrinsics GetMotionIntrinsics(bool *ok) const;
  /**
   * Get the extrinsics from one stream to motion.
   */
  Extrinsics GetMotionExtrinsics(const Stream &from, bool *ok) const;

  /**
   * Set the intrinsics of stream.
   */
  void SetIntrinsics(const Stream &stream, const Intrinsics &in);
  /**
   * Set the extrinsics from one stream to another.
   */
  void SetExtrinsics(
      const Stream &from, const Stream &to, const Extrinsics &ex);
  /**
   * Set the intrinsics of motion.
   */
  void SetMotionIntrinsics(const MotionIntrinsics &in);
  /**
   * Set the extrinsics from one stream to motion.
   */
  void SetMotionExtrinsics(const Stream &from, const Extrinsics &ex);

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
   * Set the option value.
   */
  void SetOptionValue(const Option &option, std::int32_t value);

  /**
   * Run the option action.
   */
  bool RunOptionAction(const Option &option) const;

  /**
   * Set the callback of stream.
   */
  void SetStreamCallback(
      const Stream &stream, stream_callback_t callback, bool async = false);
  /**
   * Set the callback of motion.
   */
  void SetMotionCallback(motion_callback_t callback, bool async = false);

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
  virtual void Start(const Source &source);
  /**
   * Stop capturing the source.
   */
  virtual void Stop(const Source &source);

  /**
   * Wait the streams are ready.
   */
  void WaitForStreams();

  /**
   * Get the datas of stream.
   * @note default cache 4 datas at most.
   */
  std::vector<device::StreamData> GetStreamDatas(const Stream &stream);
  /**
   * Get the latest data of stream.
   */
  device::StreamData GetLatestStreamData(const Stream &stream);

  /**
   * Enable cache motion datas.
   */
  void EnableMotionDatas();
  /**
   * Enable cache motion datas.
   */
  void EnableMotionDatas(std::size_t max_size);
  /**
   * Get the motion datas.
   */
  std::vector<device::MotionData> GetMotionDatas();

 protected:
  std::shared_ptr<uvc::device> device() const {
    return device_;
  }

  std::shared_ptr<Streams> streams() const {
    return streams_;
  }

  std::shared_ptr<Channels> channels() const {
    return channels_;
  }

  std::shared_ptr<Motions> motions() const {
    return motions_;
  }

  const StreamRequest &GetStreamRequest(const Capabilities &capability);

  virtual void StartVideoStreaming();
  virtual void StopVideoStreaming();

  virtual void StartMotionTracking();
  virtual void StopMotionTracking();

  virtual void OnStereoStreamUpdate();

  virtual std::vector<Stream> GetKeyStreams() const = 0;

  bool video_streaming_;
  bool motion_tracking_;

 private:
  Model model_;
  std::shared_ptr<uvc::device> device_;
  std::shared_ptr<DeviceInfo> device_info_;

  std::map<Stream, Intrinsics> stream_intrinsics_;
  std::map<Stream, std::map<Stream, Extrinsics>> stream_from_extrinsics_;

  std::shared_ptr<MotionIntrinsics> motion_intrinsics_;
  std::map<Stream, Extrinsics> motion_from_extrinsics_;

  stream_callbacks_t stream_callbacks_;
  motion_callback_t motion_callback_;

  std::map<Stream, stream_async_callback_ptr_t> stream_async_callbacks_;
  motion_async_callback_ptr_t motion_async_callback_;

  std::shared_ptr<Streams> streams_;

  std::map<Capabilities, StreamRequest> stream_config_requests_;

  std::mutex mtx_streams_;

  std::shared_ptr<Channels> channels_;

  std::shared_ptr<Motions> motions_;

  void ReadAllInfos();

  void CallbackPushedStreamData(const Stream &stream);
  void CallbackMotionData(const device::MotionData &data);

  friend API;
  friend tools::DeviceWriter;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_DEVICE_H_
