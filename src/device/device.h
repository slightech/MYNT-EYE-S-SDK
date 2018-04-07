#ifndef MYNTEYE_DEVICE_H_  // NOLINT
#define MYNTEYE_DEVICE_H_
#pragma once

#include <map>
#include <memory>
#include <string>

#include "mynteye/mynteye.h"
#include "mynteye/types.h"

#include "internal/callbacks.h"

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

struct device;

}  // namespace uvc

struct DeviceInfo;

class Device {
 public:
  using stream_callback_t = device::StreamCallback;
  using motion_callback_t = device::MotionCallback;

  using stream_callbacks_t = std::map<Stream, stream_callback_t>;

  Device(const Model &model, std::shared_ptr<uvc::device> device);
  virtual ~Device();

  static std::shared_ptr<Device> Create(
      const std::string &name, std::shared_ptr<uvc::device> device);

  Model GetModel() const {
    return model_;
  }

  bool Supports(const Stream &stream) const;
  bool Supports(const Capabilities &capability) const;
  bool Supports(const Option &option) const;

  std::shared_ptr<DeviceInfo> GetInfo() const;
  std::string GetInfo(const Info &info) const;

  ImgIntrinsics GetImgIntrinsics() const;
  ImgExtrinsics GetImgExtrinsics() const;

  ImuIntrinsics GetImuIntrinsics() const;
  ImuExtrinsics GetImuExtrinsics() const;

  void SetStreamCallback(const Stream &stream, stream_callback_t callback);
  void SetMotionCallback(motion_callback_t callback);

  virtual void Start(const Source &source);
  virtual void Stop(const Source &source);

 protected:
  std::shared_ptr<uvc::device> device() const {
    return device_;
  }

  std::shared_ptr<DeviceInfo> device_info() const {
    return device_info_;
  }

  StreamRequest GetStreamRequest(const Capabilities &capability) const;

  virtual void StartVideoStreaming();
  virtual void StopVideoStreaming();

  virtual void StartMotionTracking();
  virtual void StopMotionTracking();

 private:
  Model model_;
  std::shared_ptr<uvc::device> device_;
  std::shared_ptr<DeviceInfo> device_info_;

  ImgIntrinsics img_intrinsics_;
  ImgExtrinsics img_extrinsics_;
  ImuIntrinsics imu_intrinsics_;
  ImuExtrinsics imu_extrinsics_;

  stream_callbacks_t stream_callbacks_;
  motion_callback_t motion_callback_;

  void ReadDeviceInfo();

  void WriteImgIntrinsics(const ImgIntrinsics &intrinsics);
  void WriteImgExtrinsics(const ImgExtrinsics &extrinsics);

  void WriteImuIntrinsics(const ImuIntrinsics &intrinsics);
  void WriteImuExtrinsics(const ImuExtrinsics &extrinsics);

  // friend DeviceWriter;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_H_ NOLINT
