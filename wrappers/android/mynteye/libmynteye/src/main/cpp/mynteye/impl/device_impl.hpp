#pragma once

#include <vector>
#include <memory>

#include "mynteye/device/device.h"

#include "device.hpp"

namespace mynteye_jni {

class DeviceImpl : public Device {
 public:
  using device_t = std::shared_ptr<MYNTEYE_NAMESPACE::Device>;

  explicit DeviceImpl(const device_t & device);
  ~DeviceImpl();

  /** Get the model */
  ::mynteye_jni::Model GetModel() override;

  /** Supports the stream or not */
  bool SupportsStream(::mynteye_jni::Stream stream) override;

  /** Supports the capability or not */
  bool SupportsCapability(::mynteye_jni::Capability capabilities) override;

  /** Supports the option or not */
  bool SupportsOption(::mynteye_jni::Option option) override;

  /** Supports the addon or not */
  bool SupportsAddon(::mynteye_jni::Addon addon) override;

  /** Get all stream requests */
  std::vector<::mynteye_jni::StreamRequest> GetStreamRequests() override;

  /** Config the stream request */
  void ConfigStreamRequest(const ::mynteye_jni::StreamRequest & request) override;

  /** Get the device info */
  std::string GetInfo(::mynteye_jni::Info info) override;

  /** Get the intrinsics of stream */
  ::mynteye_jni::Intrinsics GetIntrinsics(::mynteye_jni::Stream stream) override;

  /** Get the extrinsics of stream */
  ::mynteye_jni::Extrinsics GetExtrinsics(::mynteye_jni::Stream from, ::mynteye_jni::Stream to) override;

  /** Get the intrinsics of motion */
  ::mynteye_jni::MotionIntrinsics GetMotionIntrinsics() override;

  /** Get the extrinsics from one stream to motion */
  ::mynteye_jni::Extrinsics GetMotionExtrinsics(::mynteye_jni::Stream from) override;

  /** Get the option info */
  ::mynteye_jni::OptionInfo GetOptionInfo(::mynteye_jni::Option option) override;

  /** Get the option value */
  int32_t GetOptionValue(::mynteye_jni::Option option) override;

  /** Set the option value */
  void SetOptionValue(::mynteye_jni::Option option, int32_t value) override;

  /** Run the option value */
  bool RunOptionAction(::mynteye_jni::Option option) override;

  /** Start capturing the source */
  void Start(::mynteye_jni::Source source) override;

  /** Stop capturing the source */
  void Stop(::mynteye_jni::Source source) override;

  /** Wait the streams are ready */
  void WaitForStreams() override;

  /** Get the latest data of stream */
  std::shared_ptr<::mynteye_jni::StreamData> GetStreamData(::mynteye_jni::Stream stream) override;

  /** Get the datas of stream */
  std::vector<std::shared_ptr<::mynteye_jni::StreamData>> GetStreamDatas(::mynteye_jni::Stream stream) override;

  /** Enable cache motion datas until get them, otherwise using callback instead */
  void EnableMotionDatas(int32_t max_size) override;

  /** Get the motion datas */
  std::vector<std::shared_ptr<::mynteye_jni::MotionData>> GetMotionDatas() override;

 private:
  device_t device_;
};

}  // namespace mynteye_jni
