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

  /** Get all stream requests */
  std::vector<::mynteye_jni::StreamRequest> GetStreamRequests() override;

  /** Config the stream request */
  void ConfigStreamRequest(const ::mynteye_jni::StreamRequest & request) override;

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

  /** Enable cache motion datas */
  void EnableCacheMotionDatas(int32_t max_size) override;

  /** Get the motion datas */
  std::vector<std::shared_ptr<::mynteye_jni::MotionData>> GetMotionDatas() override;

 private:
  device_t device_;
};

}  // namespace mynteye_jni
