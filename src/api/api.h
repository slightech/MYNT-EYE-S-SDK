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

  static std::shared_ptr<API> Create();
  static std::shared_ptr<API> Create(std::shared_ptr<Device> device);

  Model GetModel() const;

  bool Supports(const Stream &stream) const;
  bool Supports(const Capabilities &capability) const;
  bool Supports(const Option &option) const;

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
  api::StreamData GetStreamData(const Stream &stream);
  std::vector<api::StreamData> GetStreamDatas(const Stream &stream);

  void EnableMotionDatas(
      std::size_t max_size = std::numeric_limits<std::size_t>::max());
  std::vector<api::MotionData> GetMotionDatas();

  std::shared_ptr<Device> device();

 private:
  std::shared_ptr<Device> device_;

  std::unique_ptr<Synthetic> synthetic_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_API_H_ NOLINT
