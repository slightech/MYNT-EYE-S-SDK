#ifndef MYNTEYE_API_H_  // NOLINT
#define MYNTEYE_API_H_
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "mynteye/mynteye.h"
#include "mynteye/types.h"

MYNTEYE_BEGIN_NAMESPACE

class Device;
class Synthetic;

class MYNTEYE_API API {
 public:
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

  std::shared_ptr<Device> device();

 private:
  std::shared_ptr<Device> device_;

  std::unique_ptr<Synthetic> synthetic_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_API_H_ NOLINT
