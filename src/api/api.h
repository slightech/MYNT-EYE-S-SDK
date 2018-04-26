#ifndef MYNTEYE_API_H_  // NOLINT
#define MYNTEYE_API_H_
#pragma once

#include <memory>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

class Device;

class MYNTEYE_API API {
 public:
  explicit API(std::shared_ptr<Device> device);
  /*virtual*/ ~API();

  static std::shared_ptr<API> Create();
  static std::shared_ptr<API> Create(std::shared_ptr<Device> device);

 private:
  std::shared_ptr<Device> device_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_API_H_ NOLINT
