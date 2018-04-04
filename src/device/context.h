#ifndef MYNTEYE_CONTEXT_H_  // NOLINT
#define MYNTEYE_CONTEXT_H_
#pragma once

#include <memory>
#include <vector>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

struct context;

}  // namespace uvc

class Device;

class Context {
 public:
  Context();
  ~Context();

  std::vector<std::shared_ptr<Device>> devices() const {
    return devices_;
  }

 private:
  std::shared_ptr<uvc::context> context_;
  std::vector<std::shared_ptr<Device>> devices_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_CONTEXT_H_ NOLINT
