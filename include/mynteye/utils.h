#ifndef MYNTEYE_UTILS_H_  // NOLINT
#define MYNTEYE_UTILS_H_
#pragma once

#include <memory>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

class Device;

namespace device {

/** Detecting MYNT EYE devices and prompt user to select one */
MYNTEYE_API std::shared_ptr<Device> select();

}  // namespace device

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_UTILS_H_ NOLINT
