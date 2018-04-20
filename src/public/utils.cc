#include "mynteye/utils.h"

#include <glog/logging.h>

#include "device/context.h"
#include "device/device.h"

MYNTEYE_BEGIN_NAMESPACE

namespace device {

std::shared_ptr<Device> select() {
  LOG(INFO) << "Detecting MYNT EYE devices";
  Context context;
  auto &&devices = context.devices();

  size_t n = devices.size();
  LOG_IF(FATAL, n <= 0) << "No MYNT EYE devices :(";

  LOG(INFO) << "MYNT EYE devices:";
  for (size_t i = 0; i < n; i++) {
    auto &&device = devices[i];
    auto &&name = device->GetInfo(Info::DEVICE_NAME);
    LOG(INFO) << "  index: " << i << ", name: " << name;
  }

  std::shared_ptr<Device> device = nullptr;
  if (n <= 1) {
    device = devices[0];
    LOG(INFO) << "Only one MYNT EYE device, select index: 0";
  } else {
    while (true) {
      size_t i;
      LOG(INFO) << "There are " << n << " MYNT EYE devices, select index: ";
      std::cin >> i;
      if (i >= n) {
        LOG(WARNING) << "Index out of range :(";
        continue;
      }
      device = devices[i];
      break;
    }
  }

  return device;
}

}  // namespace device

MYNTEYE_END_NAMESPACE
