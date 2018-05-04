#include <glog/logging.h>

#include "mynteye/api.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);

  LOG(INFO) << "Device name: " << api->GetInfo(Info::DEVICE_NAME);
  LOG(INFO) << "Serial number: " << api->GetInfo(Info::SERIAL_NUMBER);
  LOG(INFO) << "Firmware version: " << api->GetInfo(Info::FIRMWARE_VERSION);
  LOG(INFO) << "Hardware version: " << api->GetInfo(Info::HARDWARE_VERSION);
  LOG(INFO) << "Spec version: " << api->GetInfo(Info::SPEC_VERSION);
  LOG(INFO) << "Lens type: " << api->GetInfo(Info::LENS_TYPE);
  LOG(INFO) << "IMU type: " << api->GetInfo(Info::IMU_TYPE);
  LOG(INFO) << "Nominal baseline: " << api->GetInfo(Info::NOMINAL_BASELINE);

  return 0;
}
