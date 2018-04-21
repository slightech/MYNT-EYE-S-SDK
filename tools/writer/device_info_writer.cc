#include "mynteye/glog_init.h"

#include "mynteye/device.h"
#include "mynteye/utils.h"

#include "writer/device_writer.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  glog_init _(argc, argv);

  const char *filepath;
  if (argc >= 2) {
    filepath = argv[1];
  } else {
    LOG(ERROR) << "Usage: ./device_info_writer <filepath>";
    return 2;
  }

  auto &&device = device::select();

  tools::DeviceWriter writer(device);
  writer.WriteDeviceInfo(filepath);

  return 0;
}
