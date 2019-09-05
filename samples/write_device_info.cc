// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "mynteye/logger.h"
#include "mynteye/device/device.h"
#include "mynteye/device/utils.h"

#include "device_writer.h"

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
  if (!device)
    return 1;

  tools::DeviceWriter writer(device);
  writer.WriteDeviceInfo(filepath);

  return 0;
}
