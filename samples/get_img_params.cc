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
#include "mynteye/api/api.h"
#include "mynteye/types.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);
  if (!api) return 1;



  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);
  auto in_left = api->GetIntrinsicsBase(Stream::LEFT);
  auto in_right = api->GetIntrinsicsBase(Stream::RIGHT);
  if (in_left->calib_model() == CalibrationModel::PINHOLE) {
    in_left = std::dynamic_pointer_cast<IntrinsicsPinhole>(in_left);
    in_right = std::dynamic_pointer_cast<IntrinsicsPinhole>(in_right);
  } else if (in_left->calib_model() == CalibrationModel::KANNALA_BRANDT) {
    in_left = std::dynamic_pointer_cast<IntrinsicsEquidistant>(in_left);
    in_right = std::dynamic_pointer_cast<IntrinsicsEquidistant>(in_right);
  } else {
    LOG(INFO) << "UNKNOW CALIB MODEL.";
    return 0;
  }
  in_left -> ResizeIntrinsics();
  in_right -> ResizeIntrinsics();
  LOG(INFO) << "Intrinsics left: {" << *in_left << "}";
  LOG(INFO) << "Intrinsics right: {" << *in_right << "}";
  LOG(INFO) << "Extrinsics right to left: {"
            << api->GetExtrinsics(Stream::RIGHT, Stream::LEFT) << "}";

  auto info = api->GetCameraROSMsgInfoPair();

  if (info && !info->isEmpty()) {
    LOG(INFO) << "ROSMsgInfoPair:";
    LOG(INFO) << *info;
    LOG(INFO) << "If you cant't have a clear understanding of the info,"
                 "you can read the ROS-doc (http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html)"  // NOLINT
                 " to learn more.";
  }

  if (api->IsDefaultIntrinsics()) {
    LOG(WARNING) << "Default intrinsics are currently being used.";
    LOG(WARNING) << "Image params not found, but we need it to process the "
                  "images. Please use `img_params_writer` "
                  "to write the image params. If you update the SDK from "
                  "1.x, the `SN*.conf` is the file contains them. Besides, "
                  "you could also calibrate them by yourself. Read the guide "
                  "doc (https://github.com/slightech/MYNT-EYE-SDK-2-Guide) "
                  "to learn more.";
  }

  return 0;
}
