#include <glog/logging.h>

#include "mynteye/api.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);

  LOG(INFO) << "Motion intrinsics: {" << api->GetMotionIntrinsics() << "}";
  LOG(INFO) << "Motion extrinsics left to imu: {"
            << api->GetMotionExtrinsics(Stream::LEFT) << "}";

  return 0;
}
