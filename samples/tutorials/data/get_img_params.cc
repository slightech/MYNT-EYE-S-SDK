#include <glog/logging.h>

#include "mynteye/api.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);

  LOG(INFO) << "Intrinsics left: {" << api->GetIntrinsics(Stream::LEFT) << "}";
  LOG(INFO) << "Intrinsics right: {" << api->GetIntrinsics(Stream::RIGHT)
            << "}";
  LOG(INFO) << "Extrinsics left to right: {"
            << api->GetExtrinsics(Stream::LEFT, Stream::RIGHT) << "}";

  return 0;
}
