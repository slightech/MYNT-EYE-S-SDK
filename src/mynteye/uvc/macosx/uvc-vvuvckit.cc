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

#include <thread>
#include <vector>
#include <atomic>
#include <mutex>
#include "mynteye/logger.h"
#include "mynteye/uvc/uvc.h"
#include "libuvc/libuvc.h"
#include "AVfoundationCamera.h"

// #define ENABLE_DEBUG_SPAM

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

// Enumerate devices
MYNTEYE_API std::shared_ptr<context> create_context() {
  //too
}
MYNTEYE_API std::vector<std::shared_ptr<device>> query_devices(
    std::shared_ptr<context> context) {
  //todo
}

// Static device properties
MYNTEYE_API std::string get_name(const device &device) {
  //todo
}
MYNTEYE_API int get_vendor_id(const device &device) {
  //todo
}
MYNTEYE_API int get_product_id(const device &device) {
  //todo
}

MYNTEYE_API std::string get_video_name(const device &device) {
  //todo
}

MYNTEYE_API bool pu_control_range(
    const device &device, Option option, int32_t *min, int32_t *max,
    int32_t *def) {
  // todo
}
MYNTEYE_API bool pu_control_query(
    const device &device, Option option, pu_query query, int32_t *value) {
  // todo
}

// Access XU (Extension Unit) controls
MYNTEYE_API bool xu_control_range(
    const device &device, const xu &xu, uint8_t selector, uint8_t id,
    int32_t *min, int32_t *max, int32_t *def) {
  //todo
}
MYNTEYE_API bool xu_control_query(  // XU_QUERY_SET, XU_QUERY_GET
    const device &device, const xu &xu, uint8_t selector, xu_query query,
    uint16_t size, uint8_t *data) {
  //todo
}

MYNTEYE_API void set_device_mode(
    device &device, int width, int height, int fourcc, int fps,  // NOLINT
    video_channel_callback callback) {
  //todo
}
MYNTEYE_API void start_streaming(device &device, int num_transfer_bufs) {
  //todo
}
MYNTEYE_API void stop_streaming(device &device) {
  //todo
}

}  // namespace uvc

MYNTEYE_END_NAMESPACE
