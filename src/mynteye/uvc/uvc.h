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
#ifndef MYNTEYE_UVC_UVC_H_
#define MYNTEYE_UVC_UVC_H_
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "mynteye/mynteye.h"
#include "mynteye/types.h"

#define MYNTEYE_VID 0x04B4
#define MYNTEYE_PID 0x00F9

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

typedef enum pu_query {
  PU_QUERY_SET,  // Set the value of a control
  PU_QUERY_GET,  // Get the value of a control
  PU_QUERY_LAST
} pu_query;

struct MYNTEYE_API guid {
  uint32_t data1;
  uint16_t data2, data3;
  uint8_t data4[8];
};

// Extension Unit
struct MYNTEYE_API xu {
  uint8_t unit;
  int node;
  guid id;
};

typedef enum xu_query {
  XU_QUERY_SET,  // Set current value of the control
  XU_QUERY_GET,  // Get current value of the control
  XU_QUERY_MIN,  // Get min value of the control
  XU_QUERY_MAX,  // Get max value of the control
  XU_QUERY_DEF,  // Get default value of the control
  XU_QUERY_LAST
} xu_query;

struct context;  // Opaque type representing access to the underlying UVC
                 // implementation
struct device;   // Opaque type representing access to a specific UVC device

// Enumerate devices
MYNTEYE_API std::shared_ptr<context> create_context();
MYNTEYE_API std::vector<std::shared_ptr<device>> query_devices(
    std::shared_ptr<context> context);

// Static device properties
MYNTEYE_API std::string get_name(const device &device);
MYNTEYE_API int get_vendor_id(const device &device);
MYNTEYE_API int get_product_id(const device &device);

MYNTEYE_API std::string get_video_name(const device &device);

// Access PU (Processing Unit) controls
inline bool is_pu_control(Option option) {
  return option >= Option::GAIN && option <= Option::CONTRAST;
}
MYNTEYE_API bool pu_control_range(
    const device &device, Option option, int32_t *min, int32_t *max,
    int32_t *def);
MYNTEYE_API bool pu_control_query(
    const device &device, Option option, pu_query query, int32_t *value);

// Access XU (Extension Unit) controls
MYNTEYE_API bool xu_control_range(
    const device &device, const xu &xu, uint8_t selector, uint8_t id,
    int32_t *min, int32_t *max, int32_t *def);
MYNTEYE_API bool xu_control_query(  // XU_QUERY_SET, XU_QUERY_GET
    const device &device, const xu &xu, uint8_t selector, xu_query query,
    uint16_t size, uint8_t *data);

// Control streaming
typedef std::function<void(const void *frame,
    std::function<void()> continuation)> video_channel_callback;

MYNTEYE_API void set_device_mode(
    device &device, int width, int height, int fourcc, int fps,  // NOLINT
    video_channel_callback callback);
MYNTEYE_API void start_streaming(device &device, int num_transfer_bufs);  // NOLINT
MYNTEYE_API void stop_streaming(device &device);                          // NOLINT

}  // namespace uvc

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_UVC_UVC_H_
