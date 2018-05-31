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
#include "uvc/uvc.h"  // NOLINT

#include <glog/logging.h>
#include <libuvc/libuvc.h>

// #define ENABLE_DEBUG_SPAM

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

static void check(const char *call, uvc_error_t status) {
  LOG_IF(FATAL, status < 0)
      << call << "(...) returned " << uvc_strerror(status);
}
#define CALL_UVC(name, ...) check(#name, name(__VA_ARGS__))

struct context {
  uvc_context_t *ctx;

  context() : ctx(nullptr) {
    VLOG(2) << __func__;
    CALL_UVC(uvc_init, &ctx, nullptr);
  }

  ~context() {
    VLOG(2) << __func__;
    if (ctx)
      uvc_exit(ctx);
  }
};

struct device {
  const std::shared_ptr<context> parent;

  uvc_device_t *uvcdevice = nullptr;
  uvc_device_handle_t *handle = nullptr;

  int vid, pid;

  device(std::shared_ptr<context> parent, uvc_device_t *uvcdevice)
      : parent(parent), uvcdevice(uvcdevice) {
    VLOG(2) << __func__;
    open();

    uvc_device_descriptor_t *desc;
    CALL_UVC(uvc_get_device_descriptor, uvcdevice, &desc);
    vid = desc->idVendor;
    pid = desc->idProduct;
    uvc_free_device_descriptor(desc);
  }

  ~device() {
    VLOG(2) << __func__;
    if (handle)
      uvc_close(handle);
    if (uvcdevice)
      uvc_unref_device(uvcdevice);
  }

  void open() {
    if (!handle)
      CALL_UVC(uvc_open, uvcdevice, &handle);
  }
};

std::shared_ptr<context> create_context() {
  return std::make_shared<context>();
}

std::vector<std::shared_ptr<device>> query_devices(
    std::shared_ptr<context> context) {
  std::vector<std::shared_ptr<device>> devices;

  uvc_device_t **list;
  CALL_UVC(uvc_get_device_list, context->ctx, &list);
  for (auto it = list; *it; ++it) {
    try {
      auto dev = std::make_shared<device>(context, *it);
      devices.push_back(dev);
    } catch (std::runtime_error &e) {
      LOG(WARNING) << "usb:" << static_cast<int>(uvc_get_bus_number(*it)) << ':'
                   << static_cast<int>(uvc_get_device_address(*it)) << ": "
                   << e.what();
    }
  }
  uvc_free_device_list(list, 1);

  return devices;
}

int get_vendor_id(const device &device) {
  return device.vid;
}

int get_product_id(const device &device) {
  return device.pid;
}

std::string get_name(const device &device) {
  // TODO(JohnZhao)
  UNUSED(device)
  return "";
}

std::string get_video_name(const device &device) {
  // TODO(JohnZhao)
  UNUSED(device)
  return "";
}

bool pu_control_range(
    const device &device, Option option, int32_t *min, int32_t *max,
    int32_t *def) {
  // TODO(JohnZhao)
  UNUSED(device)
  UNUSED(option)
  UNUSED(min)
  UNUSED(max)
  UNUSED(def)
  return false;
}

bool pu_control_query(
    const device &device, Option option, pu_query query, int32_t *value) {
  // TODO(JohnZhao)
  UNUSED(device)
  UNUSED(option)
  UNUSED(query)
  UNUSED(value)
  return false;
}

bool xu_control_range(
    const device &device, const xu &xu, uint8_t selector, uint8_t id, int32_t *min,
    int32_t *max, int32_t *def) {
  // TODO(JohnZhao)
  UNUSED(device)
  UNUSED(xu)
  UNUSED(selector)
  UNUSED(id)
  UNUSED(min)
  UNUSED(max)
  UNUSED(def)
  return false;
}

bool xu_control_query(
    const device &device, const xu &xu, uint8_t selector, xu_query query,
    uint16_t size, uint8_t *data) {
  // TODO(JohnZhao)
  UNUSED(device)
  UNUSED(xu)
  UNUSED(selector)
  UNUSED(query)
  UNUSED(size)
  UNUSED(data)
  return false;
}

void set_device_mode(
    device &device, int width, int height, int fourcc, int fps,  // NOLINT
    video_channel_callback callback) {
  // TODO(JohnZhao)
  UNUSED(device)
  UNUSED(width)
  UNUSED(height)
  UNUSED(fourcc)
  UNUSED(fps)
  UNUSED(callback)
}

void start_streaming(device &device, int num_transfer_bufs) {  // NOLINT
  // TODO(JohnZhao)
  UNUSED(device)
  UNUSED(num_transfer_bufs)
}

void stop_streaming(device &device) {  // NOLINT
  // TODO(JohnZhao)
  UNUSED(device)
}

}  // namespace uvc

MYNTEYE_END_NAMESPACE
