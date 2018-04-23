#include "uvc/uvc.h"  // NOLINT

#include <glog/logging.h>

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

struct context {
  context() {
    VLOG(2) << __func__;
  }

  ~context() {
    VLOG(2) << __func__;
  }
};

struct device {
  const std::shared_ptr<context> parent;

  int vid, pid;

  device(std::shared_ptr<context> parent) : parent(parent) {
    VLOG(2) << __func__;
  }

  ~device() {
    VLOG(2) << __func__;
  }
};

std::shared_ptr<context> create_context() {
  return std::make_shared<context>();
}

std::vector<std::shared_ptr<device>> query_devices(
    std::shared_ptr<context> context) {
  std::vector<std::shared_ptr<device>> devices;
  UNUSED(context)
  return devices;
}

int get_vendor_id(const device &device) {
  return device.vid;
}

int get_product_id(const device &device) {
  return device.pid;
}

std::string get_name(const device &device) {
  UNUSED(device)
  return "";
}

std::string get_video_name(const device &device) {
  UNUSED(device)
  return "";
}

bool pu_control_range(
    const device &device, Option option, int32_t *min, int32_t *max,
    int32_t *def) {
  UNUSED(device)
  UNUSED(option)
  UNUSED(min)
  UNUSED(max)
  UNUSED(def)
  return false;
}

bool pu_control_query(
    const device &device, Option option, pu_query query, int32_t *value) {
  UNUSED(device)
  UNUSED(option)
  UNUSED(query)
  UNUSED(value)
  return false;
}

bool xu_control_query(
    const device &device, const xu &xu, uint8_t selector, xu_query query,
    uint16_t size, uint8_t *data) {
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
  UNUSED(device)
  UNUSED(width)
  UNUSED(height)
  UNUSED(fourcc)
  UNUSED(fps)
  UNUSED(callback)
}

void start_streaming(device &device, int num_transfer_bufs) {  // NOLINT
  UNUSED(device)
  UNUSED(num_transfer_bufs)
}

void stop_streaming(device &device) {  // NOLINT
  UNUSED(device)
}

}  // namespace uvc

MYNTEYE_END_NAMESPACE
