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

}  // namespace uvc

MYNTEYE_END_NAMESPACE
