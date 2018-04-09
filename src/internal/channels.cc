#include "internal/channels.h"

#include <glog/logging.h>

MYNTEYE_BEGIN_NAMESPACE

Channels::Channels(std::shared_ptr<uvc::device> device) : device_(device) {
  VLOG(2) << __func__;
}

Channels::~Channels() {
  VLOG(2) << __func__;
}

bool Channels::XuControlQuery(
    uint8_t selector, uvc::xu_query query, uint16_t size, uint8_t *data) {
  return XuControlQuery({3}, selector, query, size, data);
}

bool Channels::XuControlQuery(
    const uvc::xu &xu, uint8_t selector, uvc::xu_query query, uint16_t size,
    uint8_t *data) {
  CHECK_NOTNULL(device_);
  return uvc::xu_control_query(*device_, xu, selector, query, size, data);
}

MYNTEYE_END_NAMESPACE
