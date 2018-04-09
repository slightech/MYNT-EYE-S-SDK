#include "internal/channels.h"

#include <glog/logging.h>

#include "uvc/uvc.h"

MYNTEYE_BEGIN_NAMESPACE

Channels::Channels(std::shared_ptr<uvc::device> device) : device_(device) {
  VLOG(2) << __func__;
}

Channels::~Channels() {
  VLOG(2) << __func__;
}

bool Channels::ControlQuery(
    const uvc::xu &xu, uint8_t selector, const query_t &query, uint16_t size,
    uint8_t *data) {
  CHECK_NOTNULL(device_);
  uvc::xu_query code;
  switch (query) {
    case SET_CUR:
      code = uvc::XU_SET_CUR;
      break;
    case GET_CUR:
      code = uvc::XU_GET_CUR;
      break;
    case GET_MIN:
      code = uvc::XU_GET_MIN;
      break;
    case GET_MAX:
      code = uvc::XU_GET_MAX;
      break;
    case GET_DEF:
      code = uvc::XU_GET_DEF;
      break;
    default:
      LOG(FATAL) << "ControlQuery query code is unaccepted";
  }
  return uvc::xu_control_query(*device_, xu, selector, code, size, data);
}

MYNTEYE_END_NAMESPACE
