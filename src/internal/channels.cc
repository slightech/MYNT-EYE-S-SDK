#include "internal/channels.h"

#include <glog/logging.h>

#include <stdexcept>

MYNTEYE_BEGIN_NAMESPACE

namespace {

int XuCamCtrlId(Option option) {
  switch (option) {
    case Option::EXPOSURE_MODE:
      return 0;
      break;
    case Option::MAX_GAIN:
      return 1;
      break;
    case Option::MAX_EXPOSURE_TIME:
      return 2;
      break;
    case Option::DESIRED_BRIGHTNESS:
      return 3;
      break;
    case Option::IMU_FREQUENCY:
      return 4;
      break;
    case Option::IR_CONTROL:
      return 5;
      break;
    case Option::HDR_MODE:
      return 6;
      break;
    case Option::FRAME_RATE:
      return 7;
      break;
    default:
      LOG(FATAL) << "No cam ctrl id for " << option;
  }
}

int XuHalfDuplexId(Option option) {
  switch (option) {
    case Option::ZERO_DRIFT_CALIBRATION:
      return 0;
      break;
    case Option::ERASE_CHIP:
      return 1;
      break;
    default:
      LOG(FATAL) << "No half duplex id for " << option;
  }
}

}  // namespace

Channels::Channels(std::shared_ptr<uvc::device> device) : device_(device) {
  VLOG(2) << __func__;
  UpdateControlInfos();
}

Channels::~Channels() {
  VLOG(2) << __func__;
}

void Channels::LogControlInfos() const {
  for (auto &&it = control_infos_.begin(); it != control_infos_.end(); it++) {
    LOG(INFO) << it->first << ": min=" << it->second.min
              << ", max=" << it->second.max << ", def=" << it->second.def
              << ", cur=" << GetControlValue(it->first);
  }
}

void Channels::UpdateControlInfos() {
  for (auto &&option : std::vector<Option>{Option::GAIN, Option::BRIGHTNESS,
                                           Option::CONTRAST}) {
    control_infos_[option] = PuControlInfo(option);
  }

  for (auto &&option : std::vector<Option>{
           Option::FRAME_RATE, Option::IMU_FREQUENCY, Option::EXPOSURE_MODE,
           Option::MAX_GAIN, Option::MAX_EXPOSURE_TIME,
           Option::DESIRED_BRIGHTNESS, Option::IR_CONTROL, Option::HDR_MODE}) {
    control_infos_[option] = XuControlInfo(option);
  }

  if (VLOG_IS_ON(2)) {
    for (auto &&it = control_infos_.begin(); it != control_infos_.end(); it++) {
      VLOG(2) << it->first << ": min=" << it->second.min
              << ", max=" << it->second.max << ", def=" << it->second.def
              << ", cur=" << GetControlValue(it->first);
    }
  }
}

Channels::control_info_t Channels::GetControlInfo(const Option &option) const {
  try {
    return control_infos_.at(option);
  } catch (const std::out_of_range &e) {
    LOG(WARNING) << "Get control info of " << option << " failed";
    return {0, 0, 0};
  }
}

std::int32_t Channels::GetControlValue(const Option &option) const {
  switch (option) {
    case Option::GAIN:
    case Option::BRIGHTNESS:
    case Option::CONTRAST:
      std::int32_t value;
      if (!PuControlQuery(option, uvc::PU_QUERY_GET, &value)) {
        LOG(WARNING) << option << " get value failed";
        return -1;
      } else {
        return value;
      }
    case Option::FRAME_RATE:
    case Option::IMU_FREQUENCY:
    case Option::EXPOSURE_MODE:
    case Option::MAX_GAIN:
    case Option::MAX_EXPOSURE_TIME:
    case Option::DESIRED_BRIGHTNESS:
    case Option::IR_CONTROL:
    case Option::HDR_MODE:
      return XuCamCtrlGet(option);
    case Option::ZERO_DRIFT_CALIBRATION:
    case Option::ERASE_CHIP:
      LOG(WARNING) << option << " get value useless";
      return -1;
    default:
      LOG(FATAL) << "Unsupported option " << option;
  }
  return -1;
}

void Channels::SetControlValue(const Option &option, std::int32_t value) {
  auto in_range = [this, &option, &value]() {
    auto &&info = GetControlInfo(option);
    if (value < info.min || value > info.max) {
      LOG(WARNING) << option << " set value out of range, " << value
                   << " not in [" << info.min << "," << info.max << "]";
      return false;
    }
    return true;
  };
  switch (option) {
    case Option::GAIN:
    case Option::BRIGHTNESS:
    case Option::CONTRAST: {
      if (!in_range())
        break;
      if (!PuControlQuery(option, uvc::PU_QUERY_SET, &value)) {
        LOG(WARNING) << option << " set value failed";
      }
    } break;
    case Option::FRAME_RATE:
    case Option::IMU_FREQUENCY:
    case Option::EXPOSURE_MODE:
    case Option::MAX_GAIN:
    case Option::MAX_EXPOSURE_TIME:
    case Option::DESIRED_BRIGHTNESS:
    case Option::IR_CONTROL:
    case Option::HDR_MODE: {
      if (!in_range())
        break;
      XuCamCtrlSet(option, value);
    } break;
    case Option::ZERO_DRIFT_CALIBRATION:
    case Option::ERASE_CHIP:
      LOG(WARNING) << option << " set value useless";
      break;
    default:
      LOG(FATAL) << "Unsupported option " << option;
  }
}

bool Channels::RunControlAction(const Option &option) const {
  switch (option) {
    case Option::ZERO_DRIFT_CALIBRATION:
      return XuHalfDuplexSet(option, XU_CMD_ZDC);
    case Option::ERASE_CHIP:
      return XuHalfDuplexSet(option, XU_CMD_ERASE);
    case Option::GAIN:
    case Option::BRIGHTNESS:
    case Option::CONTRAST:
    case Option::FRAME_RATE:
    case Option::IMU_FREQUENCY:
    case Option::EXPOSURE_MODE:
    case Option::MAX_GAIN:
    case Option::MAX_EXPOSURE_TIME:
    case Option::DESIRED_BRIGHTNESS:
    case Option::IR_CONTROL:
    case Option::HDR_MODE:
      LOG(WARNING) << option << " run action useless";
      return false;
    default:
      LOG(FATAL) << "Unsupported option " << option;
  }
}

bool Channels::PuControlRange(
    Option option, int32_t *min, int32_t *max, int32_t *def) const {
  CHECK_NOTNULL(device_);
  return uvc::pu_control_range(*device_, option, min, max, def);
}

bool Channels::PuControlQuery(
    Option option, uvc::pu_query query, int32_t *value) const {
  CHECK_NOTNULL(device_);
  return uvc::pu_control_query(*device_, option, query, value);
}

bool Channels::XuControlQuery(
    channel_t channel, uvc::xu_query query, uint16_t size,
    uint8_t *data) const {
  return XuControlQuery({3}, channel >> 8, query, size, data);
}

bool Channels::XuControlQuery(
    const uvc::xu &xu, uint8_t selector, uvc::xu_query query, uint16_t size,
    uint8_t *data) const {
  CHECK_NOTNULL(device_);
  return uvc::xu_control_query(*device_, xu, selector, query, size, data);
}

bool Channels::XuCamCtrlQuery(
    uvc::xu_query query, uint16_t size, uint8_t *data) const {
  return XuControlQuery(CHANNEL_CAM_CTRL, query, size, data);
}

std::int32_t Channels::XuCamCtrlGet(Option option) const {
  int id = XuCamCtrlId(option);

  std::uint8_t data[3] = {static_cast<std::uint8_t>((id | 0x80) & 0xFF), 0, 0};
  if (!XuCamCtrlQuery(uvc::XU_QUERY_SET, 3, data)) {
    LOG(WARNING) << "XuCamCtrlGet value of " << option << " failed";
    return -1;
  }

  data[0] = id & 0xFF;
  if (XuCamCtrlQuery(uvc::XU_QUERY_GET, 3, data)) {
    return (data[1] << 8) + (data[2]);
  } else {
    LOG(WARNING) << "XuCamCtrlGet value of " << option << " failed";
    return -1;
  }
}

void Channels::XuCamCtrlSet(Option option, std::int32_t value) const {
  int id = XuCamCtrlId(option);
  std::uint8_t data[3] = {static_cast<std::uint8_t>(id & 0xFF),
                          static_cast<std::uint8_t>((value >> 8) & 0xFF),
                          static_cast<std::uint8_t>(value & 0xFF)};
  if (!XuCamCtrlQuery(uvc::XU_QUERY_SET, 3, data)) {
    LOG(WARNING) << "XuCamCtrlSet value (" << value << ") of " << option
                 << " failed";
  } else {
    VLOG(2) << "XuCamCtrlSet value (" << value << ") of " << option
            << " success";
  }
}

bool Channels::XuHalfDuplexSet(Option option, xu_cmd_t cmd) const {
  int id = XuHalfDuplexId(option);
  std::uint8_t data[3] = {// must be 3 now
                          static_cast<std::uint8_t>(id & 0xFF), cmd};
  if (!XuControlQuery(CHANNEL_HALF_DUPLEX, uvc::XU_QUERY_SET, 3, data)) {
    LOG(WARNING) << "XuHalfDuplexSet value (0x" << std::hex << std::uppercase
                 << cmd << ") of " << option << " failed";
    return false;
  } else {
    VLOG(2) << "XuHalfDuplexSet value (0x" << std::hex << std::uppercase << cmd
            << ") of " << option << " success";
    return true;
  }
}

Channels::control_info_t Channels::PuControlInfo(Option option) const {
  int32_t min = 0, max = 0, def = 0;
  if (!PuControlRange(option, &min, &max, &def)) {
    LOG(WARNING) << "Get PuControlInfo of " << option << " failed";
  }
  return {min, max, def};
}

Channels::control_info_t Channels::XuControlInfo(Option option) const {
  int id = XuCamCtrlId(option);

  std::uint8_t data[3] = {static_cast<std::uint8_t>((id | 0x80) & 0xFF), 0, 0};
  if (!XuCamCtrlQuery(uvc::XU_QUERY_SET, 3, data)) {
    LOG(WARNING) << "Get XuControlInfo of " << option << " failed";
    return {0, 0, 0};
  }

  control_info_t info{0, 0, 0};

  data[0] = id & 0xFF;
  if (XuCamCtrlQuery(uvc::XU_QUERY_MIN, 3, data)) {
    info.min = (data[1] << 8) + (data[2]);
  } else {
    LOG(WARNING) << "Get XuControlInfo.min of " << option << " failed";
  }
  if (XuCamCtrlQuery(uvc::XU_QUERY_MAX, 3, data)) {
    info.max = (data[1] << 8) + (data[2]);
  } else {
    LOG(WARNING) << "Get XuControlInfo.max of " << option << " failed";
  }
  if (XuCamCtrlQuery(uvc::XU_QUERY_DEF, 3, data)) {
    info.def = (data[1] << 8) + (data[2]);
  } else {
    LOG(WARNING) << "Get XuControlInfo.def of " << option << " failed";
  }

  return info;
}

MYNTEYE_END_NAMESPACE
