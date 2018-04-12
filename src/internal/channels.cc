#include "internal/channels.h"

#include <glog/logging.h>

#include <chrono>
#include <iomanip>
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

Channels::Channels(std::shared_ptr<uvc::device> device)
    : device_(device),
      is_imu_tracking_(false),
      imu_track_stop_(false),
      imu_sn_(0),
      imu_callback_(nullptr) {
  VLOG(2) << __func__;
  UpdateControlInfos();
}

Channels::~Channels() {
  VLOG(2) << __func__;
  StopImuTracking();
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
      if (PuControlQuery(option, uvc::PU_QUERY_GET, &value)) {
        return value;
      } else {
        LOG(WARNING) << option << " get value failed";
        return -1;
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

void Channels::SetImuCallback(imu_callback_t callback) {
  imu_callback_ = callback;
}

void Channels::StartImuTracking(imu_callback_t callback) {
  if (is_imu_tracking_) {
    LOG(WARNING) << "Start imu tracking failed, is tracking already";
    return;
  }
  if (callback) {
    imu_callback_ = callback;
  }
  is_imu_tracking_ = true;
  imu_track_thread_ = std::thread([this]() {
    imu_sn_ = 0;
    ImuReqPacket req_packet{imu_sn_};
    ImuResPacket res_packet;
    // auto sleep_milli = [](std::intmax_t n) {
    //   std::this_thread::sleep_for(std::chrono::milliseconds(n));
    // };
    while (!imu_track_stop_) {
      req_packet.serial_number = imu_sn_;
      if (!XuImuWrite(req_packet)) {
        continue;
      }

      if (!XuImuRead(&res_packet)) {
        continue;
      }

      if (res_packet.packets.size() == 0) {
        continue;
      }

      VLOG(2) << "Imu req sn: " << imu_sn_
              << ", res count: " << [&res_packet]() {
                   std::size_t n = 0;
                   for (auto &&packet : res_packet.packets) {
                     n += packet.count;
                   }
                   return n;
                 }();

      auto &&sn = res_packet.packets.back().serial_number;
      if (imu_sn_ == sn) {
        VLOG(2) << "New imu not ready, dropped";
        continue;
      }
      imu_sn_ = sn;

      if (imu_callback_) {
        for (auto &&packet : res_packet.packets) {
          imu_callback_(packet);
        }
      }

      res_packet.packets.clear();
    }
  });
}

void Channels::StopImuTracking() {
  if (!is_imu_tracking_) {
    return;
  }
  if (imu_track_thread_.joinable()) {
    imu_track_stop_ = true;
    imu_track_thread_.join();
    imu_track_stop_ = false;
    is_imu_tracking_ = false;
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
  if (XuCamCtrlQuery(uvc::XU_QUERY_SET, 3, data)) {
    VLOG(2) << "XuCamCtrlSet value (" << value << ") of " << option
            << " success";
  } else {
    LOG(WARNING) << "XuCamCtrlSet value (" << value << ") of " << option
                 << " failed";
  }
}

bool Channels::XuHalfDuplexSet(Option option, xu_cmd_t cmd) const {
  int id = XuHalfDuplexId(option);
  std::uint8_t data[20] = {static_cast<std::uint8_t>(id & 0xFF),
                           static_cast<std::uint8_t>(cmd)};
  if (XuControlQuery(CHANNEL_HALF_DUPLEX, uvc::XU_QUERY_SET, 20, data)) {
    VLOG(2) << "XuHalfDuplexSet value (0x" << std::hex << std::uppercase << cmd
            << ") of " << option << " success";
    return true;
  } else {
    LOG(WARNING) << "XuHalfDuplexSet value (0x" << std::hex << std::uppercase
                 << cmd << ") of " << option << " failed";
    return false;
  }
}

bool Channels::XuImuWrite(const ImuReqPacket &req) const {
  auto &&data = req.to_data();
  if (XuControlQuery(
          CHANNEL_IMU_WRITE, uvc::XU_QUERY_SET, data.size(), data.data())) {
    VLOG(2) << "XuImuWrite request success";
    return true;
  } else {
    LOG(WARNING) << "XuImuWrite request failed";
    return false;
  }
}

bool Channels::XuImuRead(ImuResPacket *res) const {
  static std::uint8_t data[2000]{};
  // std::fill(data, data + 2000, 0);  // reset
  if (XuControlQuery(CHANNEL_IMU_READ, uvc::XU_QUERY_GET, 2000, data)) {
    res->from_data(data);

    if (res->header != 0x5B) {
      LOG(WARNING) << "Imu response packet header must be 0x5B, but 0x"
                   << std::hex << std::uppercase << std::setw(2)
                   << std::setfill('0') << static_cast<int>(res->header)
                   << " now";
      return false;
    }

    if (res->state != 0) {
      LOG(WARNING) << "Imu response packet state must be 0, but " << res->state
                   << " now";
      return false;
    }

    std::uint8_t checksum = 0;
    for (std::size_t i = 4, n = 4 + res->size; i < n; i++) {
      checksum = (checksum ^ data[i]);
    }
    if (checksum != res->checksum) {
      LOG(WARNING) << "Imu response packet checksum should be 0x" << std::hex
                   << std::uppercase << std::setw(2) << std::setfill('0')
                   << static_cast<int>(checksum) << ", but 0x" << std::setw(2)
                   << std::setfill('0') << static_cast<int>(res->checksum)
                   << " now";
      return false;
    }

    VLOG(2) << "XuImuRead response success";
    return true;
  } else {
    LOG(WARNING) << "XuImuRead response failed";
    return false;
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
