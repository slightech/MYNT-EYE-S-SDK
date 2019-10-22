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
#include "mynteye/device/channel/channels.h"

#include <bitset>
#include <chrono>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "mynteye/device/config.h"
#include "mynteye/logger.h"
#include "mynteye/util/times.h"

#define IMU_TRACK_PERIOD 25  // ms

// for time limmit debug
#define DEBUG_TIME_LIMIT true

MYNTEYE_BEGIN_NAMESPACE

mynteye::ImuPacket2 to_pak2(const mynteye::ImuPacket& pak1,
    const int &accel_range,
    const int &gyro_range) {
  mynteye::ImuPacket2 res;
  res.version = pak1.version;
  res.count = pak1.count;
  res.serial_number = pak1.serial_number;
  for (size_t i = 0; i < pak1.segments.size(); i++) {
    mynteye::ImuSegment2 tpr;
    tpr.accel[0] = pak1.segments[i].accel[0] * 1.f * accel_range / 0x10000;
    tpr.accel[1] = pak1.segments[i].accel[1] * 1.f * accel_range / 0x10000;
    tpr.accel[2] = pak1.segments[i].accel[2] * 1.f * accel_range / 0x10000;
    tpr.gyro[0] = pak1.segments[i].gyro[0] * gyro_range / 0x10000;
    tpr.gyro[1] = pak1.segments[i].gyro[1] * gyro_range / 0x10000;
    tpr.gyro[2] = pak1.segments[i].gyro[2] * gyro_range / 0x10000;
    tpr.flag = pak1.segments[i].flag;
    tpr.frame_id = pak1.segments[i].frame_id;
    tpr.is_ets = pak1.segments[i].is_ets;
    tpr.temperature = pak1.segments[i].temperature / 326.8f + 25;
    tpr.timestamp = pak1.segments[i].timestamp;
    res.segments.push_back(tpr);
  }
  return res;
}


namespace {

const uvc::xu mynteye_xu = {3, 2,
  {
    0x947a6d9f, 0x8a2f, 0x418d,
    {0x85, 0x9e, 0x6c, 0x9a, 0xa0, 0x38, 0x10, 0x14}
  }
};

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
    case Option::MIN_EXPOSURE_TIME:
      return 8;
      break;
    case Option::ACCELEROMETER_RANGE:
      return 9;
      break;
    case Option::GYROSCOPE_RANGE:
      return 10;
      break;
    case Option::ACCELEROMETER_LOW_PASS_FILTER:
      return 11;
      break;
    case Option::GYROSCOPE_LOW_PASS_FILTER:
      return 12;
      break;
    case Option::IIC_ADDRESS_SETTING:
      return 13;
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
    case Option::SYNC_TIMESTAMP:
      return 3;
      break;
    default:
      LOG(FATAL) << "No half duplex id for " << option;
  }
}

}  // namespace

Channels::Channels(const std::shared_ptr<uvc::device> &device,
    const std::shared_ptr<ChannelsAdapter> &adapter)
  : device_(device),
    adapter_(adapter),
    is_imu_tracking_(false),
    is_imu_proto2_(false),
    is_s2_(false),
    enable_imu_correspondence(false),
    imu_track_stop_(false),
    imu_sn_(0),
    imu_callback_(nullptr),
    dev_info_(nullptr),
    timestamp_compensate_(0),
    is_nearly_before_timestamp_limmit_(0),
    current_datum_(0) {
  VLOG(2) << __func__;
  UpdateControlInfos();
  accel_range = GetControlValue(Option::ACCELEROMETER_RANGE);
  if (accel_range == -1)
    accel_range = GetAccelRangeDefault();

  gyro_range = GetControlValue(Option::GYROSCOPE_RANGE);
  if (gyro_range == -1)
    gyro_range = GetGyroRangeDefault();
}

Channels::~Channels() {
  VLOG(2) << __func__;
  StopImuTracking();
}

std::int32_t Channels::GetAccelRangeDefault() {
  return adapter_->GetAccelRangeDefault();
}

std::int32_t Channels::GetGyroRangeDefault() {
  return adapter_->GetGyroRangeDefault();
}

void Channels::LogControlInfos() const {
  for (auto &&it = control_infos_.begin(); it != control_infos_.end(); it++) {
    LOG(INFO) << it->first << ": min=" << it->second.min
              << ", max=" << it->second.max << ", def=" << it->second.def
              << ", cur=" << GetControlValue(it->first);
  }
}

void Channels::UpdateControlInfos() {
  auto &&supports = adapter_->GetOptionSupports();
  for (auto &&option : std::vector<Option>{
      Option::GAIN, Option::BRIGHTNESS, Option::CONTRAST}) {
    if (supports.find(option) != supports.end())
      control_infos_[option] = PuControlInfo(option);
  }

  for (auto &&option : std::vector<Option>{
      Option::FRAME_RATE, Option::IMU_FREQUENCY,
      Option::EXPOSURE_MODE, Option::MAX_GAIN,
      Option::MAX_EXPOSURE_TIME, Option::MIN_EXPOSURE_TIME,
      Option::DESIRED_BRIGHTNESS, Option::IR_CONTROL,
      Option::HDR_MODE, Option::ACCELEROMETER_RANGE,
      Option::GYROSCOPE_RANGE, Option::ACCELEROMETER_LOW_PASS_FILTER,
      Option::GYROSCOPE_LOW_PASS_FILTER,
      Option::IIC_ADDRESS_SETTING}) {
    if (supports.find(option) != supports.end())
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
    case Option::MIN_EXPOSURE_TIME:
    case Option::ACCELEROMETER_RANGE:
    case Option::GYROSCOPE_RANGE:
    case Option::ACCELEROMETER_LOW_PASS_FILTER:
    case Option::GYROSCOPE_LOW_PASS_FILTER:
    case Option::IIC_ADDRESS_SETTING:
      return XuCamCtrlGet(option);
    case Option::ZERO_DRIFT_CALIBRATION:
    case Option::ERASE_CHIP:
      LOG(WARNING) << option << " get value useless";
      return -1;
    case Option::SYNC_TIMESTAMP:
      LOG(WARNING) << option << " get value useless";
      return -1;
    default:
      LOG(ERROR) << "Unsupported option " << option;
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
  auto in_values = [&option, &value](std::vector<std::int32_t> values) {
    if (std::find(values.begin(), values.end(), value) != values.end()) {
      return true;
    } else {
      std::ostringstream ss;
      std::copy(
          values.begin(), values.end(),
          std::ostream_iterator<std::int32_t>(ss, ","));
      LOG(WARNING) << option << " set value invalid, must in [" << ss.str()
                   << "]";
      return false;
    }
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
    case Option::FRAME_RATE: {
      if (!in_range() ||
          !in_values({10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60}))
        break;
      XuCamCtrlSet(option, value);
    } break;
    case Option::IMU_FREQUENCY: {
      if (!in_range() || !in_values({100, 200, 250, 333, 500}))
        break;
      XuCamCtrlSet(option, value);
    } break;
    case Option::ACCELEROMETER_RANGE: {
      if (!in_range() || !in_values(adapter_->GetAccelRangeValues()))
        break;
      XuCamCtrlSet(option, value);
    } break;
    case Option::GYROSCOPE_RANGE: {
      if (!in_range() || !in_values(adapter_->GetGyroRangeValues()))
        break;
      XuCamCtrlSet(option, value);
    } break;
    case Option::ACCELEROMETER_LOW_PASS_FILTER: {
      if (!in_range() || !in_values({0, 1, 2}))
        break;
      XuCamCtrlSet(option, value);
    } break;
    case Option::GYROSCOPE_LOW_PASS_FILTER: {
      if (!in_range() || !in_values({23, 64}))
        break;
      XuCamCtrlSet(option, value);
    } break;
    case Option::EXPOSURE_MODE:
    case Option::MAX_GAIN:
    case Option::MAX_EXPOSURE_TIME:
    case Option::DESIRED_BRIGHTNESS:
    case Option::IR_CONTROL:
    case Option::HDR_MODE:
    case Option::MIN_EXPOSURE_TIME:
    case Option::IIC_ADDRESS_SETTING: {
      if (!in_range())
        break;
      XuCamCtrlSet(option, value);
    } break;
    case Option::ZERO_DRIFT_CALIBRATION:
    case Option::ERASE_CHIP:
      LOG(WARNING) << option << " set value useless";
      break;
    case Option::SYNC_TIMESTAMP:
      XuCamCtrlSet(option, value);
      break;
    default:
      LOG(ERROR) << "Unsupported option " << option;
  }
}

bool Channels::SetControlValue(const Option &option, std::uint64_t value) {
  switch (option) {
    case Option::GAIN:
    case Option::BRIGHTNESS:
    case Option::CONTRAST:
    case Option::FRAME_RATE:
    case Option::IMU_FREQUENCY:
    case Option::ACCELEROMETER_RANGE:
    case Option::GYROSCOPE_RANGE:
    case Option::ACCELEROMETER_LOW_PASS_FILTER:
    case Option::GYROSCOPE_LOW_PASS_FILTER:
    case Option::EXPOSURE_MODE:
    case Option::MAX_GAIN:
    case Option::MAX_EXPOSURE_TIME:
    case Option::DESIRED_BRIGHTNESS:
    case Option::IR_CONTROL:
    case Option::HDR_MODE:
    case Option::MIN_EXPOSURE_TIME:
    case Option::IIC_ADDRESS_SETTING:
    case Option::ZERO_DRIFT_CALIBRATION:
      LOG(WARNING) << option << " refer to function SetControlValue(const Option &option, std::int32_t value)";  // NOLINT
      break;
    case Option::ERASE_CHIP:
      LOG(WARNING) << option << " set value useless";
      break;
    case Option::SYNC_TIMESTAMP:
      return XuHalfDuplexSet(option, value);
      break;
    default:
      LOG(ERROR) << "Unsupported option " << option;
  }

  return false;
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
    case Option::MIN_EXPOSURE_TIME:
    case Option::ACCELEROMETER_RANGE:
    case Option::GYROSCOPE_RANGE:
    case Option::ACCELEROMETER_LOW_PASS_FILTER:
    case Option::GYROSCOPE_LOW_PASS_FILTER:
    case Option::IIC_ADDRESS_SETTING:
      LOG(WARNING) << option << " run action useless";
      return false;
    default:
      LOG(ERROR) << "Unsupported option " << option;
      return false;
  }
}

void Channels::SetImuCallback(imu_callback_t callback) {
  imu_callback_ = callback;
}

void Channels::DoImuTrack() {
  if (IsImuProtocol2()) {
    return DoImuTrack2();
  } else {
    if (IsS2()) {
      return DoImuTrack1();
    } else {
      return DoImuTrack1WithTimeLimmitFix();
    }
  }
}

void Channels::DoImuTrack1() {
  static ImuReqPacket req_packet{0};
  static ImuResPacket res_packet;

  req_packet.serial_number = imu_sn_;
  if (!XuImuWrite(req_packet)) {
    return;
  }

  if (!XuImuRead(&res_packet)) {
    return;
  }

  if (res_packet.packets.size() == 0) {
    return;
  }

  if (res_packet.packets.back().count == 0) {
    return;
  }

  VLOG(2) << "Imu req sn: " << imu_sn_ << ", res count: " << []() {
    std::size_t n = 0;
    for (auto &&packet : res_packet.packets) {
      n += packet.count;
    }
    return n;
  }();

  auto &&sn = res_packet.packets.back().serial_number;
  if (imu_sn_ == sn) {
    VLOG(2) << "New imu not ready, dropped";
    return;
  }
  imu_sn_ = sn;

  if (imu_callback_) {
    for (auto &packet : res_packet.packets) {
      imu_callback_(to_pak2(packet, accel_range, gyro_range));
    }
  }
  res_packet.packets.clear();
}

void Channels::DoImuTrack1WithTimeLimmitFix() {
  static ImuReqPacket req_packet{0};
  static ImuResPacket res_packet;

  req_packet.serial_number = imu_sn_;
  if (!XuImuWrite(req_packet)) {
    return;
  }

  if (!XuImuRead(&res_packet)) {
    return;
  }

  if (res_packet.packets.size() == 0) {
    return;
  }

  if (res_packet.packets.back().count == 0) {
    return;
  }

  VLOG(2) << "Imu req sn: " << imu_sn_ << ", res count: " << []() {
    std::size_t n = 0;
    for (auto &&packet : res_packet.packets) {
      n += packet.count;
    }
    return n;
  }();

  auto &&sn = res_packet.packets.back().serial_number;
  if (imu_sn_ == sn) {
    VLOG(2) << "New imu not ready, dropped";
    return;
  }
  imu_sn_ = sn;

  if (imu_callback_) {
    for (auto &packet : res_packet.packets) {
      auto pak2_tmp = to_pak2(packet, accel_range, gyro_range);
      CheckTimeStampLimmit(pak2_tmp);
      imu_callback_(pak2_tmp);
    }
  }
  res_packet.packets.clear();
}

void Channels::CheckTimeStampLimmit(mynteye::ImuPacket2 &packet) {
  for (auto &segment2 : packet.segments) {
    segment2.timestamp += (timestamp_compensate_ * 42949672960);
    // the timestamp nearly 8 imu frame before limmit
    if ((segment2.timestamp % 42949672960) > 42949606770 &&
        is_nearly_before_timestamp_limmit_ == 0) {
      current_datum_ = segment2.timestamp;
      timestamp_compensate_++;
      is_nearly_before_timestamp_limmit_ = LIMMIT_CHECK_DORMANCY_THRESHOLD;
    }
    if (is_nearly_before_timestamp_limmit_ > 0) {
      is_nearly_before_timestamp_limmit_--;
      uint64_t abs_tmp = 0;
      if (current_datum_ > segment2.timestamp) {
        abs_tmp = current_datum_ - segment2.timestamp;
      } else {
        abs_tmp = segment2.timestamp - current_datum_;
      }
      if (abs_tmp > (uint64_t)(42949672960/2)) {  // NOLINT
        segment2.timestamp -= 42949672960;
      }
    }
  }
}

void Channels::DoImuTrack2() {
  // LOG(INFO) << "wait to adapter!";
  static ImuReqPacket2 req_packet{0x5A, imu_sn_, enable_imu_correspondence};
  static ImuResPacket2 res_packet;
  if (!XuImuWrite(req_packet)) {
    return;
  }
  if (!XuImuRead(&res_packet)) {
    return;
  }
  if (res_packet.packets.size() == 0) {
    return;
  }
  if (res_packet.packets.back().count == 0) {
    return;
  }
  VLOG(2) << "Imu req sn: " << imu_sn_ << ", res count: " << []() {
    std::size_t n = 0;
    for (auto &&packet : res_packet.packets) {
      n += packet.count;
    }
    return n;
  }();
  auto &&sn = res_packet.packets.back().serial_number;
  if (imu_sn_ == sn) {
    VLOG(2) << "New imu not ready, dropped";
    return;
  }
  imu_sn_ = sn;
  if (imu_callback_) {
    for (auto &packet : res_packet.packets) {
      imu_callback_(packet);
    }
  }

  res_packet.packets.clear();
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
    auto sleep = [](const times::system_clock::time_point &time_beg) {
      auto &&time_elapsed_ms =
          times::count<times::milliseconds>(times::now() - time_beg);
      if (time_elapsed_ms < IMU_TRACK_PERIOD) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(IMU_TRACK_PERIOD - time_elapsed_ms));
        VLOG(2) << "Imu track cost " << time_elapsed_ms << " ms"
                << ", sleep " << (IMU_TRACK_PERIOD - time_elapsed_ms) << " ms";
      }
    };
    if (IsImuProtocol2()) {
      while (!imu_track_stop_) {
        auto &&time_beg = times::now();
        DoImuTrack2();
        sleep(time_beg);
      }
    } else {
      if (IsS2()) {
        while (!imu_track_stop_) {
          auto &&time_beg = times::now();
          DoImuTrack1();
          sleep(time_beg);
        }
      } else {
        while (!imu_track_stop_) {
          auto &&time_beg = times::now();
          DoImuTrack1WithTimeLimmitFix();
          sleep(time_beg);
        }
      }
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

bool Channels::GetFiles(
    device_info_t *info, img_params_t *img_params, imu_params_t *imu_params) {
  if (info == nullptr && img_params == nullptr && imu_params == nullptr) {
    LOG(WARNING) << "Files are not provided to get";
    return false;
  }

  std::uint8_t data[2000]{};

  std::bitset<8> header;
  header[7] = 0;  // get

  header[0] = (info != nullptr);
  header[1] = (img_params != nullptr);
  header[2] = (imu_params != nullptr);

  data[0] = static_cast<std::uint8_t>(header.to_ulong());
  VLOG(2) << "GetFiles header: 0x" << std::hex << std::uppercase << std::setw(2)
          << std::setfill('0') << static_cast<int>(data[0]);
  if (!XuFileQuery(uvc::XU_QUERY_SET, 2000, data)) {
    LOG(WARNING) << "GetFiles failed";
    return false;
  }

  if (XuFileQuery(uvc::XU_QUERY_GET, 2000, data)) {
    // header = std::bitset<8>(data[0]);
    std::uint16_t size = bytes::_from_data<std::uint16_t>(data + 1);
    std::uint8_t checksum = data[3 + size];
    VLOG(2) << "GetFiles data size: " << size << ", checksum: 0x" << std::hex
            << std::setw(2) << std::setfill('0') << static_cast<int>(checksum);

    std::uint8_t checksum_now = 0;
    for (std::size_t i = 3, n = 3 + size; i < n; i++) {
      checksum_now = (checksum_now ^ data[i]);
    }
    if (checksum != checksum_now) {
      LOG(WARNING) << "Files checksum should be 0x" << std::hex
                   << std::uppercase << std::setw(2) << std::setfill('0')
                   << static_cast<int>(checksum) << ", but 0x" << std::setw(2)
                   << std::setfill('0') << static_cast<int>(checksum_now)
                   << " now";
      return false;
    }

    std::size_t i = 3;
    std::size_t end = 3 + size;
    while (i < end) {
      std::uint8_t file_id = *(data + i);
      std::uint16_t file_size = bytes::_from_data<std::uint16_t>(data + i + 1);
      VLOG(2) << "GetFiles id: " << static_cast<int>(file_id)
              << ", size: " << file_size;
      i += 3;
      switch (file_id) {
        case FID_DEVICE_INFO: {
          auto &&n = file_channel_.GetDeviceInfoFromData(
              data + i, file_size, info);
          CHECK_EQ(n, file_size)
              << "The firmware not support getting device info, you could "
                 "upgrade to latest";
          dev_info_ = std::make_shared<DeviceInfo>(*info);
          is_imu_proto2_ = dev_info_ &&
              dev_info_->spec_version >= Version(1, 3) &&
              strstr(dev_info_->name.c_str(), "S2") != nullptr;
          is_s2_ = strstr(dev_info_->name.c_str(), "S2") != nullptr;
        } break;
        case FID_IMG_PARAMS: {
          if (file_size > 0) {
            auto &&n = file_channel_.GetImgParamsFromData(
                data + i, file_size, img_params);
            CHECK_EQ(n, file_size)
              << "The firmware not support getting device info, you could "
                 "upgrade to latest";
          }
        } break;
        case FID_IMU_PARAMS: {
          imu_params->ok = file_size > 0;
          if (imu_params->ok) {
            auto &&n = file_channel_.GetImuParamsFromData(
                data + i, file_size, imu_params);
            CHECK_EQ(n, file_size)
              << "The firmware not support getting device info, you could "
                 "upgrade to latest";
          }
        } break;
        default:
          LOG(FATAL) << "Unsupported file id: " << file_id;
      }
      i += file_size;
    }

    VLOG(2) << "GetFiles success";
    return true;
  } else {
    LOG(WARNING) << "GetFiles failed";
    return false;
  }
}

bool Channels::SetFiles(
    device_info_t *info, img_params_t *img_params, imu_params_t *imu_params) {
  if (info == nullptr && img_params == nullptr && imu_params == nullptr) {
    LOG(WARNING) << "Files are not provided to set";
    return false;
  }

  std::uint8_t data[2000]{};

  std::bitset<8> header;
  header[7] = 1;  // set

  std::uint16_t size = 0;
  if (info != nullptr) {
    auto n = file_channel_.SetDeviceInfoToData(info, data + 3 + size);
    if (n > 0) {
      header[0] = true;
      size += n;
    }
  }
  if (img_params != nullptr) {
    // remove not supported resolution
    auto&& res = adapter_->GetResolutionSupports();
    for (auto it = img_params->begin(); it != img_params->end(); ) {
      if (res.find(it->first) == res.end()) {
        LOG(WARNING) << "Image params of resolution "
            << it->first.width << "x" << it->first.height << " not supported";
        it = img_params->erase(it);
      } else {
        ++it;
      }
    }

    if (img_params->empty()) {
      std::ostringstream os;
      os << "Image params resolution must be ";
      for (auto&& r : res) {
        os << r.width << "x" << r.height << " ";
      }
      LOG(WARNING) << os.str();
    } else {
      auto n = file_channel_.SetImgParamsToData(img_params, data + 3 + size);
      if (n > 0) {
        header[1] = true;
        size += n;
      }
    }
  }
  if (imu_params != nullptr) {
    auto n = file_channel_.SetImuParamsToData(imu_params, data + 3 + size);
    if (n > 0) {
      header[2] = true;
      size += n;
    }
  }
  if (size + 3 > 2000) {
    LOG(FATAL) << "SetFiles failed, data is too large: " << (size + 3);
  }

  data[0] = static_cast<std::uint8_t>(header.to_ulong());
  data[1] = static_cast<std::uint8_t>((size >> 8) & 0xFF);
  data[2] = static_cast<std::uint8_t>(size & 0xFF);

  VLOG(2) << "SetFiles header: 0x" << std::hex << std::uppercase << std::setw(2)
          << std::setfill('0') << static_cast<int>(data[0]);
  if (XuFileQuery(uvc::XU_QUERY_SET, 2000, data)) {
    VLOG(2) << "SetFiles success";
    return true;
  } else {
    LOG(WARNING) << "SetFiles failed";
    return false;
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

bool Channels::XuControlRange(
    channel_t channel, uint8_t id, int32_t *min, int32_t *max,
    int32_t *def) const {
  return XuControlRange(mynteye_xu, channel, id, min, max, def);
}

bool Channels::XuControlRange(
    const uvc::xu &xu, uint8_t selector, uint8_t id, int32_t *min, int32_t *max,
    int32_t *def) const {
  CHECK_NOTNULL(device_);
  return uvc::xu_control_range(*device_, xu, selector, id, min, max, def);
}

bool Channels::XuControlQuery(
    channel_t channel, uvc::xu_query query, uint16_t size,
    uint8_t *data) const {
  return XuControlQuery(mynteye_xu, channel, query, size, data);
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
    return (data[1] << 8) | (data[2]);
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

bool Channels::XuHalfDuplexSet(Option option, std::uint64_t value) const {
  int id = XuHalfDuplexId(option);
  std::uint8_t data[20] = {static_cast<std::uint8_t>(id & 0xFF),
                           static_cast<std::uint8_t>(value & 0xFF),
                           static_cast<std::uint8_t>((value >> 8) & 0xFF),
                           static_cast<std::uint8_t>((value >> 16) & 0xFF),
                           static_cast<std::uint8_t>((value >> 24) & 0xFF),
                           static_cast<std::uint8_t>((value >> 32) & 0xFF),
                           static_cast<std::uint8_t>((value >> 40) & 0xFF),
                           static_cast<std::uint8_t>((value >> 48) & 0xFF),
                           static_cast<std::uint8_t>((value >> 56) & 0xFF)};

  if (XuControlQuery(CHANNEL_HALF_DUPLEX, uvc::XU_QUERY_SET, 20, data)) {
    VLOG(2) << "XuHalfDuplexSet value (0x" << std::hex << std::uppercase << value  // NOLINT
            << ") of " << option << " success";
    return true;
  } else {
    LOG(WARNING) << "XuHalfDuplexSet value (0x" << std::hex << std::uppercase
                 << value << ") of " << option << " failed";
    return false;
  }
}

bool Channels::XuImuWrite(const ImuReqPacket &req) const {
  auto &&data = req.to_data();
  // LOG(INFO) << data.size() << "||" << (int)data[0] << " " <<  (int)data[1] << " " << (int)data[2] << " " << (int)data[3] << " " << (int)data[4];  // NOLINT
  if (XuControlQuery(
          CHANNEL_IMU_WRITE, uvc::XU_QUERY_SET, data.size(), data.data())) {
    VLOG(2) << "XuImuWrite request success";
    return true;
  } else {
    LOG(WARNING) << "XuImuWrite request failed";
    return false;
  }
}

bool Channels::XuImuWrite(const ImuReqPacket2 &req) const {
  auto &&data = req.to_data();
  // LOG(INFO) << data.size() << "||" << (int)data[0] << " " <<  (int)data[1] << " " << (int)data[2] << " " << (int)data[3] << " " << (int)data[4];  // NOLINT
  if (XuControlQuery(
          CHANNEL_IMU_WRITE, uvc::XU_QUERY_SET, data.size(), data.data())) {
    VLOG(2) << "XuImuWrite request success";
    return true;
  } else {
    LOG(WARNING) << "XuImuWrite request failed";
    return false;
  }
}

bool Channels::XuImuRead(ImuResPacket2 *res) const {
  static std::uint8_t data[2000]{};
  if (XuControlQuery(CHANNEL_IMU_READ, uvc::XU_QUERY_GET, 2000, data)) {
    adapter_->GetImuResPacket2(data, res, enable_imu_correspondence);
    if (res->header != 0x5B) {
      LOG(WARNING) << "Imu response packet header must be 0x5B, but 0x"
                  << std::hex << std::uppercase << std::setw(2)
                  << std::setfill('0') << static_cast<int>(res->header)
                  << " now";
      return false;
    }
    if (res->state != 0) {
      LOG(WARNING) << "Imu response packet state must be 0, but " << res->state  // NOLINT
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
bool Channels::XuImuRead(ImuResPacket *res) const {
  static std::uint8_t data[2000]{};
  // std::fill(data, data + 2000, 0);  // reset
  if (XuControlQuery(CHANNEL_IMU_READ, uvc::XU_QUERY_GET, 2000, data)) {
    adapter_->GetImuResPacket(data, res);
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
    if (res->checksum != checksum) {
      LOG(WARNING) << "Imu response packet checksum should be 0x" << std::hex
                   << std::uppercase << std::setw(2) << std::setfill('0')
                   << static_cast<int>(res->checksum) << ", but 0x"
                   << std::setw(2) << std::setfill('0')
                   << static_cast<int>(checksum) << " now";
      return false;
    }

    VLOG(2) << "XuImuRead response success";
    return true;
  } else {
    LOG(WARNING) << "XuImuRead response failed";
    return false;
  }
}

bool Channels::XuFileQuery(
    uvc::xu_query query, uint16_t size, uint8_t *data) const {
  return XuControlQuery(CHANNEL_FILE, query, size, data);
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

  int32_t min = 0, max = 0, def = 0;
  if (!XuControlRange(
          CHANNEL_CAM_CTRL, static_cast<std::uint8_t>(id), &min, &max, &def)) {
    LOG(WARNING) << "Get XuControlInfo of " << option << " failed";
  }
  return {min, max, def};
}

// ChannelsAdapter

ChannelsAdapter::ChannelsAdapter(const Model &model)
  : model_(model) {
}

ChannelsAdapter::~ChannelsAdapter() {
}

std::set<Option> ChannelsAdapter::GetOptionSupports() {
  return option_supports_map.at(model_);
}

std::set<Resolution> ChannelsAdapter::GetResolutionSupports() {
  std::set<Resolution> res;
  auto requests_map = stream_requests_map.at(model_);
  for (auto&& r_map : requests_map) {
    if (r_map.first == Capabilities::STEREO) {
      for (auto&& r : r_map.second) {
        res.insert({r.width, r.height});
      }
    } else if (r_map.first == Capabilities::STEREO_COLOR) {
      for (auto&& r : r_map.second) {
        res.insert({static_cast<std::uint16_t>(r.width / 2), r.height});
      }
    }
  }
  return res;
}

MYNTEYE_END_NAMESPACE
