#include "internal/channels.h"

#include <glog/logging.h>

#include <bitset>
#include <chrono>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>

#include "internal/strings.h"

MYNTEYE_BEGIN_NAMESPACE

namespace {

const uvc::xu mynteye_xu = {3, 2, {0x18682d34, 0xdd2c, 0x4073, {0xad, 0x23, 0x72, 0x14, 0x73, 0x9a, 0x07, 0x4c}}};

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

void CheckSpecVersion(const Version *spec_version) {
  if (spec_version == nullptr) {
    LOG(FATAL) << "Spec version must be specified";
  }

  std::vector<std::string> spec_versions{"1.0"};
  for (auto &&spec_ver : spec_versions) {
    if (*spec_version == Version(spec_ver)) {
      return;  // supported
    }
  }

  std::ostringstream ss;
  std::copy(
      spec_versions.begin(), spec_versions.end(),
      std::ostream_iterator<std::string>(ss, ","));
  LOG(FATAL) << "Spec version " << spec_version->to_string()
             << " not supported, must in [" << ss.str() << "]";
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

namespace {

template <typename T>
T _from_data(const std::uint8_t *data) {
  std::size_t size = sizeof(T) / sizeof(std::uint8_t);
  T value = 0;
  for (std::size_t i = 0; i < size; i++) {
    value |= data[i] << (8 * (size - i - 1));
  }
  return value;
}

template <>
double _from_data(const std::uint8_t *data) {
  return *(reinterpret_cast<const double *>(data));
}

std::string _from_data(const std::uint8_t *data, std::size_t count) {
  std::string s(reinterpret_cast<const char *>(data), count);
  strings::trim(s);
  return s;
}

std::size_t from_data(Channels::device_info_t *info, const std::uint8_t *data) {
  std::size_t i = 4;  // skip vid, pid
  // name, 16
  info->name = _from_data(data + i, 16);
  i += 16;
  // serial_number, 16
  info->serial_number = _from_data(data + i, 16);
  i += 16;
  // firmware_version, 2
  info->firmware_version.set_major(data[i]);
  info->firmware_version.set_minor(data[i + 1]);
  i += 2;
  // hardware_version, 3
  info->hardware_version.set_major(data[i]);
  info->hardware_version.set_minor(data[i + 1]);
  info->hardware_version.set_flag(std::bitset<8>(data[i + 2]));
  i += 3;
  // spec_version, 2
  info->spec_version.set_major(data[i]);
  info->spec_version.set_minor(data[i + 1]);
  i += 2;
  // lens_type, 4
  info->lens_type.set_vendor(_from_data<std::uint16_t>(data + i));
  info->lens_type.set_product(_from_data<std::uint16_t>(data + i + 2));
  i += 4;
  // imu_type, 4
  info->imu_type.set_vendor(_from_data<std::uint16_t>(data + i));
  info->imu_type.set_product(_from_data<std::uint16_t>(data + i + 2));
  i += 4;
  // nominal_baseline, 2
  info->nominal_baseline = _from_data<std::uint16_t>(data + i);
  i += 2;

  return i;
}

std::size_t from_data(
    Intrinsics *in, const std::uint8_t *data, const Version *spec_version) {
  std::size_t i = 0;

  // width, 2
  in->width = _from_data<std::uint16_t>(data + i);
  i += 2;
  // height, 2
  in->height = _from_data<std::uint16_t>(data + i);
  i += 2;
  // fx, 8
  in->fx = _from_data<double>(data + i);
  i += 8;
  // fy, 8
  in->fy = _from_data<double>(data + i);
  i += 8;
  // cx, 8
  in->cx = _from_data<double>(data + i);
  i += 8;
  // cy, 8
  in->cy = _from_data<double>(data + i);
  i += 8;
  // model, 1
  in->model = data[i];
  i += 1;
  // coeffs, 40
  for (std::size_t j = 0; j < 5; j++) {
    in->coeffs[j] = _from_data<double>(data + i + j * 8);
  }
  i += 40;

  UNUSED(spec_version)
  return i;
}

std::size_t from_data(
    ImuIntrinsics *in, const std::uint8_t *data, const Version *spec_version) {
  std::size_t i = 0;

  // scale
  for (std::size_t j = 0; j < 3; j++) {
    for (std::size_t k = 0; k < 3; k++) {
      in->scale[j][k] = _from_data<double>(data + i + (j * 3 + k) * 8);
    }
  }
  i += 72;
  // drift
  for (std::size_t j = 0; j < 3; j++) {
    in->drift[j] = _from_data<double>(data + i + j * 8);
  }
  i += 24;
  // noise
  for (std::size_t j = 0; j < 3; j++) {
    in->noise[j] = _from_data<double>(data + i + j * 8);
  }
  i += 24;
  // bias
  for (std::size_t j = 0; j < 3; j++) {
    in->bias[j] = _from_data<double>(data + i + j * 8);
  }
  i += 24;

  UNUSED(spec_version)
  return i;
}

std::size_t from_data(
    Extrinsics *ex, const std::uint8_t *data, const Version *spec_version) {
  std::size_t i = 0;

  // rotation
  for (std::size_t j = 0; j < 3; j++) {
    for (std::size_t k = 0; k < 3; k++) {
      ex->rotation[j][k] = _from_data<double>(data + i + (j * 3 + k) * 8);
    }
  }
  i += 72;
  // translation
  for (std::size_t j = 0; j < 3; j++) {
    ex->translation[j] = _from_data<double>(data + i + j * 8);
  }
  i += 24;

  UNUSED(spec_version)
  return i;
}

std::size_t from_data(
    Channels::img_params_t *img_params, const std::uint8_t *data,
    const Version *spec_version) {
  std::size_t i = 0;
  i += from_data(&img_params->in_left, data + i, spec_version);
  i += from_data(&img_params->in_right, data + i, spec_version);
  i += from_data(&img_params->ex_left_to_right, data + i, spec_version);
  return i;
}

std::size_t from_data(
    Channels::imu_params_t *imu_params, const std::uint8_t *data,
    const Version *spec_version) {
  std::size_t i = 0;
  i += from_data(&imu_params->in_accel, data + i, spec_version);
  i += from_data(&imu_params->in_gyro, data + i, spec_version);
  i += from_data(&imu_params->ex_left_to_imu, data + i, spec_version);
  return i;
}

}  // namespace

bool Channels::GetFiles(
    device_info_t *info, img_params_t *img_params, imu_params_t *imu_params,
    Version *spec_version) const {
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
    std::uint16_t size = _from_data<std::uint16_t>(data + 1);
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

    Version *spec_ver = spec_version;
    std::size_t i = 3;
    std::size_t end = 3 + size;
    while (i < end) {
      std::uint8_t file_id = *(data + i);
      std::uint16_t file_size = _from_data<std::uint16_t>(data + i + 1);
      VLOG(2) << "GetFiles id: " << static_cast<int>(file_id)
              << ", size: " << file_size;
      i += 3;
      switch (file_id) {
        case FID_DEVICE_INFO: {
          CHECK_EQ(from_data(info, data + i), file_size)
              << "The firmware not support getting device info, you could "
                 "upgrade to latest";
          spec_ver = &info->spec_version;
          CheckSpecVersion(spec_ver);
        } break;
        case FID_IMG_PARAMS: {
          img_params->ok = file_size > 0;
          if (img_params->ok) {
            CheckSpecVersion(spec_ver);
            CHECK_EQ(from_data(img_params, data + i, spec_ver), file_size);
          }
        } break;
        case FID_IMU_PARAMS: {
          imu_params->ok = file_size > 0;
          if (imu_params->ok) {
            CheckSpecVersion(spec_ver);
            CHECK_EQ(from_data(imu_params, data + i, spec_ver), file_size);
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

namespace {

template <typename T>
std::size_t _to_data(T value, std::uint8_t *data) {
  std::size_t size = sizeof(T) / sizeof(std::uint8_t);
  for (std::size_t i = 0; i < size; i++) {
    data[i] = static_cast<std::uint8_t>((value >> (8 * (size - i - 1))) & 0xFF);
  }
  return size;
}

template <>
std::size_t _to_data(double value, std::uint8_t *data) {
  std::uint8_t *val = reinterpret_cast<std::uint8_t *>(&value);
  std::copy(val, val + 8, data);
  return 8;
}

std::size_t _to_data(std::string value, std::uint8_t *data, std::size_t count) {
  std::copy(value.begin(), value.end(), data);
  for (std::size_t i = value.size(); i < count; i++) {
    data[i] = ' ';
  }
  return count;
}

std::size_t to_data(
    const Channels::device_info_t *info, std::uint8_t *data,
    const Version *spec_version) {
  std::size_t i = 3;  // skip id, size
  i += 4;             // skip vid, pid
  // name, 16
  _to_data(info->name, data + i, 16);
  i += 16;
  // serial_number, 16
  _to_data(info->serial_number, data + i, 16);
  i += 16;
  // firmware_version, 2
  data[i] = info->firmware_version.major();
  data[i + 1] = info->firmware_version.minor();
  i += 2;
  // hardware_version, 3
  data[i] = info->hardware_version.major();
  data[i + 1] = info->hardware_version.minor();
  data[i + 2] =
      static_cast<std::uint8_t>(info->hardware_version.flag().to_ulong());
  i += 3;
  // spec_version, 2
  data[i] = info->spec_version.major();
  data[i + 1] = info->spec_version.minor();
  i += 2;
  // lens_type, 4
  _to_data(info->lens_type.vendor(), data + i);
  _to_data(info->lens_type.product(), data + i + 2);
  i += 4;
  // imu_type, 4
  _to_data(info->imu_type.vendor(), data + i);
  _to_data(info->imu_type.product(), data + i + 2);
  i += 4;
  // nominal_baseline, 2
  _to_data(info->nominal_baseline, data + i);
  i += 2;

  UNUSED(spec_version)

  // others
  std::size_t size = i - 3;
  data[0] = Channels::FID_DEVICE_INFO;
  data[1] = static_cast<std::uint8_t>((size >> 8) & 0xFF);
  data[2] = static_cast<std::uint8_t>(size & 0xFF);
  return size + 3;
}

std::size_t to_data(
    const Intrinsics *in, std::uint8_t *data, const Version *spec_version) {
  std::size_t i = 0;

  // width, 2
  _to_data(in->width, data + i);
  i += 2;
  // height, 2
  _to_data(in->height, data + i);
  i += 2;
  // fx, 8
  _to_data(in->fx, data + i);
  i += 8;
  // fy, 8
  _to_data(in->fy, data + i);
  i += 8;
  // cx, 8
  _to_data(in->cx, data + i);
  i += 8;
  // cy, 8
  _to_data(in->cy, data + i);
  i += 8;
  // model, 1
  data[i] = in->model;
  i += 1;
  // coeffs, 40
  for (std::size_t j = 0; j < 5; j++) {
    _to_data(in->coeffs[j], data + i + j * 8);
  }
  i += 40;

  UNUSED(spec_version)
  return i;
}

std::size_t to_data(
    const ImuIntrinsics *in, std::uint8_t *data, const Version *spec_version) {
  std::size_t i = 0;

  // scale
  for (std::size_t j = 0; j < 3; j++) {
    for (std::size_t k = 0; k < 3; k++) {
      _to_data(in->scale[j][k], data + i + (j * 3 + k) * 8);
    }
  }
  i += 72;
  // drift
  for (std::size_t j = 0; j < 3; j++) {
    _to_data(in->drift[j], data + i + j * 8);
  }
  i += 24;
  // noise
  for (std::size_t j = 0; j < 3; j++) {
    _to_data(in->noise[j], data + i + j * 8);
  }
  i += 24;
  // bias
  for (std::size_t j = 0; j < 3; j++) {
    _to_data(in->bias[j], data + i + j * 8);
  }
  i += 24;

  UNUSED(spec_version)
  return i;
}

std::size_t to_data(
    const Extrinsics *ex, std::uint8_t *data, const Version *spec_version) {
  std::size_t i = 0;

  // rotation
  for (std::size_t j = 0; j < 3; j++) {
    for (std::size_t k = 0; k < 3; k++) {
      _to_data(ex->rotation[j][k], data + i + (j * 3 + k) * 8);
    }
  }
  i += 72;
  // translation
  for (std::size_t j = 0; j < 3; j++) {
    _to_data(ex->translation[j], data + i + j * 8);
  }
  i += 24;

  UNUSED(spec_version)
  return i;
}

std::size_t to_data(
    const Channels::img_params_t *img_params, std::uint8_t *data,
    const Version *spec_version) {
  std::size_t i = 3;  // skip id, size
  i += to_data(&img_params->in_left, data + i, spec_version);
  i += to_data(&img_params->in_right, data + i, spec_version);
  i += to_data(&img_params->ex_left_to_right, data + i, spec_version);
  // others
  std::size_t size = i - 3;
  data[0] = Channels::FID_IMG_PARAMS;
  data[1] = static_cast<std::uint8_t>((size >> 8) & 0xFF);
  data[2] = static_cast<std::uint8_t>(size & 0xFF);
  return size + 3;
}

std::size_t to_data(
    const Channels::imu_params_t *imu_params, std::uint8_t *data,
    const Version *spec_version) {
  std::size_t i = 3;  // skip id, size
  i += to_data(&imu_params->in_accel, data + i, spec_version);
  i += to_data(&imu_params->in_gyro, data + i, spec_version);
  i += to_data(&imu_params->ex_left_to_imu, data + i, spec_version);
  // others
  std::size_t size = i - 3;
  data[0] = Channels::FID_IMU_PARAMS;
  data[1] = static_cast<std::uint8_t>((size >> 8) & 0xFF);
  data[2] = static_cast<std::uint8_t>(size & 0xFF);
  return size + 3;
}

}  // namespace

bool Channels::SetFiles(
    device_info_t *info, img_params_t *img_params, imu_params_t *imu_params,
    Version *spec_version) {
  if (info == nullptr && img_params == nullptr && imu_params == nullptr) {
    LOG(WARNING) << "Files are not provided to set";
    return false;
  }
  Version *spec_ver = spec_version;
  if (spec_ver == nullptr && info != nullptr) {
    spec_ver = &info->spec_version;
  }
  CheckSpecVersion(spec_ver);

  std::uint8_t data[2000]{};

  std::bitset<8> header;
  header[7] = 1;  // set

  std::uint16_t size = 0;
  if (info != nullptr) {
    header[0] = true;
    size += to_data(info, data + 3 + size, spec_ver);
  }
  if (img_params != nullptr) {
    header[1] = true;
    size += to_data(img_params, data + 3 + size, spec_ver);
  }
  if (imu_params != nullptr) {
    header[2] = true;
    size += to_data(imu_params, data + 3 + size, spec_ver);
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

bool Channels::XuControlQuery(
    channel_t channel, uvc::xu_query query, uint16_t size,
    uint8_t *data) const {
  return XuControlQuery(mynteye_xu, channel >> 8, query, size, data);
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

  std::uint8_t data[3] = {static_cast<std::uint8_t>((id | 0x80) & 0xFF), 0, 0};
  if (!XuCamCtrlQuery(uvc::XU_QUERY_SET, 3, data)) {
    LOG(WARNING) << "Get XuControlInfo of " << option << " failed";
    return {0, 0, 0};
  }

  control_info_t info{0, 0, 0};

  data[0] = id & 0xFF;
  if (XuCamCtrlQuery(uvc::XU_QUERY_MIN, 3, data)) {
    info.min = (data[1] << 8) | (data[2]);
  } else {
    LOG(WARNING) << "Get XuControlInfo.min of " << option << " failed";
  }
  if (XuCamCtrlQuery(uvc::XU_QUERY_MAX, 3, data)) {
    info.max = (data[1] << 8) | (data[2]);
  } else {
    LOG(WARNING) << "Get XuControlInfo.max of " << option << " failed";
  }
  if (XuCamCtrlQuery(uvc::XU_QUERY_DEF, 3, data)) {
    info.def = (data[1] << 8) | (data[2]);
  } else {
    LOG(WARNING) << "Get XuControlInfo.def of " << option << " failed";
  }

  return info;
}

MYNTEYE_END_NAMESPACE
