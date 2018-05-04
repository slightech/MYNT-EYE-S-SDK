#include "device/device.h"

#include <glog/logging.h>

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "device/device_s.h"
#include "internal/channels.h"
#include "internal/config.h"
#include "internal/motions.h"
#include "internal/streams.h"
#include "internal/strings.h"
#include "internal/types.h"
#include "uvc/uvc.h"

MYNTEYE_BEGIN_NAMESPACE

namespace {

struct DeviceModel {
  char type;
  std::uint8_t generation;
  std::uint8_t baseline_code;
  std::uint8_t hardware_code;
  std::uint8_t custom_code;
  bool ir_fixed;

  DeviceModel() = default;
  explicit DeviceModel(std::string model) {
    CHECK_GE(model.size(), 5);
    type = model[0];
    generation = model[1];
    baseline_code = model[2];
    hardware_code = model[3];
    custom_code = model[4];
    ir_fixed = (model.size() == 8) && model.substr(5) == "-IR";
  }
};

}  // namespace

Device::Device(const Model &model, std::shared_ptr<uvc::device> device)
    : video_streaming_(false),
      motion_tracking_(false),
      model_(model),
      device_(device),
      streams_(nullptr),
      channels_(std::make_shared<Channels>(device)),
      motions_(std::make_shared<Motions>(channels_)) {
  VLOG(2) << __func__;
  ReadAllInfos();
}

Device::~Device() {
  VLOG(2) << __func__;
}

std::shared_ptr<Device> Device::Create(
    const std::string &name, std::shared_ptr<uvc::device> device) {
  if (name == "MYNTEYE") {
    return std::make_shared<StandardDevice>(device);
  } else if (strings::starts_with(name, "MYNT-EYE-")) {
    // TODO(JohnZhao): Create different device by name, such as MYNT-EYE-S1000
    std::string model_s = name.substr(9);
    VLOG(2) << "MYNE EYE Model: " << model_s;
    DeviceModel model(model_s);
    switch (model.type) {
      case 'S':
        return std::make_shared<StandardDevice>(device);
      default:
        LOG(FATAL) << "MYNT EYE model is not supported now";
    }
  }
  return nullptr;
}

bool Device::Supports(const Stream &stream) const {
  auto &&supports = stream_supports_map.at(model_);
  return supports.find(stream) != supports.end();
}

bool Device::Supports(const Capabilities &capability) const {
  auto &&supports = capabilities_supports_map.at(model_);
  return supports.find(capability) != supports.end();
}

bool Device::Supports(const Option &option) const {
  auto &&supports = option_supports_map.at(model_);
  return supports.find(option) != supports.end();
}

bool Device::Supports(const AddOns &addon) const {
  CHECK_NOTNULL(device_info_);
  auto &&hw_flag = device_info_->hardware_version.flag();
  switch (addon) {
    case AddOns::INFRARED:
      return hw_flag[0];
    case AddOns::INFRARED2:
      return hw_flag[1];
    default: { LOG(FATAL) << "Unknown add-on"; }
  }
}

const std::vector<StreamRequest> &Device::GetStreamRequests(
    const Capabilities &capability) const {
  if (!Supports(capability)) {
    LOG(FATAL) << "Unsupported capability: " << capability;
  }
  try {
    auto &&cap_requests = stream_requests_map.at(model_);
    return cap_requests.at(capability);
  } catch (const std::out_of_range &e) {
    LOG(FATAL) << "Stream request of " << capability << " of " << model_
               << " not found";
  }
}

void Device::ConfigStreamRequest(
    const Capabilities &capability, const StreamRequest &request) {
  auto &&requests = GetStreamRequests(capability);
  if (std::find(requests.cbegin(), requests.cend(), request) ==
      requests.cend()) {
    LOG(FATAL) << "Config stream request of " << capability
               << " is not accpected";
  }
  stream_config_requests_[capability] = request;
}

std::shared_ptr<DeviceInfo> Device::GetInfo() const {
  return device_info_;
}

std::string Device::GetInfo(const Info &info) const {
  CHECK_NOTNULL(device_info_);
  switch (info) {
    case Info::DEVICE_NAME: {
      return device_info_->name;
    } break;
    case Info::SERIAL_NUMBER: {
      return device_info_->serial_number;
    } break;
    case Info::FIRMWARE_VERSION: {
      return device_info_->firmware_version.to_string();
    } break;
    case Info::HARDWARE_VERSION: {
      return device_info_->hardware_version.to_string();
    } break;
    case Info::SPEC_VERSION: {
      return device_info_->spec_version.to_string();
    } break;
    case Info::LENS_TYPE: {
      return device_info_->lens_type.to_string();
    } break;
    case Info::IMU_TYPE: {
      return device_info_->imu_type.to_string();
    } break;
    case Info::NOMINAL_BASELINE: {
      return std::to_string(device_info_->nominal_baseline);
    } break;
    default: { LOG(FATAL) << "Unknown device info"; }
  }
}

Intrinsics Device::GetIntrinsics(const Stream &stream) const {
  try {
    return stream_intrinsics_.at(stream);
  } catch (const std::out_of_range &e) {
    LOG(WARNING) << "Intrinsics of " << stream << " not found";
    return {};
  }
}

Extrinsics Device::GetExtrinsics(const Stream &from, const Stream &to) const {
  try {
    return stream_from_extrinsics_.at(from).at(to);
  } catch (const std::out_of_range &e) {
    try {
      return stream_from_extrinsics_.at(to).at(from).Inverse();
    } catch (const std::out_of_range &e) {
      LOG(WARNING) << "Extrinsics from " << from << " to " << to
                   << " not found";
      return {};
    }
  }
}

MotionIntrinsics Device::GetMotionIntrinsics() const {
  if (motion_intrinsics_) {
    return *motion_intrinsics_;
  } else {
    LOG(WARNING) << "Motion intrinsics not found";
    return {};
  }
}

Extrinsics Device::GetMotionExtrinsics(const Stream &from) const {
  try {
    return motion_from_extrinsics_.at(from);
  } catch (const std::out_of_range &e) {
    LOG(WARNING) << "Motion extrinsics from " << from << " not found";
    return {};
  }
}

void Device::SetIntrinsics(const Stream &stream, const Intrinsics &in) {
  stream_intrinsics_[stream] = in;
}

void Device::SetExtrinsics(
    const Stream &from, const Stream &to, const Extrinsics &ex) {
  stream_from_extrinsics_[from][to] = ex;
}

void Device::SetMotionIntrinsics(const MotionIntrinsics &in) {
  if (motion_intrinsics_ == nullptr) {
    motion_intrinsics_ = std::make_shared<MotionIntrinsics>();
  }
  *motion_intrinsics_ = in;
}

void Device::SetMotionExtrinsics(const Stream &from, const Extrinsics &ex) {
  motion_from_extrinsics_[from] = ex;
}

void Device::LogOptionInfos() const {
  channels_->LogControlInfos();
}

OptionInfo Device::GetOptionInfo(const Option &option) const {
  if (!Supports(option)) {
    LOG(WARNING) << "Unsupported option: " << option;
    return {0, 0, 0};
  }
  auto &&info = channels_->GetControlInfo(option);
  return {info.min, info.max, info.def};
}

std::int32_t Device::GetOptionValue(const Option &option) const {
  if (!Supports(option)) {
    LOG(WARNING) << "Unsupported option: " << option;
    return -1;
  }
  return channels_->GetControlValue(option);
}

void Device::SetOptionValue(const Option &option, std::int32_t value) {
  if (!Supports(option)) {
    LOG(WARNING) << "Unsupported option: " << option;
    return;
  }
  channels_->SetControlValue(option, value);
}

bool Device::RunOptionAction(const Option &option) const {
  if (!Supports(option)) {
    LOG(WARNING) << "Unsupported option: " << option;
    return false;
  }
  return channels_->RunControlAction(option);
}

void Device::SetStreamCallback(
    const Stream &stream, stream_callback_t callback) {
  if (!Supports(stream)) {
    LOG(WARNING) << "Unsupported stream: " << stream;
    return;
  }
  if (callback) {
    stream_callbacks_[stream] = callback;
  } else {
    stream_callbacks_.erase(stream);
  }
}

void Device::SetMotionCallback(motion_callback_t callback) {
  motion_callback_ = callback;
}

bool Device::HasStreamCallback(const Stream &stream) const {
  try {
    return stream_callbacks_.at(stream) != nullptr;
  } catch (const std::out_of_range &e) {
    return false;
  }
}

bool Device::HasMotionCallback() const {
  return motion_callback_ != nullptr;
}

void Device::Start(const Source &source) {
  if (source == Source::VIDEO_STREAMING) {
    StartVideoStreaming();
  } else if (source == Source::MOTION_TRACKING) {
    StartMotionTracking();
  } else if (source == Source::ALL) {
    Start(Source::VIDEO_STREAMING);
    Start(Source::MOTION_TRACKING);
  } else {
    LOG(FATAL) << "Unsupported source :(";
  }
}

void Device::Stop(const Source &source) {
  if (source == Source::VIDEO_STREAMING) {
    StopVideoStreaming();
  } else if (source == Source::MOTION_TRACKING) {
    StopMotionTracking();
  } else if (source == Source::ALL) {
    Stop(Source::MOTION_TRACKING);
    // Must stop motion tracking before video streaming and sleep a moment here
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    Stop(Source::VIDEO_STREAMING);
  } else {
    LOG(FATAL) << "Unsupported source :(";
  }
}

void Device::WaitForStreams() {
  CHECK(video_streaming_);
  CHECK_NOTNULL(streams_);
  streams_->WaitForStreams();
}

std::vector<device::StreamData> Device::GetStreamDatas(const Stream &stream) {
  CHECK(video_streaming_);
  CHECK_NOTNULL(streams_);
  std::lock_guard<std::mutex> _(mtx_streams_);
  return streams_->GetStreamDatas(stream);
}

device::StreamData Device::GetLatestStreamData(const Stream &stream) {
  CHECK(video_streaming_);
  CHECK_NOTNULL(streams_);
  std::lock_guard<std::mutex> _(mtx_streams_);
  return streams_->GetLatestStreamData(stream);
}

void Device::EnableMotionDatas(std::size_t max_size) {
  CHECK_NOTNULL(motions_);
  motions_->EnableMotionDatas(max_size);
}

std::vector<device::MotionData> Device::GetMotionDatas() {
  CHECK(motion_tracking_);
  CHECK_NOTNULL(motions_);
  return motions_->GetMotionDatas();
}

const StreamRequest &Device::GetStreamRequest(const Capabilities &capability) {
  try {
    return stream_config_requests_.at(capability);
  } catch (const std::out_of_range &e) {
    auto &&requests = GetStreamRequests(capability);
    if (requests.size() == 1) {
      VLOG(2) << "Get the only one stream request of " << capability;
      return requests[0];
    } else {
      LOG(FATAL) << "Please config the stream request of " << capability;
    }
  }
}

void Device::StartVideoStreaming() {
  if (video_streaming_) {
    LOG(WARNING) << "Cannot start video streaming without first stopping it";
    return;
  }

  streams_ = std::make_shared<Streams>(GetKeyStreams());

  // if stream capabilities are supported with subdevices of device_
  /*
  Capabilities stream_capabilities[] = {
    Capabilities::STEREO,
    Capabilities::COLOR,
    Capabilities::DEPTH,
    Capabilities::POINTS,
    Capabilities::FISHEYE,
    Capabilities::INFRARED,
    Capabilities::INFRARED2
  };
  for (auto &&capability : stream_capabilities) {
  }
  */
  if (Supports(Capabilities::STEREO)) {
    // do stream request selection if more than one request of each stream
    auto &&stream_request = GetStreamRequest(Capabilities::STEREO);

    streams_->ConfigStream(Capabilities::STEREO, stream_request);
    uvc::set_device_mode(
        *device_, stream_request.width, stream_request.height,
        static_cast<int>(stream_request.format), stream_request.fps,
        [this](const void *data) {
          // drop the first stereo stream data
          static std::uint8_t drop_count = 1;
          if (drop_count > 0) {
            --drop_count;
            return;
          }
          std::lock_guard<std::mutex> _(mtx_streams_);
          streams_->PushStream(Capabilities::STEREO, data);
          if (HasStreamCallback(Stream::LEFT)) {
            auto &&stream_datas = streams_->stream_datas(Stream::LEFT);
            if (stream_datas.size() > 0) {
              stream_callbacks_.at(Stream::LEFT)(stream_datas.back());
            }
          }
          if (HasStreamCallback(Stream::RIGHT)) {
            auto &&stream_datas = streams_->stream_datas(Stream::RIGHT);
            if (stream_datas.size() > 0) {
              stream_callbacks_.at(Stream::RIGHT)(stream_datas.back());
            }
          }
        });
  } else {
    LOG(FATAL) << "Not any stream capabilities are supported by this device";
  }

  uvc::start_streaming(*device_, 0);
  video_streaming_ = true;
}

void Device::StopVideoStreaming() {
  if (!video_streaming_) {
    LOG(WARNING) << "Cannot stop video streaming without first starting it";
    return;
  }
  stop_streaming(*device_);
  video_streaming_ = false;
}

void Device::StartMotionTracking() {
  if (!Supports(Capabilities::IMU)) {
    LOG(FATAL) << "IMU capability is not supported by this device";
  }
  if (motion_tracking_) {
    LOG(WARNING) << "Cannot start motion tracking without first stopping it";
    return;
  }
  motions_->SetMotionCallback([this](const device::MotionData &data) {
    if (motion_callback_) {
      motion_callback_(data);
    }
  });
  motions_->StartMotionTracking();
  motion_tracking_ = true;
}

void Device::StopMotionTracking() {
  if (!motion_tracking_) {
    LOG(WARNING) << "Cannot stop motion tracking without first starting it";
    return;
  }
  motions_->StopMotionTracking();
  motion_tracking_ = false;
}

void Device::ReadAllInfos() {
  device_info_ = std::make_shared<DeviceInfo>();

  CHECK_NOTNULL(channels_);
  Channels::img_params_t img_params;
  Channels::imu_params_t imu_params;
  if (!channels_->GetFiles(device_info_.get(), &img_params, &imu_params)) {
    LOG(FATAL) << "Read device infos failed :(";
  }
  VLOG(2) << "Device info: {name: " << device_info_->name
          << ", serial_number: " << device_info_->serial_number
          << ", firmware_version: "
          << device_info_->firmware_version.to_string()
          << ", hardware_version: "
          << device_info_->hardware_version.to_string()
          << ", spec_version: " << device_info_->spec_version.to_string()
          << ", lens_type: " << device_info_->lens_type.to_string()
          << ", imu_type: " << device_info_->imu_type.to_string()
          << ", nominal_baseline: " << device_info_->nominal_baseline << "}";

  device_info_->name = uvc::get_name(*device_);
  if (img_params.ok) {
    SetIntrinsics(Stream::LEFT, img_params.in_left);
    SetIntrinsics(Stream::RIGHT, img_params.in_right);
    SetExtrinsics(Stream::LEFT, Stream::RIGHT, img_params.ex_left_to_right);
    VLOG(2) << "Intrinsics left: {" << GetIntrinsics(Stream::LEFT) << "}";
    VLOG(2) << "Intrinsics right: {" << GetIntrinsics(Stream::RIGHT) << "}";
    VLOG(2) << "Extrinsics left to right: {"
            << GetExtrinsics(Stream::LEFT, Stream::RIGHT) << "}";
  } else {
    LOG(WARNING) << "Intrinsics & extrinsics not exist";
  }
  if (imu_params.ok) {
    SetMotionIntrinsics({imu_params.in_accel, imu_params.in_gyro});
    SetMotionExtrinsics(Stream::LEFT, imu_params.ex_left_to_imu);
    VLOG(2) << "Motion intrinsics: {" << GetMotionIntrinsics() << "}";
    VLOG(2) << "Motion extrinsics left to imu: {"
            << GetMotionExtrinsics(Stream::LEFT) << "}";
  } else {
    LOG(WARNING) << "Motion intrinsics & extrinsics not exist";
  }
}

MYNTEYE_END_NAMESPACE
