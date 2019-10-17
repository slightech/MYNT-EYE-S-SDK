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
#include "mynteye/device/device.h"

#include <algorithm>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

#include "mynteye/logger.h"
#include "mynteye/device/async_callback.h"
#include "mynteye/device/channel/channels.h"
#include "mynteye/device/config.h"
#include "mynteye/device/motions.h"
#include "mynteye/device/standard/device_s.h"
#include "mynteye/device/standard2/device_s2.h"
#include "mynteye/device/streams.h"
#include "mynteye/device/types.h"
#include "mynteye/util/strings.h"
#include "mynteye/util/times.h"
#include "mynteye/uvc/uvc.h"

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

bool CheckSupports(
    const Device *const device, const Stream &stream, bool fatal = true) {
  if (device->Supports(stream)) {
    return true;
  } else {
    auto &&supports = stream_supports_map.at(device->GetModel());
    std::ostringstream ss;
    std::copy(
        supports.begin(), supports.end(),
        std::ostream_iterator<Stream>(ss, ", "));
    if (fatal) {
      LOG(FATAL) << "Unsupported stream: " << stream
                 << ". Please use these: " << ss.str();
    } else {
      LOG(WARNING) << "Unsupported stream: " << stream
                   << ". Please use these: " << ss.str();
    }
    return false;
  }
}

}  // namespace

Device::Device(const Model &model,
    const std::shared_ptr<uvc::device> &device,
    const std::shared_ptr<StreamsAdapter> &streams_adapter,
    const std::shared_ptr<ChannelsAdapter> &channels_adapter)
  : video_streaming_(false),
    motion_tracking_(false),
    model_(model),
    device_(device),
    streams_(std::make_shared<Streams>(streams_adapter)),
    channels_(std::make_shared<Channels>(device_, channels_adapter)),
    motions_(std::make_shared<Motions>(channels_)),
    is_default_intrinsics_(false) {
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
    std::string model_s = name.substr(9, 5);
    VLOG(2) << "MYNE EYE Model: " << model_s;
    DeviceModel model(model_s);
    if (model.type == 'S') {
      if (model.generation == '1') {
        return std::make_shared<StandardDevice>(device);
      } else if (model.generation == '2') {
        if (model.baseline_code == '1') {
          if (model.custom_code == '0') {
            return std::make_shared<Standard2Device>(Model::STANDARD2, device);
          } else if (model.custom_code == 'A') {
            return std::make_shared<Standard2Device>(
                Model::STANDARD210A, device);
          } else {
            LOG(FATAL) << "No such custom code now";
          }
        } else if (model.baseline_code == '0') {
          if (model.custom_code == 'B') {
            return std::make_shared<Standard2Device>(Model::STANDARD200B, device);
          } else {
            LOG(FATAL) << "No such custom code now";
          }
        } else {
          LOG(FATAL) << "No such baseline now";
        }
      } else {
        LOG(FATAL) << "No such generation now";
      }
    } else {
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
    default:
      LOG(WARNING) << "Unknown add-on";
      return false;
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
    LOG(WARNING) << "Config stream request of " << capability
                 << " is not accpected";
    return;
  }
  stream_config_requests_[capability] = request;
  UpdateStreamIntrinsics(capability, request);
}

const StreamRequest &Device::GetStreamRequest(
    const Capabilities &capability) const {
  try {
    return stream_config_requests_.at(capability);
  } catch (const std::out_of_range &e) {
    auto &&requests = GetStreamRequests(capability);
    if (requests.size() >= 1) {
      return requests[0];
    } else {
      LOG(FATAL) << "Please config the stream request of " << capability;
    }
  }
}

const std::vector<StreamRequest> &Device::GetStreamRequests() const {
  return GetStreamRequests(GetKeyStreamCapability());
}

void Device::ConfigStreamRequest(const StreamRequest &request) {
  ConfigStreamRequest(GetKeyStreamCapability(), request);
}

const StreamRequest &Device::GetStreamRequest() const {
  return GetStreamRequest(GetKeyStreamCapability());
}

std::shared_ptr<DeviceInfo> Device::GetInfo() const {
  return device_info_;
}

std::string Device::GetInfo(const Info &info) const {
  CHECK_NOTNULL(device_info_);
  switch (info) {
    case Info::DEVICE_NAME:
      return device_info_->name;
    case Info::SERIAL_NUMBER:
      return device_info_->serial_number;
    case Info::FIRMWARE_VERSION:
      return device_info_->firmware_version.to_string();
    case Info::HARDWARE_VERSION:
      return device_info_->hardware_version.to_string();
    case Info::SPEC_VERSION:
      return device_info_->spec_version.to_string();
    case Info::LENS_TYPE:
      return device_info_->lens_type.to_string();
    case Info::IMU_TYPE:
      return device_info_->imu_type.to_string();
    case Info::NOMINAL_BASELINE:
      return std::to_string(device_info_->nominal_baseline);
    case Info::AUXILIARY_CHIP_VERSION:
      return device_info_->auxiliary_chip_version.to_string();
    case Info::ISP_VERSION:
      return device_info_->isp_version.to_string();
    default:
      LOG(WARNING) << "Unknown device info";
      return "";
  }
}

std::shared_ptr<IntrinsicsBase> Device::GetIntrinsics(
    const Stream &stream) const {
  bool ok;
  return GetIntrinsics(stream, &ok);
}

Extrinsics Device::GetExtrinsics(const Stream &from, const Stream &to) const {
  bool ok;
  return GetExtrinsics(from, to, &ok);
}

MotionIntrinsics Device::GetMotionIntrinsics() const {
  bool ok;
  return GetMotionIntrinsics(&ok);
}

Extrinsics Device::GetMotionExtrinsics(const Stream &from) const {
  bool ok;
  return GetMotionExtrinsics(from, &ok);
}

std::shared_ptr<IntrinsicsBase> Device::GetIntrinsics(
    const Stream &stream, bool *ok) const {
  try {
    *ok = true;
    return stream_intrinsics_.at(stream);
  } catch (const std::out_of_range &e) {
    *ok = false;
    LOG(WARNING) << "Intrinsics of " << stream << " not found";
    return {};
  }
}

Extrinsics Device::GetExtrinsics(
    const Stream &from, const Stream &to, bool *ok) const {
  try {
    *ok = true;
    return stream_from_extrinsics_.at(from).at(to);
  } catch (const std::out_of_range &e) {
    try {
      *ok = true;
      return stream_from_extrinsics_.at(to).at(from).Inverse();
    } catch (const std::out_of_range &e) {
      *ok = false;
      LOG(WARNING) << "Extrinsics from " << from << " to " << to
                   << " not found";
      return {};
    }
  }
}

MotionIntrinsics Device::GetMotionIntrinsics(bool *ok) const {
  if (motion_intrinsics_) {
    *ok = true;
    return *motion_intrinsics_;
  } else {
    *ok = false;
    VLOG(2) << "Motion intrinsics not found";
    return {};
  }
}

Extrinsics Device::GetMotionExtrinsics(const Stream &from, bool *ok) const {
  try {
    *ok = true;
    return motion_from_extrinsics_.at(from);
  } catch (const std::out_of_range &e) {
    *ok = false;
    VLOG(2) << "Motion extrinsics from " << from << " not found";
    return {};
  }
}

void Device::SetIntrinsics(const Stream &stream,
    const std::shared_ptr<IntrinsicsBase> &in) {
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
  motions_->SetMotionIntrinsics(motion_intrinsics_);
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
    if (option == Option::FRAME_RATE) {
      return GetStreamRequest().fps;
    }
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

bool Device::SetOptionValue(const Option &option, std::uint64_t value) {
  if (!Supports(option)) {
    LOG(WARNING) << "Unsupported option: " << option;
    return false;
  }
  return channels_->SetControlValue(option, value);
}

bool Device::RunOptionAction(const Option &option) const {
  if (!Supports(option)) {
    LOG(WARNING) << "Unsupported option: " << option;
    return false;
  }
  return channels_->RunControlAction(option);
}

void Device::SetStreamCallback(
    const Stream &stream, stream_callback_t callback, bool async) {
  if (!CheckSupports(this, stream, false)) {
    return;
  }
  if (callback) {
    stream_callbacks_[stream] = callback;
    if (async)
      stream_async_callbacks_[stream] =
          std::make_shared<stream_async_callback_t>(
              to_string(stream), callback);  // max_data_size = 1
  } else {
    stream_callbacks_.erase(stream);
    stream_async_callbacks_.erase(stream);
  }
}

void Device::SetMotionCallback(motion_callback_t callback, bool async) {
  motion_callback_ = callback;
  if (callback) {
    if (async)
      motion_async_callback_ =
          std::make_shared<motion_async_callback_t>("motion", callback, 1000);
    // will drop old motion datas after callback cost > 2 s (1000 / 500 Hz)
  } else {
    motion_async_callback_ = nullptr;
  }
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
    LOG(ERROR) << "Unsupported source :(";
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
    LOG(ERROR) << "Unsupported source :(";
  }
}

void Device::WaitForStreams() {
  CHECK(video_streaming_);
  CHECK_NOTNULL(streams_);
  streams_->WaitForStreams();
}

device::StreamData Device::GetStreamData(const Stream &stream) {
  CHECK(video_streaming_);
  CHECK_NOTNULL(streams_);
  CheckSupports(this, stream);
  std::lock_guard<std::mutex> _(mtx_streams_);
  return streams_->GetLatestStreamData(stream);
}

device::StreamData Device::GetLatestStreamData(const Stream &stream) {
  return GetStreamData(stream);
}

std::vector<device::StreamData> Device::GetStreamDatas(const Stream &stream) {
  CHECK(video_streaming_);
  CHECK_NOTNULL(streams_);
  CheckSupports(this, stream);
  std::lock_guard<std::mutex> _(mtx_streams_);
  return streams_->GetStreamDatas(stream);
}

void Device::DisableMotionDatas() {
  CHECK_NOTNULL(motions_);
  motions_->DisableMotionDatas();
}

void Device::EnableMotionDatas() {
  EnableMotionDatas(std::numeric_limits<std::size_t>::max());
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

void Device::StartVideoStreaming() {
  if (video_streaming_) {
    LOG(WARNING) << "Cannot start video streaming without first stopping it";
    return;
  }

  // if stream capabilities are supported with subdevices of device_
  /*
  Capabilities stream_capabilities[] = {
      Capabilities::STEREO,       Capabilities::STEREO_COLOR,
      Capabilities::COLOR,        Capabilities::DEPTH,
      Capabilities::POINTS,       Capabilities::FISHEYE,
      Capabilities::INFRARED,     Capabilities::INFRARED2};
  for (auto &&capability : stream_capabilities) {
  }
  */
  auto &&stream_cap = GetKeyStreamCapability();
  if (Supports(stream_cap)) {
    // do stream request selection if more than one request of each stream
    auto &&stream_request = GetStreamRequest(stream_cap);
    streams_->ConfigStream(stream_cap, stream_request);

    if (strstr(this->device_info_->name.c_str(), "S2") == nullptr) {
      uvc::set_device_mode(
          *device_, stream_request.width, stream_request.height,
          static_cast<int>(stream_request.format), stream_request.fps,
          [this, stream_cap](
              const void *data, std::function<void()> continuation) {
            // drop the first stereo stream data
            static std::uint8_t drop_count = 1;
            if (drop_count > 0) {
              --drop_count;
              continuation();
              return;
            }
            // auto &&time_beg = times::now();
            {
              std::lock_guard<std::mutex> _(mtx_streams_);
              if (streams_->PushStreamS1(stream_cap, data)) {
                CallbackPushedStreamData(Stream::LEFT);
                CallbackPushedStreamData(Stream::RIGHT);
              }
            }
            continuation();
            OnStereoStreamUpdate();
            // VLOG(2) << "Stereo video callback cost "
            //     << times::count<times::milliseconds>(times::now() - time_beg)
            //     << " ms";
          });
    } else {
      uvc::set_device_mode(
          *device_, stream_request.width, stream_request.height,
          static_cast<int>(stream_request.format), stream_request.fps,
          [this, stream_cap](
              const void *data, std::function<void()> continuation) {
            // drop the first stereo stream data
            static std::uint8_t drop_count = 1;
            if (drop_count > 0) {
              --drop_count;
              continuation();
              return;
            }
            // auto &&time_beg = times::now();
            {
              std::lock_guard<std::mutex> _(mtx_streams_);
              if (streams_->PushStream(stream_cap, data)) {
                CallbackPushedStreamData(Stream::LEFT);
                CallbackPushedStreamData(Stream::RIGHT);
              }
            }
            continuation();
            OnStereoStreamUpdate();
            // VLOG(2) << "Stereo video callback cost "
            //     << times::count<times::milliseconds>(times::now() - time_beg)
            //     << " ms";
          });
    }
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
  motions_->SetMotionCallback(
      std::bind(&Device::CallbackMotionData, this, std::placeholders::_1));
  // motions_->StartMotionTracking();
  motion_tracking_ = true;
}

void Device::StopMotionTracking() {
  if (!motion_tracking_) {
    LOG(WARNING) << "Cannot stop motion tracking without first starting it";
    return;
  }
  // motions_->StopMotionTracking();
  motion_tracking_ = false;
}

void Device::OnStereoStreamUpdate() {}

void Device::EnableImuCorrespondence(bool is_enable) {
  channels_->EnableImuCorrespondence(is_enable);
}

void Device::ReadAllInfos() {
  device_info_ = std::make_shared<DeviceInfo>();

  CHECK_NOTNULL(channels_);
  all_img_params_.clear();
  Device::imu_params_t imu_params;
  if (!channels_->GetFiles(device_info_.get(), &all_img_params_, &imu_params)) {
#if defined(WITH_DEVICE_INFO_REQUIRED)
    LOG(FATAL)
#else
    LOG(WARNING)
#endif
        << "Read device infos failed. Please upgrade your firmware to the "
           "latest version.";
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

  motions_->SetDeviceInfo(device_info_);

  if (all_img_params_.empty()) {
    is_default_intrinsics_ = true;
    LOG(ERROR) << "Image params not found, but we need it to process the "
                  "images. Please `make tools` and use `img_params_writer` "
                  "to write the image params. If you update the SDK from "
                  "1.x, the `SN*.conf` is the file contains them. Besides, "
                  "you could also calibrate them by yourself. Read the guide "
                  "doc (https://github.com/slightech/MYNT-EYE-SDK-2-Guide) "
                  "to learn more.";
    LOG(WARNING) << "Intrinsics & extrinsics not exist. Use default intrinsics.";
    LOG(INFO) << "camera calib model: unknow, use default pinhole data";
    auto spec_string = device_info_->spec_version.to_string();
    for (auto &&resolution : resolution_list) {
      all_img_params_[resolution] = {
          true, spec_string, getDefaultIntrinsics(resolution),
          getDefaultIntrinsics(resolution), *getDefaultExtrinsics()};
    }
  }

  for (auto &&params : all_img_params_) {
    auto &&img_params = params.second;
    if (img_params.ok) {
      SetIntrinsics(Stream::LEFT, img_params.in_left);
      SetIntrinsics(Stream::RIGHT, img_params.in_right);
      SetExtrinsics(Stream::RIGHT, Stream::LEFT, img_params.ex_right_to_left);
      VLOG(2) << "Intrinsics left: {" << *GetIntrinsics(Stream::LEFT) << "}";
      VLOG(2) << "Intrinsics right: {" << *GetIntrinsics(Stream::RIGHT) << "}";
      VLOG(2) << "Extrinsics left to right: {"
              << GetExtrinsics(Stream::RIGHT, Stream::LEFT) << "}";
      break;
    }
  }

  if (imu_params.ok) {
    imu_params_ = imu_params;
    SetMotionIntrinsics({imu_params.in_accel, imu_params.in_gyro});
    SetMotionExtrinsics(Stream::LEFT, imu_params.ex_left_to_imu);
    VLOG(2) << "Motion intrinsics: {" << GetMotionIntrinsics() << "}";
    VLOG(2) << "Motion extrinsics left to imu: {"
            << GetMotionExtrinsics(Stream::LEFT) << "}";
  } else {
    imu_params_.ok = false;
    VLOG(2) << "Motion intrinsics & extrinsics not exist";
  }
}

void Device::UpdateStreamIntrinsics(
    const Capabilities &capability, const StreamRequest &request) {
  if (capability != GetKeyStreamCapability()) {
    return;
  }

  for (auto &&params : all_img_params_) {
    auto &&img_res = params.first;
    auto &&img_params = params.second;
    bool ok = false;
    if (capability == Capabilities::STEREO_COLOR) {
      ok = img_params.ok &&
            img_res.height == request.GetResolution().height &&
            img_res.width == request.GetResolution().width / 2;
    } else if (capability == Capabilities::STEREO) {
      if (img_res == request.GetResolution()) {
        ok = img_params.ok;
      } else if (request.GetResolution().height / img_res.height ==
                request.GetResolution().width / img_res.width) {
        double scale = static_cast<double> (
          1.0 * request.GetResolution().height / img_res.height);
        img_params.in_left->resize_scale = scale;
        img_params.in_right->resize_scale = scale;
        ok = img_params.ok;
      } else {
        ok = false;
      }
    }
    if (ok) {
      SetIntrinsics(Stream::LEFT, img_params.in_left);
      SetIntrinsics(Stream::RIGHT, img_params.in_right);
      SetExtrinsics(Stream::RIGHT, Stream::LEFT, img_params.ex_right_to_left);
      VLOG(2) << "Intrinsics left: {" << *GetIntrinsics(Stream::LEFT) << "}";
      VLOG(2) << "Intrinsics right: {" << *GetIntrinsics(Stream::RIGHT) << "}";
      VLOG(2) << "Extrinsics left to right: {"
              << GetExtrinsics(Stream::RIGHT, Stream::LEFT) << "}";
      break;
    }
  }
}

void Device::CallbackPushedStreamData(const Stream &stream) {
  if (HasStreamCallback(stream)) {
    auto &&datas = streams_->stream_datas(stream);
    // if (datas.size() > 0) {}
    auto &&data = datas.back();
    if (stream_async_callbacks_.find(stream) != stream_async_callbacks_.end()) {
      stream_async_callbacks_.at(stream)->PushData(data);
    } else {
      stream_callbacks_.at(stream)(data);
    }
  }
}

void Device::CallbackMotionData(const device::MotionData &data) {
  if (HasMotionCallback()) {
    if (motion_async_callback_) {
      motion_async_callback_->PushData(data);
    } else {
      motion_callback_(data);
    }
  }
}

bool Device::GetFiles(
    DeviceInfo *info, img_params_map_t *img_params, imu_params_t *imu_params) {
  return channels_->GetFiles(info, img_params, imu_params);
}

bool Device::SetFiles(
    DeviceInfo *info, img_params_map_t *img_params, imu_params_t *imu_params) {
  return channels_->SetFiles(info, img_params, imu_params);
}

void Device::EnableProcessMode(const ProcessMode& mode) {
  EnableProcessMode(static_cast<std::int32_t>(mode));
}

void Device::EnableProcessMode(const std::int32_t& mode) {
  motions_->EnableProcessMode(mode);
}

bool Device::CheckImageParams() {
  return is_default_intrinsics_;
}

MYNTEYE_END_NAMESPACE
