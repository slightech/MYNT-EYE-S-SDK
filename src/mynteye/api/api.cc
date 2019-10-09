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
#include "mynteye/api/api.h"

#ifdef WITH_BOOST_FILESYSTEM
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#endif

#include <algorithm>
#include <thread>

#include "mynteye/logger.h"
#include "mynteye/api/correspondence.h"
#include "mynteye/api/dl.h"
#include "mynteye/api/plugin.h"
#include "mynteye/api/synthetic.h"
#include "mynteye/api/version_checker.h"
#include "mynteye/device/device.h"
#include "mynteye/device/utils.h"

#if defined(WITH_FILESYSTEM) && defined(WITH_NATIVE_FILESYSTEM)
#if defined(MYNTEYE_OS_WIN)
#include <windows.h>
#endif
#endif

MYNTEYE_BEGIN_NAMESPACE

namespace {

#if defined(WITH_FILESYSTEM)

#if defined(WITH_BOOST_FILESYSTEM)

namespace fs = boost::filesystem;

bool file_exists(const fs::path &p) {
  try {
    fs::file_status s = fs::status(p);
    return fs::exists(s) && fs::is_regular_file(s);
  } catch (fs::filesystem_error &e) {
    LOG(ERROR) << e.what();
    return false;
  }
}

bool dir_exists(const fs::path &p) {
  try {
    fs::file_status s = fs::status(p);
    return fs::exists(s) && fs::is_directory(s);
  } catch (fs::filesystem_error &e) {
    LOG(ERROR) << e.what();
    return false;
  }
}

#elif defined(WITH_NATIVE_FILESYSTEM)

#if defined(MYNTEYE_OS_WIN)

bool file_exists(const std::string &p) {
  DWORD attrs = GetFileAttributes(p.c_str());
  return (attrs != INVALID_FILE_ATTRIBUTES) &&
         !(attrs & FILE_ATTRIBUTE_DIRECTORY);
}

bool dir_exists(const std::string &p) {
  DWORD attrs = GetFileAttributes(p.c_str());
  return (attrs != INVALID_FILE_ATTRIBUTES) &&
         (attrs & FILE_ATTRIBUTE_DIRECTORY);
}

#else
#error "Unsupported native filesystem"
#endif

#endif

std::vector<std::string> get_plugin_paths() {
  std::string info_path = utils::get_sdk_install_dir();
  if (info_path.empty()) return {};

  info_path.append(MYNTEYE_OS_SEP "share" MYNTEYE_OS_SEP "mynteye"
                   MYNTEYE_OS_SEP "build.info");

  cv::FileStorage fs(info_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    LOG(WARNING) << "build.info not found: " << info_path;
    return {};
  }

  auto to_lower = [](std::string &s) {  // NOLINT
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  };

  std::string host_os = fs["HOST_OS"];
  to_lower(host_os);
  std::string host_name = fs["HOST_NAME"];
  to_lower(host_name);
  std::string host_arch = fs["HOST_ARCH"];
  to_lower(host_arch);
  std::string host_compiler = fs["HOST_COMPILER"];
  to_lower(host_compiler);

  // std::string compiler_version = fs["COMPILER_VERSION"];
  int compiler_version_major = fs["COMPILER_VERSION_MAJOR"];
  // int compiler_version_minor = fs["COMPILER_VERSION_MINOR"];
  // int compiler_version_patch = fs["COMPILER_VERSION_PATCH"];
  // int compiler_version_tweak = fs["COMPILER_VERSION_TWEAK"];

  std::string cuda_version = fs["CUDA_VERSION"];
  // int cuda_version_major = fs["CUDA_VERSION_MAJOR"];
  // int cuda_version_minor = fs["CUDA_VERSION_MINOR"];
  // std::string cuda_version_string = fs["CUDA_VERSION_STRING"];

  std::string opencv_version = fs["OpenCV_VERSION"];
  // int opencv_version_major = fs["OpenCV_VERSION_MAJOR"];
  // int opencv_version_minor = fs["OpenCV_VERSION_MINOR"];
  // int opencv_version_patch = fs["OpenCV_VERSION_PATCH"];
  // int opencv_version_tweak = fs["OpenCV_VERSION_TWEAK"];
  // std::string opencv_version_status = fs["OpenCV_VERSION_STATUS"];
  std::string opencv_with_world = fs["OpenCV_WITH_WORLD"];
  to_lower(opencv_with_world);

  std::string mynteye_version = fs["MYNTEYE_VERSION"];
  // int mynteye_version_major = fs["MYNTEYE_VERSION_MAJOR"];
  // int mynteye_version_minor = fs["MYNTEYE_VERSION_MINOR"];
  // int mynteye_version_patch = fs["MYNTEYE_VERSION_PATCH"];
  // int mynteye_version_tweak = fs["MYNTEYE_VERSION_TWEAK"];

  fs.release();

  std::string lib_prefix;
  std::string lib_suffix;
  if (host_os == "linux") {
    if (host_compiler != "gnu" || compiler_version_major < 5)
      return {};
    lib_prefix = "lib";
    lib_suffix = ".so";
  } else if (host_os == "win") {
    lib_prefix = "";
    lib_suffix = ".dll";
  } else if (host_os == "mac") {
    lib_prefix = "lib";
    lib_suffix = ".dylib";
  } else {
    return {};
  }

  std::vector<std::string> names;
  {
    std::vector<std::string> prefixes{
        // lib_prefix + "plugin_b_ocl" + ocl_version,
        lib_prefix + "plugin_g_cuda" + cuda_version,
    };
    std::string opencv_name("_opencv" + opencv_version);
    if (opencv_with_world == "true") {
      opencv_name.append("-world");
    }
    for (auto &&prefix : prefixes) {
      names.push_back(prefix + opencv_name + "_mynteye" + mynteye_version);
      names.push_back(prefix + opencv_name);
      names.push_back(prefix);
    }
    for (auto &&name : names) {
      name.append(lib_suffix);
    }
  }

  std::vector<std::string> paths;

  std::vector<std::string> plats;
  if (host_name != host_os) {
    plats.push_back(host_name + "-" + host_arch);
  }
  plats.push_back(host_os + "-" + host_arch);

  std::vector<std::string> dirs{
      utils::get_sdk_root_dir(), utils::get_sdk_install_dir()};
  for (auto &&plat : plats) {
    for (auto &&dir : dirs) {
      auto &&plat_dir = dir + MYNTEYE_OS_SEP "plugins" + MYNTEYE_OS_SEP + plat;
      // VLOG(2) << "plat_dir: " << plat_dir;
      if (!dir_exists(plat_dir))
        continue;
      for (auto &&name : names) {
        // VLOG(2) << "  name: " << name;
        auto &&path = plat_dir + MYNTEYE_OS_SEP + name;
        if (!file_exists(path))
          continue;
        paths.push_back(path);
      }
    }
  }

  return paths;
}

#endif

}  // namespace

API::API(std::shared_ptr<Device> device, CalibrationModel calib_model)
    : device_(device), correspondence_(nullptr),
      api_correspondence_enable_(false),
      dev_correspondence_enable_(false) {
  VLOG(2) << __func__;
  // std::dynamic_pointer_cast<StandardDevice>(device_);
  synthetic_.reset(new Synthetic(this, calib_model));
}

API::~API() {
  VLOG(2) << __func__;
}

std::shared_ptr<API> API::Create(int argc, char *argv[]) {
  auto &&device = device::select();
  if (!device) return nullptr;
  auto api = Create(argc, argv, device);
  if (api && checkFirmwareVersion(api))
    return api;
  return nullptr;
}

std::shared_ptr<API> API::Create(
    int argc, char *argv[], const std::shared_ptr<Device> &device) {
  static glog_init _(argc, argv);
  return Create(device);
}

std::shared_ptr<API> API::Create(const std::shared_ptr<Device> &device) {
  std::shared_ptr<API> api = nullptr;
  if (device != nullptr) {
    bool in_l_ok, in_r_ok;
    auto left_intr  = device->GetIntrinsics(Stream::LEFT, &in_l_ok);
    auto right_intr = device->GetIntrinsics(Stream::RIGHT, &in_r_ok);

    if (left_intr->calib_model() != right_intr->calib_model()) {
      LOG(ERROR) << "left camera and right camera use different calib models!";
      LOG(WARNING) << "use pinhole as default";
      api = std::make_shared<API>(device, CalibrationModel::UNKNOW);
      return api;
    } else {
      api = std::make_shared<API>(device, left_intr->calib_model());
      return api;
    }
  } else {
    LOG(ERROR) <<"no device!";
    return nullptr;
  }
  return api;
}

Model API::GetModel() const {
  return device_->GetModel();
}

bool API::Supports(const Stream &stream) const {
  return synthetic_->Supports(stream);
}

bool API::Supports(const Capabilities &capability) const {
  return device_->Supports(capability);
}

bool API::Supports(const Option &option) const {
  return device_->Supports(option);
}

bool API::Supports(const AddOns &addon) const {
  return device_->Supports(addon);
}

StreamRequest API::SelectStreamRequest(bool *ok) const {
  return device::select_request(device_, ok);
}

const std::vector<StreamRequest> &API::GetStreamRequests(
    const Capabilities &capability) const {
  return device_->GetStreamRequests(capability);
}

void API::ConfigStreamRequest(
    const Capabilities &capability, const StreamRequest &request) {
  device_->ConfigStreamRequest(capability, request);
  synthetic_->NotifyImageParamsChanged();
}

const StreamRequest &API::GetStreamRequest(
    const Capabilities &capability) const {
  return device_->GetStreamRequest(capability);
}

const std::vector<StreamRequest> &API::GetStreamRequests() const {
  return device_->GetStreamRequests();
}

void API::ConfigStreamRequest(const StreamRequest &request) {
  device_->ConfigStreamRequest(request);
  synthetic_->NotifyImageParamsChanged();
}

const StreamRequest &API::GetStreamRequest() const {
  return device_->GetStreamRequest();
}

std::shared_ptr<DeviceInfo> API::GetInfo() const {
  return device_->GetInfo();
}

std::string API::GetInfo(const Info &info) const {
  return device_->GetInfo(info);
}

std::string API::GetSDKVersion() const {
  return MYNTEYE_API_VERSION_STR;
}

IntrinsicsPinhole API::GetIntrinsics(const Stream &stream) const {
  auto in = GetIntrinsicsBase(stream);
  if (in->calib_model() == CalibrationModel::PINHOLE) {
    return *std::dynamic_pointer_cast<IntrinsicsPinhole>(in);
  }
  throw std::runtime_error("Intrinsics is not pinhole model"
      ", please use GetIntrinsicsBase() or GetIntrinsics<T>() instead.");
}

std::shared_ptr<IntrinsicsBase> API::GetIntrinsicsBase(
    const Stream &stream) const {
  return device_->GetIntrinsics(stream);
}

Extrinsics API::GetExtrinsics(const Stream &from, const Stream &to) const {
  return device_->GetExtrinsics(from, to);
}

MotionIntrinsics API::GetMotionIntrinsics() const {
  return device_->GetMotionIntrinsics();
}

Extrinsics API::GetMotionExtrinsics(const Stream &from) const {
  return device_->GetMotionExtrinsics(from);
}

void API::LogOptionInfos() const {
  device_->LogOptionInfos();
}

OptionInfo API::GetOptionInfo(const Option &option) const {
  return device_->GetOptionInfo(option);
}

std::int32_t API::GetOptionValue(const Option &option) const {
  return device_->GetOptionValue(option);
}

void API::SetOptionValue(const Option &option, std::int32_t value) {
  device_->SetOptionValue(option, value);
}

bool API::SetOptionValue(const Option &option, std::uint64_t value) {
  return device_->SetOptionValue(option, value);
}

bool API::RunOptionAction(const Option &option) const {
  return device_->RunOptionAction(option);
}

void API::SetStreamCallback(const Stream &stream, stream_callback_t callback) {
  synthetic_->SetStreamCallback(stream, callback);
}

void API::SetMotionCallback(motion_callback_t callback) {
  if (correspondence_) {
    correspondence_->SetMotionCallback(callback);
    return;
  }
  callback_ = callback;
  if (callback_) {
    device_->SetMotionCallback([this](const device::MotionData &data) {
      callback_({data.imu});
    }, true);
  } else {
    device_->SetMotionCallback(nullptr);
  }
}

bool API::HasStreamCallback(const Stream &stream) const {
  return synthetic_->HasStreamCallback(stream);
}

bool API::HasMotionCallback() const {
  return device_->HasMotionCallback();
}

void API::Start(const Source &source) {
  if (source == Source::VIDEO_STREAMING) {
#ifdef WITH_FILESYSTEM
    if (!synthetic_->HasPlugin()) {
      try {
        auto &&plugin_paths = get_plugin_paths();
        if (plugin_paths.size() > 0) {
          EnablePlugin(plugin_paths[0]);
        }
      } catch (...) {
        LOG(WARNING) << "Incorrect yaml format: build.info";
      }
    }
#endif
    synthetic_->StartVideoStreaming();
  } else if (source == Source::MOTION_TRACKING) {
    device_->StartMotionTracking();
  } else if (source == Source::ALL) {
    Start(Source::VIDEO_STREAMING);
    Start(Source::MOTION_TRACKING);
  } else {
    LOG(ERROR) << "Unsupported source :(";
  }
}

void API::Stop(const Source &source) {
  if (source == Source::VIDEO_STREAMING) {
    synthetic_->StopVideoStreaming();
  } else if (source == Source::MOTION_TRACKING) {
    device_->StopMotionTracking();
  } else if (source == Source::ALL) {
    Stop(Source::MOTION_TRACKING);
    // Must stop motion tracking before video streaming and sleep a moment here
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    Stop(Source::VIDEO_STREAMING);
  } else {
    LOG(ERROR) << "Unsupported source :(";
  }
}

void API::WaitForStreams() {
  if (correspondence_) {
    correspondence_->WaitForStreams();
  } else {
    synthetic_->WaitForStreams();
  }
}

void API::EnableStreamData(const Stream &stream) {
  synthetic_->EnableStreamData(stream);
}

void API::DisableStreamData(const Stream &stream) {
  synthetic_->DisableStreamData(stream);
}

void API::EnableStreamData(
    const Stream &stream, stream_switch_callback_t callback,
    bool try_tag) {
  synthetic_->EnableStreamData(stream, callback, try_tag);
}
void API::DisableStreamData(
    const Stream &stream, stream_switch_callback_t callback,
    bool try_tag) {
  synthetic_->DisableStreamData(stream, callback, try_tag);
}

api::StreamData API::GetStreamData(const Stream &stream) {
  if (correspondence_ && correspondence_->Watch(stream)) {
    return correspondence_->GetStreamData(stream);
  } else {
    return synthetic_->GetStreamData(stream);
  }
}

std::vector<api::StreamData> API::GetStreamDatas(const Stream &stream) {
  if (correspondence_ && correspondence_->Watch(stream)) {
    return correspondence_->GetStreamDatas(stream);
  } else {
    return synthetic_->GetStreamDatas(stream);
  }
}

void API::EnableMotionDatas(std::size_t max_size) {
  if (correspondence_) return;  // not cache them
  device_->EnableMotionDatas(max_size);
}

std::vector<api::MotionData> API::GetMotionDatas() {
  if (correspondence_) {
    return correspondence_->GetMotionDatas();
  } else {
    std::vector<api::MotionData> datas;
    for (auto &&data : device_->GetMotionDatas()) {
      datas.push_back({data.imu});
    }
    return datas;
  }
}

void API::EnableTimestampCorrespondence(const Stream &stream,
    bool keep_accel_then_gyro) {
  if (!dev_correspondence_enable_) {
    api_correspondence_enable_ = keep_accel_then_gyro;
  } else {
    LOG(WARNING) << "dev_correspondence_enable_ "
                    "has been set to true, "
                    "you should close it first when you want to use "
                    "api_correspondence_enable_.";
    return;
  }
  if (correspondence_ == nullptr) {
    correspondence_.reset(new Correspondence(device_, stream));
    correspondence_->KeepAccelThenGyro(keep_accel_then_gyro);
    {
      device_->DisableMotionDatas();
      if (callback_) {
        correspondence_->SetMotionCallback(callback_);
        callback_ = nullptr;
      }
    }
    using namespace std::placeholders;  // NOLINT
    device_->SetMotionCallback(
        std::bind(&Correspondence::OnMotionDataCallback,
            correspondence_.get(), _1),
        true);
    synthetic_->SetStreamDataListener(
        std::bind(&Correspondence::OnStreamDataCallback,
            correspondence_.get(), _1, _2));
  }
}

void API::EnableImuTimestampCorrespondence(bool is_enable) {
  if (!api_correspondence_enable_) {
     dev_correspondence_enable_= is_enable;
  } else {
    LOG(WARNING) << "api_correspondence_enable_ "
                    "has been set to true, "
                    "you should close it first when you want to use "
                    "dev_correspondence_enable_.";
    return;
  }
  device_->EnableImuCorrespondence(is_enable);
}

void API::EnablePlugin(const std::string &path) {
  static DL dl;
  CHECK(dl.Open(path.c_str())) << "Open plugin failed: " << path;

  plugin_version_code_t *plugin_version_code =
      dl.Sym<plugin_version_code_t>("plugin_version_code");
  LOG(INFO) << "Enable plugin success";
  LOG(INFO) << "  version code: " << plugin_version_code();
  LOG(INFO) << "  path: " << path;

  plugin_create_t *plugin_create = dl.Sym<plugin_create_t>("plugin_create");
  plugin_destroy_t *plugin_destroy = dl.Sym<plugin_destroy_t>("plugin_destroy");

  std::shared_ptr<Plugin> plugin(plugin_create(), plugin_destroy);
  plugin->OnCreate(this);

  synthetic_->SetPlugin(plugin);
}

void API::setDuplicate(bool isEnable) {
  synthetic_->setDuplicate(isEnable);
}

void API::SetDisparityComputingMethodType(
      const DisparityComputingMethod &MethodType) {
  synthetic_->SetDisparityComputingMethodType(MethodType);
}

void API::SetRectifyAlpha(const double &alpha) {
  synthetic_->SetRectifyAlpha(alpha);
}

std::shared_ptr<Device> API::device() {
  return device_;
}

// TODO(Kalman): Call this function in the appropriate place
void API::CheckImageParams() {
  if (device_ != nullptr) {
    if (device_->CheckImageParams()) {
      LOG(FATAL) << "Image params not found, but we need it to process the "
                    "images. Please `make tools` and use `img_params_writer` "
                    "to write the image params. If you update the SDK from "
                    "1.x, the `SN*.conf` is the file contains them. Besides, "
                    "you could also calibrate them by yourself. Read the guide "
                    "doc (https://github.com/slightech/MYNT-EYE-SDK-2-Guide) "
                    "to learn more.";
    }
  }
}

void API::EnableProcessMode(const ProcessMode& mode) {
  EnableProcessMode(static_cast<std::int32_t>(mode));
}

void API::EnableProcessMode(const std::int32_t& mode) {
  device_->EnableProcessMode(mode);
}

std::shared_ptr<struct CameraROSMsgInfoPair> API::GetCameraROSMsgInfoPair() {
  return synthetic_->GetCameraROSMsgInfoPair();
}

bool API::ConfigDisparityFromFile(const std::string& config_file) {
  return synthetic_->ConfigDisparityFromFile(config_file);
}

bool API::IsDefaultIntrinsics() {
  if (device_ != nullptr) {
    return device_->CheckImageParams();
  }

  return false;
}

MYNTEYE_END_NAMESPACE
