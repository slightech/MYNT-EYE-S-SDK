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

#include <thread>
#include <vector>
#include <atomic>
#include <mutex>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>

#include "mynteye/logger.h"
#include "mynteye/uvc/uvc.h"
#include "libuvc/libuvc.h"
#include "AVfoundationCamera.h"

#define MYNT_DEV_NAME "MYNT-EYE-S"

// #define ENABLE_DEBUG_SPAM

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

struct context {
  context() {
    VLOG(2) << __func__;
  }

  ~context() {
    VLOG(2) << __func__;
  }
};

struct buffer {
  void *start;
  size_t length;
};

void printConfig(std::vector<CameraConfig> cfg_list) {
  if (cfg_list.size() == 0) return;

  int device = -1;
  int width = -1;
  int height = -1;
  float fps = -1;
  int format = -1;
  int frame_mode = -1;

  for (unsigned int i=0; i < cfg_list.size(); i++) {
    if (cfg_list[i].device != device) {
      if (device >= 0) printf("\b fps\n");
      device = cfg_list[i].device;
      if (strncmp(cfg_list[i].name,
          MYNT_DEV_NAME, strlen(MYNT_DEV_NAME)) == 0) {
        printf("*");
      } else {
        printf(" ");
      }
      printf(" %d: %s\n", cfg_list[i].device, cfg_list[i].name);
      format = -1;
    }

    if ((cfg_list[i].cam_format != format) ||
        (cfg_list[i].frame_mode != frame_mode)) {
      if (format >= 0) printf("\b fps\n");
      format = cfg_list[i].cam_format;
      if (cfg_list[i].frame_mode < 0) {
        printf("    format: %s", fstr[cfg_list[i].cam_format]);
      } else {
        printf("    format7_%d: %s", cfg_list[i].frame_mode,
            fstr[cfg_list[i].cam_format]);
      }
      // if(cfg_list[i].compress) printf(" (default)");
      width = height = fps = -1;
      printf("\n");
    }

    if ((cfg_list[i].cam_width != width) ||
        (cfg_list[i].cam_height != height)) {
      if (width > 0) printf("\b fps\n");
      printf("      %dx%d ", cfg_list[i].cam_width, cfg_list[i].cam_height);
      width = cfg_list[i].cam_width;
      height = cfg_list[i].cam_height;
      fps = INT_MAX;
    }

    if (cfg_list[i].frame_mode >= 0) {
      printf("max|");
    } else if (cfg_list[i].cam_fps != fps) {
      if (static_cast<int>(cfg_list[i].cam_fps) == cfg_list[i].cam_fps) {
        printf("%d|", static_cast<int>(cfg_list[i].cam_fps));
      } else {
        printf("%.1f|", cfg_list[i].cam_fps);
      }
      fps = cfg_list[i].cam_fps;
    }
  }
  printf("\b fps\n");
}

std::vector<CameraConfig> findDevicesConfig() {
  std::vector<CameraConfig> dev_list;
  std::vector<CameraConfig> sys_list = AVfoundationCamera::getCameraConfigs();
  dev_list.insert(dev_list.end(), sys_list.begin(), sys_list.end());
  return dev_list;
}

void pv_sleep(int ms = 1) {
  usleep(ms*1000);
}

struct device;

struct device : public AVfoundationCamera{
  const std::shared_ptr<context> parent;

  static std::vector <struct device*> s_devices;
  int _vendor_id = -1;
  int _product_id = -1;

  CameraConfig _config;
  video_channel_callback callback = nullptr;

  bool is_capturing = false;
  std::vector<buffer> buffers;

  std::thread thread;
  volatile bool pause_ = false;
  volatile bool stop = false;

  unsigned char *camera_buffer = NULL;
  unsigned char *camera_write_buffer = NULL;

  std::mutex _devices_mutex;

  CameraConfig get_camera_config() const {
    return _config;
  }

  device(std::shared_ptr<context> parent, const CameraConfig &config)
      : AVfoundationCamera(& const_cast<CameraConfig&>(config)),
      parent(parent), _config(config) {
    VLOG(2) << __func__;
    if (strncmp(config.name, MYNT_DEV_NAME, strlen(MYNT_DEV_NAME)) == 0) {
      _vendor_id = MYNTEYE_VID;  // 0x04B4
      _product_id = MYNTEYE_PID;  // 0x00F9
      open();
    }

    std::lock_guard<std::mutex> lock(_devices_mutex);
    s_devices.push_back(this);
  }

  ~device() {
    VLOG(2) << __func__;
    std::lock_guard<std::mutex> lock(_devices_mutex);
    for (unsigned int i = 0 ; i < s_devices.size() ; i++) {
      if (this == s_devices[i]) {
        s_devices.erase(s_devices.begin()+i);
      }
    }
    pause_ = true;
    teardown();
  }

  void open() {
    setup_camera();
  }

  void start_capture() {
    if (is_capturing) {
      LOG(WARNING) << "Start capture failed, is capturing already";
      return;
    }
    is_capturing = true;
    startCamera();
  }

  void stop_capture() {
    if (!is_capturing)
      return;
    is_capturing = false;
    stopCamera();
  }

  std::string get_name() const {
    return std::string(_config.name);
  }
  int get_vendor_id() const {
    return _vendor_id;
  }
  int get_product_id() const {
    return _product_id;
  }
  std::string get_video_name() const {
    return std::string("[video name is not supported on osx]");
  }

  void setup_camera() {
    if (initCamera()) {
      printInfo();
    } else {
      LOG(ERROR) << "could not initialize selected camera";
      closeCamera();
      return;
    }
  }

  void poll() {
    if (is_capturing) {
      camera_buffer = getFrame();
      if (camera_buffer != NULL) {
        if (callback) {
          callback(camera_buffer, [this]() mutable {
            // todo
          });
        }
        pv_sleep();
      } else {
        if ((is_capturing) && (!stillRunning())) {
          LOG(ERROR) << "error status in camera running.";
        } else { pv_sleep(); }
      }
    } else { pv_sleep(5); }
  }

  void pause(bool pause) {
    pause_ = pause;
  }

  void start_streaming() {
    if (!callback) {
      LOG(WARNING) << __func__ << " failed: video_channel_callback is empty";
      return;
    }
    start_capture();
    showSettingsDialog(false);
    thread = std::thread([this]() {
      while (!stop)
        poll();
    });
  }

  void stop_streaming() {
    if (thread.joinable()) {
      stop = true;
      thread.join();
      stop = false;
      stop_capture();
    }
  }

  void teardown() {
    pause_ = true;
    stopCamera();
    closeCamera();
  }
};

std::vector <struct device*> device::s_devices;

// Enumerate devices
MYNTEYE_API std::shared_ptr<context> create_context() {
  return std::make_shared<context>();
}

MYNTEYE_API std::vector<std::shared_ptr<device>> query_devices(
    std::shared_ptr<context> context) {

  std::vector<std::shared_ptr<device>> devices;
  auto camerasConfig = findDevicesConfig();
  printConfig(camerasConfig);
  for (unsigned int i = 0; i < camerasConfig.size(); i++) {
    auto dev = std::make_shared<device>(context, camerasConfig[i]);
    devices.push_back(dev);
  }
  return devices;
}

// Static device properties
MYNTEYE_API std::string get_name(const device &device) {
  return device.get_name();
}
MYNTEYE_API int get_vendor_id(const device &device) {
  return device.get_vendor_id();
}
MYNTEYE_API int get_product_id(const device &device) {
  return device.get_product_id();
}

MYNTEYE_API std::string get_video_name(const device &device) {
  return device.get_video_name();
}

MYNTEYE_API bool pu_control_range(
    const device &device, Option option, int32_t *min, int32_t *max,
    int32_t *def) {
  struct device &dev = const_cast<struct device &> (device);
  switch (option) {
    case Option::GAIN: {
      *max = dev.getMaxCameraSetting(GAIN);
      *min = dev.getMinCameraSetting(GAIN);
      *def = dev.getDefaultCameraSetting(GAIN);
    } break;
    case Option::BRIGHTNESS: {
      *max = dev.getMaxCameraSetting(BRIGHTNESS);
      *min = dev.getMinCameraSetting(BRIGHTNESS);
      *def = dev.getDefaultCameraSetting(BRIGHTNESS);
    } break;
    case Option::CONTRAST: {
      *max = dev.getMaxCameraSetting(CONTRAST);
      *min = dev.getMinCameraSetting(CONTRAST);
      *def = dev.getDefaultCameraSetting(CONTRAST);
    } break;
    default: {
      LOG(WARNING) << __func__
                   << " failed: the option "
                   << static_cast<int>(option) << "is invalid!";
    } break;
  }
  return true;
}
MYNTEYE_API bool pu_control_query(
    const device &device, Option option, pu_query query, int32_t *value) {
  struct device &dev = const_cast<struct device &> (device);
  if (query == PU_QUERY_SET) {
    switch (option) {
      case Option::GAIN:
        return dev.setCameraSetting(GAIN, *value);
      case Option::BRIGHTNESS:
        return dev.setCameraSetting(BRIGHTNESS, *value);
      case Option::CONTRAST:
        return dev.setCameraSetting(CONTRAST, *value);
      default: {
        LOG(WARNING) << __func__
                    << " failed: the option "
                    << static_cast<int>(option) << "is invalid!";
      } return false;
    }
  } else if (query == PU_QUERY_GET) {
    switch (option) {
      case Option::GAIN:
        *value = dev.getCameraSetting(GAIN);
      case Option::BRIGHTNESS:
        *value = dev.getCameraSetting(BRIGHTNESS);
      case Option::CONTRAST:
        *value = dev.getCameraSetting(CONTRAST);
      default: {
        LOG(WARNING) << __func__
                    << " failed: the option "
                    << static_cast<int>(option) << "is invalid!";
      } return false;
    }
    return true;
  } else {
    LOG(WARNING) << __func__
                   << " failed: the query "
                   << static_cast<int>(query) << "is invalid!";
    return false;
  }
  return true;
}

// Access XU (Extension Unit) controls , Not supported on osx
MYNTEYE_API bool xu_control_range(
    const device &/*device*/, const xu &/*xu*/,
    uint8_t /*selector*/, uint8_t /*id*/,
    int32_t * /*min*/, int32_t * /*max*/, int32_t * /*def*/) {
  // not supported on osx
  LOG(WARNING) << __func__ << " failed: this API is not supported on osx";
  return false;
}
MYNTEYE_API bool xu_control_query(  // XU_QUERY_SET, XU_QUERY_GET
    const device &/*device*/, const xu &/*xu*/,
    uint8_t /*selector*/, xu_query /*query*/,
    uint16_t /*size*/, uint8_t * /*data*/) {
  // not supported on osx
  LOG(WARNING) << __func__ << " failed: this API is not supported on osx";
  return false;
}

MYNTEYE_API void set_device_mode(
    device &device, int width, int height, int fourcc, int fps,  // NOLINT
    video_channel_callback callback) {
  if (width != device.get_camera_config().cam_width ||
      height != device.get_camera_config().cam_height) {
    LOG(ERROR) << __func__
                 << width << "x" << height << "|"
                 << device.get_camera_config().cam_width << "x"
                 << device.get_camera_config().cam_height << std::endl
                 << " failed: the different size can't be set to get frame.";
    return;
  }
  if (fps < device.getFps()) {
    LOG(WARNING) << __func__
                 << " The fps requied is less than the sdk support.";
  } else if (fps > device.getFps()) {
    LOG(WARNING) << __func__
                 << " The fps requied is more than the"
                 << " sdk max support, use the max fps instead.";
  }

  if (fourcc != static_cast<int>(Format::YUYV)) {
    LOG(WARNING) << __func__
                 << " sdk max just support yuyv video mode.";
  }
  device.callback = callback;
}
MYNTEYE_API void start_streaming(device &device, int num_transfer_bufs) { // NOLINT
  device.start_streaming();
}
MYNTEYE_API void stop_streaming(device &device) { // NOLINT
  device.stop_streaming();
}

}  // namespace uvc

MYNTEYE_END_NAMESPACE
