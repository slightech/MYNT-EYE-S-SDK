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
#include "mynteye/logger.h"
#include "mynteye/uvc/uvc.h"
#include "libuvc/libuvc.h"

#include "mynteye/uvc/macosx/uvc_osx_internal.h"

// #define ENABLE_DEBUG_SPAM

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

static void check(const char *call, uvc_error_t status) {
  LOG_IF(FATAL, status < 0)
      << call << "(...) returned " << uvc_strerror(status);
}

#define CALL_UVC_WITHOUT_CHECK(name, ...) name(__VA_ARGS__)
#define CALL_UVC(name, ...) check(#name, name(__VA_ARGS__))

struct context {
  uvc_context_t *ctx;

  context() : ctx(nullptr) {
    VLOG(2) << __func__;
    CALL_UVC(uvc_init, &ctx, nullptr);
  }

  ~context() {
    VLOG(2) << __func__;
    if (ctx)
      uvc_exit(ctx);
  }
};

/** UVC request code (A.8) */
enum uvc_req_code {
    UVC_RC_UNDEFINED = 0x00,
    UVC_SET_CUR = 0x01,
    UVC_GET_CUR = 0x81,
    UVC_GET_MIN = 0x82,
    UVC_GET_MAX = 0x83,
    UVC_GET_RES = 0x84,
    UVC_GET_LEN = 0x85,
    UVC_GET_INFO = 0x86,
    UVC_GET_DEF = 0x87,
    UVC_REQ_TYPE_GET = 0xa1,
    UVC_REQ_TYPE_SET = 0x21
};

struct device;

struct device {
  const std::shared_ptr<context> parent;

  uvc_device_t *uvcdevice = nullptr;
  uvc_device_handle_t *handle = nullptr;
  /** Serial number (null if unavailable) */
  std::string serialNumber = "";
  /** Device-reported manufacturer name (or null) */
  std::string manufacturer = "";
  /** Device-reporter product name (or null) */
  std::string product = "";
  uvc_device_info_t info;
  int width, height, format, fps;
  int vid, pid;
  video_channel_callback callback = nullptr;
  static std::vector <struct device*> s_devices;

  std::mutex _devices_mutex;

  device(std::shared_ptr<context> parent, uvc_device_t *uvcdevice)
      : parent(parent), uvcdevice(uvcdevice) {
    VLOG(2) << __func__;
    open();

    uvc_device_descriptor_t *desc;
    CALL_UVC(uvc_get_device_descriptor, uvcdevice, &desc);

    serialNumber = std::string(desc->serialNumber);
    manufacturer = std::string(desc->manufacturer);
    product = std::string(desc->product);

    vid = desc->idVendor;
    pid = desc->idProduct;
    uvc_free_device_descriptor(desc);
    std::lock_guard<std::mutex> lock(_devices_mutex);
    s_devices.push_back(this);
  }

  ~device() {
    VLOG(2) << __func__;
    if (handle)
      uvc_close(handle);
    if (uvcdevice)
      uvc_unref_device(uvcdevice);
    std::lock_guard<std::mutex> lock(_devices_mutex);
    for(unsigned long i = 0 ; i < s_devices.size() ; i++) {
      if(this == s_devices[i]) {
        s_devices.erase(s_devices.begin()+i);
      }
    }
  }

  void open() {
    if (!handle)
      CALL_UVC(uvc_open, uvcdevice, &handle);
  }

  void set_format(
      int width, int height, int format, int fps,
      video_channel_callback callback) {
    this->width = width;
    this->height = height;
    this->format = format;
    this->fps = fps;
    this->callback = callback;
  }

  static void uvc_frame_callback (struct uvc_frame *frame, void *user_ptr)
  {
    for(unsigned long i = 0 ; i < s_devices.size() ; i++) {
      if(user_ptr == (void*)s_devices[i]) {
        printf("bingo\n");
      }
    }
  }

  // int32_t get_data_usb( uvc_req_code action, int control, int unit) const {
  //     unsigned char buffer[4];

  //     int status = libusb_control_transfer(handle->usb_devh,
  //                                           UVC_REQ_TYPE_GET,
  //                                           action,
  //                                           control << 8,
  //                                           unit << 8 | (1),// _interface
  //                                           buffer,
  //                                           sizeof(int32_t), 0);
  //     MYNTEYE_UNUSED(status);
  //     if (status < 0) throw std::runtime_error(
  //                 to_string() << "libusb_control_transfer(...) returned "
  //                             << libusb_error_name(status));

  //     if (status != sizeof(int32_t))
  //         throw std::runtime_error("insufficient data read from USB");

  //     return DW_TO_INT(buffer);
  // }

  // void set_data_usb( uvc_req_code action, int control, int unit, int value) const {
  //     unsigned char buffer[4];

  //     INT_TO_DW(value, buffer);

  //     int status = libusb_control_transfer(handle->usb_devh,
  //                                           UVC_REQ_TYPE_SET,
  //                                           action,
  //                                           control << 8,
  //                                           unit << 8 | (1),// _interface
  //                                           buffer,
  //                                           sizeof(int32_t), 0);

  //     if (status < 0) throw std::runtime_error(
  //                 to_string() << "libusb_control_transfer(...) returned "
  //                             << libusb_error_name(status));

  //     if (status != sizeof(int32_t))
  //         throw std::runtime_error("insufficient data writen to USB");
  // }
};

std::vector <struct device*> device::s_devices;

std::shared_ptr<context> create_context() {
  return std::make_shared<context>();
}

std::vector<std::shared_ptr<device>> query_devices(
    std::shared_ptr<context> context) {
  std::vector<std::shared_ptr<device>> devices;

  uvc_device_t **list;
  CALL_UVC(uvc_get_device_list, context->ctx, &list);
  for (auto it = list; *it; ++it) {
    try {
      auto dev = std::make_shared<device>(context, *it);
      devices.push_back(dev);
    } catch (std::runtime_error &e) {
      LOG(WARNING) << "usb:" << static_cast<int>(uvc_get_bus_number(*it)) << ':'
                   << static_cast<int>(uvc_get_device_address(*it)) << ": "
                   << e.what();
    }
  }
  uvc_free_device_list(list, 1);

  return devices;
}

int get_vendor_id(const device &device) {
  return device.vid;
}

int get_product_id(const device &device) {
  return device.pid;
}

std::string get_name(const device &device) {
  return device.serialNumber + "/" + device.manufacturer + "/" + device.product;
}

std::string get_video_name(const device &device) {
  return device.serialNumber + "/" + device.manufacturer + "/" + device.product;
}

// class uvc_device
//         {
//         public:
//             virtual void probe_and_commit(stream_profile profile, frame_callback callback, int buffers = DEFAULT_V4L2_FRAME_BUFFERS) = 0;
//             virtual void stream_on(std::function<void(const notification& n)> error_handler = [](const notification& n){}) = 0;
//             virtual void start_callbacks() = 0;
//             virtual void stop_callbacks() = 0;
//             virtual void close(stream_profile profile) = 0;

//             virtual void set_power_state(power_state state) = 0;
//             virtual power_state get_power_state() const = 0;

//             virtual void init_xu(const extension_unit& xu) = 0;
//             virtual bool set_xu(const extension_unit& xu, uint8_t ctrl, const uint8_t* data, int len) = 0;
//             virtual bool get_xu(const extension_unit& xu, uint8_t ctrl, uint8_t* data, int len) const = 0;
//             virtual control_range get_xu_range(const extension_unit& xu, uint8_t ctrl, int len) const = 0;

//             virtual bool get_pu(rs2_option opt, int32_t& value) const = 0;
//             virtual bool set_pu(rs2_option opt, int32_t value) = 0;
//             virtual control_range get_pu_range(rs2_option opt) const = 0;

//             virtual std::vector<stream_profile> get_profiles() const = 0;

//             virtual void lock() const = 0;
//             virtual void unlock() const = 0;

//             virtual std::string get_device_location() const = 0;
//             virtual usb_spec  get_usb_specification() const = 0;

//             virtual ~uvc_device() = default;

//         protected:
//             std::function<void(const notification& n)> _error_handler;
//         };

bool pu_control_range(
    const device &device, Option option, int32_t *min, int32_t *max,
    int32_t *def) {
  // TODO(JohnZhao)
  MYNTEYE_UNUSED(device)
  MYNTEYE_UNUSED(option)
  MYNTEYE_UNUSED(min)
  MYNTEYE_UNUSED(max)
  MYNTEYE_UNUSED(def)
  // device.uvcdevice -> set_pu(option, *def);
  return false;
}

bool pu_control_query(
    const device &device, Option option, pu_query query, int32_t *value) {
  // TODO(JohnZhao)
  MYNTEYE_UNUSED(device)
  MYNTEYE_UNUSED(option)
  MYNTEYE_UNUSED(query)
  MYNTEYE_UNUSED(value)
  return false;
}

bool xu_control_range(
    const device &device, const xu &xu, uint8_t selector, uint8_t id, int32_t *min,
    int32_t *max, int32_t *def) {
  // TODO(JohnZhao)
  MYNTEYE_UNUSED(device)
  MYNTEYE_UNUSED(xu)
  MYNTEYE_UNUSED(selector)
  MYNTEYE_UNUSED(id)
  MYNTEYE_UNUSED(min)
  MYNTEYE_UNUSED(max)
  MYNTEYE_UNUSED(def)
  return false;
}

bool xu_control_query(
    const device &device, const xu &xu, uint8_t selector, xu_query query,
    uint16_t size, uint8_t *data) {
  // TODO(JohnZhao)
  MYNTEYE_UNUSED(device)
  MYNTEYE_UNUSED(xu)
  MYNTEYE_UNUSED(selector)
  MYNTEYE_UNUSED(query)
  MYNTEYE_UNUSED(size)
  MYNTEYE_UNUSED(data)
  return false;
}

void set_device_mode(
    device &device, int width, int height, int format, int fps,  // NOLINT
    video_channel_callback callback) {
  device.set_format(width, height, format, fps, callback);
}

void start_streaming(device &device, int num_transfer_bufs) {  // NOLINT
  // MYNTEYE_UNUSED(device)
  // MYNTEYE_UNUSED(num_transfer_bufs)

//   typedef struct uvc_stream_ctrl {
//   uint16_t bmHint;
//   uint8_t bFormatIndex;
//   uint8_t bFrameIndex;
//   uint32_t dwFrameInterval;
//   uint16_t wKeyFrameRate;
//   uint16_t wPFrameRate;
//   uint16_t wCompQuality;
//   uint16_t wCompWindowSize;
//   uint16_t wDelay;
//   uint32_t dwMaxVideoFrameSize;
//   uint32_t dwMaxPayloadTransferSize;
//   uint32_t dwClockFrequency;
//   uint8_t bmFramingInfo;
//   uint8_t bPreferredVersion;
//   uint8_t bMinVersion;
//   uint8_t bMaxVersion;
//   uint8_t bInterfaceNumber;
// } uvc_stream_ctrl_t;

  // uvc_error_t uvc_get_stream_ctrl_format_size(
  //   uvc_device_handle_t *devh,
  //   uvc_stream_ctrl_t *ctrl,
  //   enum uvc_frame_format format,
  //   int width, int height,
  //   int fps
  //   );

  uvc_stream_ctrl_t ctrl_st = {};

  /** Color coding of stream, transport-independent
 * @ingroup streaming
 */
// enum uvc_frame_format {
//   UVC_FRAME_FORMAT_UNKNOWN = 0,
//   /** Any supported format */
//   UVC_FRAME_FORMAT_ANY = 0,
//   UVC_FRAME_FORMAT_UNCOMPRESSED,
//   UVC_FRAME_FORMAT_COMPRESSED,
//   /** YUYV/YUV2/YUV422: YUV encoding with one luminance value per pixel and
//    * one UV (chrominance) pair for every two pixels.
//    */
//   UVC_FRAME_FORMAT_YUYV,
//   UVC_FRAME_FORMAT_UYVY,
//   /** 24-bit RGB */
//   UVC_FRAME_FORMAT_RGB,
//   UVC_FRAME_FORMAT_BGR,
//   /** Motion-JPEG (or JPEG) encoded images */
//   UVC_FRAME_FORMAT_MJPEG,
//   /** Greyscale images */
//   UVC_FRAME_FORMAT_GRAY8,
//   UVC_FRAME_FORMAT_GRAY16,
//   /* Raw colour mosaic images */
//   UVC_FRAME_FORMAT_BY8,
//   UVC_FRAME_FORMAT_BA81,
//   UVC_FRAME_FORMAT_SGRBG8,
//   UVC_FRAME_FORMAT_SGBRG8,
//   UVC_FRAME_FORMAT_SRGGB8,
//   UVC_FRAME_FORMAT_SBGGR8,
//   /** Number of formats understood */
//   UVC_FRAME_FORMAT_COUNT,
// };
  CALL_UVC(uvc_get_stream_ctrl_format_size,
          device.handle,
          &ctrl_st,
          UVC_FRAME_FORMAT_ANY, // UVC_FRAME_FORMAT_YUYV, //(enum uvc_frame_format)device.format,
          device.width,
          device.height,
          device.fps);

 /** A callback function to handle incoming assembled UVC frames
 * @ingroup streaming
 */
// typedef void(uvc_frame_callback_t)(struct uvc_frame *frame, void *user_ptr);
  // uvc_frame_callback_t *cb = nullptr;

  CALL_UVC(uvc_start_streaming,
          device.handle,
          &ctrl_st,
          device::uvc_frame_callback,
          &device,
          num_transfer_bufs);
  
  printf("begin\n");

//   uvc_error_t uvc_start_streaming(
//     uvc_device_handle_t *devh,
//     uvc_stream_ctrl_t *ctrl,
//     uvc_frame_callback_t *cb,
//     void *user_ptr,
//     uint8_t flags);
}

void stop_streaming(device &device) {  // NOLINT
  // MYNTEYE_UNUSED(device)

  CALL_UVC_WITHOUT_CHECK(uvc_stop_streaming, device.handle);
}

}  // namespace uvc

MYNTEYE_END_NAMESPACE
