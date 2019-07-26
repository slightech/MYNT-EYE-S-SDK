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
#include "mynteye/uvc/uvc.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <linux/videodev2.h>

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

#define LOG_ERROR(severity, str)                                           \
  do {                                                                     \
    LOG(severity) << str << " error " << errno << ", " << strerror(errno); \
  } while (0)

#define NO_DATA_MAX_COUNT 200
#define LIVING_MAX_COUNT 9000

int no_data_count = 0;
int living_count = 0;
/*
class device_error : public std::exception {
 public:
  explicit device_error(const std::string &what_arg) noexcept
      : what_message_(std::move(what_arg)) {}
  explicit device_error(const char *what_arg) noexcept
      : what_message_(std::move(what_arg)) {}

  const char *what() const noexcept {
    return what_message_.c_str();
  }
 private:
  std::string what_message_;
};
*/

struct throw_error {
  throw_error() = default;

  explicit throw_error(const std::string &s) {
    ss << s;
  }

  ~throw_error() noexcept(false) {
    throw std::runtime_error(ss.str());
    // throw device_error(ss.str());
  }

  template<class T>
  throw_error &operator<<(const T &val) {
    ss << val;
    return *this;
  }

  std::ostringstream ss;
};

static int xioctl(int fh, int request, void *arg) {
  int r;
  do {
    r = ioctl(fh, request, arg);
  } while (r < 0 && errno == EINTR);
  return r;
}

struct buffer {
  void *start;
  size_t length;
};

struct context {
  context() {
    VLOG(2) << __func__;
  }

  ~context() {
    VLOG(2) << __func__;
  }
};

struct device {
  const std::shared_ptr<context> parent;

  std::string dev_name;  // Device name (typically of the form /dev/video*)

  std::string name;  // Device description name
  int vid, pid, mi;  // Vendor ID, product ID, and multiple interface index
  int fd = -1;       // File descriptor for this device

  int width, height, format, fps;
  video_channel_callback callback = nullptr;

  bool is_capturing = false;
  std::vector<buffer> buffers;

  std::thread thread;
  volatile bool stop = false;

  device(std::shared_ptr<context> parent, const std::string &name)
      : parent(parent), dev_name("/dev/" + name) {
    VLOG(2) << __func__ << ": " << dev_name;

    struct stat st;
    if (stat(dev_name.c_str(), &st) < 0) {  // file status
      throw_error() << "Cannot identify '" << dev_name << "': " << errno << ", "
                    << strerror(errno);
    }
    if (!S_ISCHR(st.st_mode)) {  // character device?
      throw_error() << dev_name << " is no device";
    }

    if (!(std::ifstream("/sys/class/video4linux/" + name + "/name") >>
          this->name))
      throw_error() << "Failed to read name";

    std::string modalias;
    if (!(std::ifstream(
              "/sys/class/video4linux/" + name + "/device/modalias") >>
          modalias))
      throw_error() << "Failed to read modalias";
    if (modalias.size() < 14 || modalias.substr(0, 5) != "usb:v" ||
        modalias[9] != 'p')
      throw_error() << "Not a usb format modalias";
    if (!(std::istringstream(modalias.substr(5, 4)) >> std::hex >> vid))
      throw_error() << "Failed to read vendor ID";
    if (!(std::istringstream(modalias.substr(10, 4)) >> std::hex >> pid))
      throw_error() << "Failed to read product ID";
    if (!(std::ifstream(
              "/sys/class/video4linux/" + name + "/device/bInterfaceNumber") >>
          std::hex >> mi))
      throw_error() << "Failed to read interface number";

    fd = open(dev_name.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd < 0) {
      throw_error() << "Cannot open '" << dev_name << "': " << errno << ", "
                    << strerror(errno);
    }

    v4l2_capability cap;
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
      if (errno == EINVAL)
        throw_error() << dev_name << " is no V4L2 device";
      else
        throw_error() << "VIDIOC_QUERYCAP error " << errno << ", "
                      << strerror(errno);
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
      throw_error() << dev_name + " is no video capture device";
    if (!(cap.capabilities & V4L2_CAP_STREAMING))
      throw_error() << dev_name + " does not support streaming I/O";

    // Select video input, video standard and tune here.
    v4l2_cropcap cropcap;
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_CROPCAP, &cropcap) == 0) {
      v4l2_crop crop;
      crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      crop.c = cropcap.defrect;  // reset to default
      if (xioctl(fd, VIDIOC_S_CROP, &crop) < 0) {
        switch (errno) {
          case EINVAL:
            break;  // Cropping not supported
          default:
            break;  // Errors ignored
        }
      }
    } else {
      throw_error() << dev_name + " is no video capture device";
    }  // Errors ignored
  }

  ~device() {
    VLOG(2) << __func__;
    stop_streaming();
    no_data_count = 0;
    if (fd != -1 && close(fd) < 0) {
      LOG_ERROR(WARNING, "close");
    }
  }

  bool pu_control_range(
      uint32_t id, int32_t *min, int32_t *max, int32_t *def) const {
    struct v4l2_queryctrl query;
    query.id = id;
    if (xioctl(fd, VIDIOC_QUERYCTRL, &query) < 0) {
      LOG_ERROR(WARNING, "pu_control_range failed");
      return false;
    }
    if (min)
      *min = query.minimum;
    if (max)
      *max = query.maximum;
    if (def)
      *def = query.default_value;
    return true;
  }

  bool pu_control_query(uint32_t id, int query, int32_t *value) const {
    CHECK_NOTNULL(value);
    struct v4l2_control control = {id, *value};
    if (xioctl(fd, query, &control) < 0) {
      LOG_ERROR(WARNING, "pu_control_query failed");
      return false;
    }
    *value = control.value;
    return true;
  }

  bool xu_control_query(
      const xu &xu, uint8_t selector, uint8_t query, uint16_t size,
      uint8_t *data) const {
    CHECK_NOTNULL(data);
    uvc_xu_control_query q = {xu.unit, selector, query, size, data};
    if (xioctl(fd, UVCIOC_CTRL_QUERY, &q) < 0) {
      LOG_ERROR(WARNING, "xu_control_query failed");
      return false;
    }
    return true;
  }

  void set_format(
      int width, int height, int fourcc, int fps,
      video_channel_callback callback) {
    this->width = width;
    this->height = height;
    this->format = fourcc;
    this->fps = fps;
    this->callback = callback;
  }

  void start_capture() {
    if (is_capturing) {
      LOG(WARNING) << "Start capture failed, is capturing already";
      return;
    }

    v4l2_format fmt;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = format;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    // fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0)
      LOG_ERROR(FATAL, "VIDIOC_S_FMT");

    v4l2_streamparm parm;
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_G_PARM, &parm) < 0)
      LOG_ERROR(FATAL, "VIDIOC_G_PARM");
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = fps;
    if (xioctl(fd, VIDIOC_S_PARM, &parm) < 0)
      LOG_ERROR(FATAL, "VIDIOC_S_PARM");

    // Init memory mapped IO
    v4l2_requestbuffers req;
    req.count = 24;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
      if (errno == EINVAL)
        LOG(FATAL) << dev_name << " does not support memory mapping";
      else
        LOG_ERROR(FATAL, "VIDIOC_REQBUFS");
    }
    if (req.count < 2) {
      LOG(FATAL) << "Insufficient buffer memory on " << dev_name;
    }

    buffers.resize(req.count);
    for (size_t i = 0; i < buffers.size(); ++i) {
      v4l2_buffer buf;
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0)
        LOG_ERROR(FATAL, "VIDIOC_QUERYBUF");

      buffers[i].length = buf.length;
      buffers[i].start = mmap(
          NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
          buf.m.offset);
      if (buffers[i].start == MAP_FAILED)
        LOG_ERROR(FATAL, "mmap");
    }

    // Start capturing
    for (size_t i = 0; i < buffers.size(); ++i) {
      v4l2_buffer buf;
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if (xioctl(fd, VIDIOC_QBUF, &buf) < 0)
        LOG_ERROR(FATAL, "VIDIOC_QBUF");
    }

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    for (int i = 0; i < 10; ++i) {
      if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      } else {
        is_capturing = true;
        return;
      }
    }
    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0)
      LOG_ERROR(FATAL, "VIDIOC_STREAMON");

    is_capturing = true;
  }

  void stop_capture() {
    if (!is_capturing)
      return;

    // Stop streamining
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMOFF, &type) < 0)
      LOG_ERROR(WARNING, "VIDIOC_STREAMOFF");

    for (size_t i = 0; i < buffers.size(); i++) {
      if (munmap(buffers[i].start, buffers[i].length) < 0)
        LOG_ERROR(WARNING, "munmap");
    }

    // Close memory mapped IO
    struct v4l2_requestbuffers req;
    req.count = 0;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
      if (errno == EINVAL)
        LOG(ERROR) << dev_name << " does not support memory mapping";
      else
        LOG_ERROR(WARNING, "VIDIOC_REQBUFS");
    }

    is_capturing = false;
  }

  void poll() {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    struct timeval tv = {0, 10000};

    if (select(fd + 1, &fds, NULL, NULL, &tv) < 0) {
      if (errno == EINTR)
        return;
      LOG_ERROR(FATAL, "select");
    }

    if (FD_ISSET(fd, &fds)) {
      v4l2_buffer buf;
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
        if (errno == EAGAIN)
          return;
        LOG_ERROR(FATAL, "VIDIOC_DQBUF");
      }

      if (callback) {
        callback(buffers[buf.index].start, [buf, this]() mutable {
          if (xioctl(fd, VIDIOC_QBUF, &buf) < 0)
            throw_error("VIDIOC_QBUF");
        });
        if (living_count < LIVING_MAX_COUNT) {
          living_count++;
        } else {
          living_count = 0;
          // LOG(INFO) << "UVC pulse detection,Please ignore.";
        }
      }

      no_data_count = 0;
    } else {
      no_data_count++;
    }

    if (no_data_count > NO_DATA_MAX_COUNT) {
      no_data_count = 0;
      living_count = 0;
      LOG(WARNING) << __func__
                   << " failed: v4l2 get stream time out, Try to reboot!";
      stop_capture();
      start_capture();
    }
  }

  void start_streaming() {
    if (!callback) {
      LOG(WARNING) << __func__ << " failed: video_channel_callback is empty";
      return;
    }

    start_capture();

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
};

std::shared_ptr<context> create_context() {
  return std::make_shared<context>();
}

std::vector<std::shared_ptr<device>> query_devices(
    std::shared_ptr<context> context) {
  std::vector<std::shared_ptr<device>> devices;

  DIR *dir = opendir("/sys/class/video4linux");
  if (!dir)
    LOG(FATAL) << "Cannot access /sys/class/video4linux";
  while (dirent *entry = readdir(dir)) {
    std::string name = entry->d_name;
    if (name == "." || name == "..")
      continue;

    // Resolve a pathname to ignore virtual video devices
    std::string path = "/sys/class/video4linux/" + name;
    char buff[PATH_MAX];
    ssize_t len = ::readlink(path.c_str(), buff, sizeof(buff) - 1);
    if (len != -1) {
      buff[len] = '\0';
      std::string real_path = std::string(buff);
      if (real_path.find("virtual") != std::string::npos)
        continue;
    }

    try {
      auto one_device = std::make_shared<device>(context, name);
      devices.push_back(one_device);
    } catch (const std::exception &e) {
      VLOG(2) << "Not a USB video device: " << e.what();
    }
  }
  closedir(dir);

  return devices;
}

std::string get_name(const device &device) {
  return device.name;
}

int get_vendor_id(const device &device) {
  return device.vid;
}

int get_product_id(const device &device) {
  return device.pid;
}

std::string get_video_name(const device &device) {
  return device.dev_name;
}

static uint32_t get_cid(Option option) {
  switch (option) {
    case Option::GAIN:
      return V4L2_CID_GAIN;
    case Option::BRIGHTNESS:
      return V4L2_CID_BRIGHTNESS;
    case Option::CONTRAST:
      return V4L2_CID_CONTRAST;
    default:
      LOG(FATAL) << "No v4l2 cid for " << option;
  }
}

bool pu_control_range(
    const device &device, Option option, int32_t *min, int32_t *max,
    int32_t *def) {
  return device.pu_control_range(get_cid(option), min, max, def);
}

bool pu_control_query(
    const device &device, Option option, pu_query query, int32_t *value) {
  int code;
  switch (query) {
    case PU_QUERY_SET:
      code = VIDIOC_S_CTRL;
      break;
    case PU_QUERY_GET:
      code = VIDIOC_G_CTRL;
      break;
    default:
      LOG(ERROR) << "pu_control_query request code is unaccepted";
      return false;
  }
  return device.pu_control_query(get_cid(option), code, value);
}

bool xu_control_range(
    const device &device, const xu &xu, uint8_t selector, uint8_t id,
    int32_t *min, int32_t *max, int32_t *def) {
  bool ret = true;

  std::uint8_t data[3]{static_cast<uint8_t>(id | 0x80), 0, 0};

  if (!xu_control_query(device, xu, selector, XU_QUERY_SET, 3, data)) {
    LOG(WARNING) << "xu_control_range query failed";
    ret = false;
  }

  if (xu_control_query(device, xu, selector, XU_QUERY_MIN, 3, data)) {
    *min = (data[1] << 8) | (data[2]);
  } else {
    LOG(WARNING) << "xu_control_range query min failed";
    ret = false;
  }
  if (xu_control_query(device, xu, selector, XU_QUERY_MAX, 3, data)) {
    *max = (data[1] << 8) | (data[2]);
  } else {
    LOG(WARNING) << "xu_control_range query max failed";
    ret = false;
  }
  if (xu_control_query(device, xu, selector, XU_QUERY_DEF, 3, data)) {
    *def = (data[1] << 8) | (data[2]);
  } else {
    LOG(WARNING) << "xu_control_range query def failed";
    ret = false;
  }
  return ret;
}

bool xu_control_query(
    const device &device, const xu &xu, uint8_t selector, xu_query query,
    uint16_t size, uint8_t *data) {
  uint8_t code;
  switch (query) {
    case XU_QUERY_SET:
      code = UVC_SET_CUR;
      break;
    case XU_QUERY_GET:
      code = UVC_GET_CUR;
      break;
    case XU_QUERY_MIN:
      code = UVC_GET_MIN;
      break;
    case XU_QUERY_MAX:
      code = UVC_GET_MAX;
      break;
    case XU_QUERY_DEF:
      code = UVC_GET_DEF;
      break;
    default:
      LOG(ERROR) << "xu_control_query request code is unaccepted";
      return false;
  }
  return device.xu_control_query(xu, selector, code, size, data);
}

void set_device_mode(
    device &device, int width, int height, int fourcc, int fps,  // NOLINT
    video_channel_callback callback) {
  device.set_format(width, height, fourcc, fps, callback);
}

void start_streaming(device &device, int /*num_transfer_bufs*/) {  // NOLINT
  device.start_streaming();
}

void stop_streaming(device &device) {  // NOLINT
  device.stop_streaming();
}

}  // namespace uvc

MYNTEYE_END_NAMESPACE

// Video4Linux (V4L) driver-specific documentation
//   https://linuxtv.org/downloads/v4l-dvb-apis/v4l-drivers/index.html
