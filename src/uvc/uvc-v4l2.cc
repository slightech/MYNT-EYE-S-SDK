#include "uvc/uvc.h"  // NOLINT

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <linux/videodev2.h>

#include <glog/logging.h>

#include <chrono>
#include <fstream>
#include <string>
#include <thread>

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

#define LOG_ERROR(severity, str)                                           \
  do {                                                                     \
    LOG(severity) << str << " error " << errno << ", " << strerror(errno); \
  } while (0)

/*
struct throw_error {
  throw_error() {}

  explicit throw_error(const std::string &s) {
    ss << s;
  }

  ~throw_error() {
    throw std::runtime_error(ss.str());
  }

  template<class T>
  throw_error &operator<<(const T &val) {
    ss << val;
    return *this;
  }

  std::ostringstream ss;
};
*/

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
  int busnum, devnum, parent_devnum;  // USB device bus number and device number

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
      LOG(FATAL) << "Cannot identify '" << dev_name << "': " << errno << ", "
                 << strerror(errno);
    }
    if (!S_ISCHR(st.st_mode)) {  // character device?
      LOG(FATAL) << dev_name << " is no device";
    }

    // Search directory and up to three parent directories to find busnum/devnum
    std::ostringstream ss;
    ss << "/sys/dev/char/" << major(st.st_rdev) << ":" << minor(st.st_rdev)
       << "/device/";
    auto path = ss.str();

    bool good = false;
    for (int i = 0; i <= 3; ++i) {
      if (std::ifstream(path + "busnum") >> busnum) {
        if (std::ifstream(path + "devnum") >> devnum) {
          if (std::ifstream(path + "../devnum") >> parent_devnum) {
            good = true;
            break;
          }
        }
      }
      path += "../";
    }
    if (!good)
      LOG(FATAL) << "Failed to read busnum/devnum";

    if (!(std::ifstream("/sys/class/video4linux/" + name + "/name") >>
          this->name))
      LOG(FATAL) << "Failed to read name";

    std::string modalias;
    if (!(std::ifstream(
              "/sys/class/video4linux/" + name + "/device/modalias") >>
          modalias))
      LOG(FATAL) << "Failed to read modalias";
    if (modalias.size() < 14 || modalias.substr(0, 5) != "usb:v" ||
        modalias[9] != 'p')
      LOG(FATAL) << "Not a usb format modalias";
    if (!(std::istringstream(modalias.substr(5, 4)) >> std::hex >> vid))
      LOG(FATAL) << "Failed to read vendor ID";
    if (!(std::istringstream(modalias.substr(10, 4)) >> std::hex >> pid))
      LOG(FATAL) << "Failed to read product ID";
    if (!(std::ifstream(
              "/sys/class/video4linux/" + name + "/device/bInterfaceNumber") >>
          std::hex >> mi))
      LOG(FATAL) << "Failed to read interface number";

    fd = open(dev_name.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd < 0) {
      LOG(FATAL) << "Cannot open '" << dev_name << "': " << errno << ", "
                 << strerror(errno);
    }

    v4l2_capability cap = {};
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
      if (errno == EINVAL)
        LOG(FATAL) << dev_name << " is no V4L2 device";
      else
        LOG_ERROR(FATAL, "VIDIOC_QUERYCAP");
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
      LOG(FATAL) << dev_name + " is no video capture device";
    if (!(cap.capabilities & V4L2_CAP_STREAMING))
      LOG(FATAL) << dev_name + " does not support streaming I/O";

    // Select video input, video standard and tune here.
    v4l2_cropcap cropcap = {};
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_CROPCAP, &cropcap) == 0) {
      v4l2_crop crop = {};
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
    }  // Errors ignored
  }

  ~device() {
    VLOG(2) << __func__;
    stop_streaming();
    if (fd != -1 && close(fd) < 0) {
      LOG_ERROR(WARNING, "close");
    }
  }

  void get_control(
      const extension_unit &xu, uint8_t control, void *data,
      size_t size) const {
    uvc_xu_control_query q = {static_cast<uint8_t>(xu.unit), control,
                              UVC_GET_CUR, static_cast<uint16_t>(size),
                              reinterpret_cast<uint8_t *>(data)};
    if (xioctl(fd, UVCIOC_CTRL_QUERY, &q) < 0) {
      LOG_ERROR(FATAL, "UVCIOC_CTRL_QUERY:UVC_GET_CUR");
    }
  }

  void set_control(
      const extension_unit &xu, uint8_t control, void *data,
      size_t size) const {
    uvc_xu_control_query q = {static_cast<uint8_t>(xu.unit), control,
                              UVC_SET_CUR, static_cast<uint16_t>(size),
                              reinterpret_cast<uint8_t *>(data)};
    if (xioctl(fd, UVCIOC_CTRL_QUERY, &q) < 0) {
      LOG_ERROR(FATAL, "UVCIOC_CTRL_QUERY:UVC_SET_CUR");
    }
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
      LOG(WARNING) << "start capture failed: is capturing already";
      return;
    }

    v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = format;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    // fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0)
      LOG_ERROR(FATAL, "VIDIOC_S_FMT");

    v4l2_streamparm parm = {};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_G_PARM, &parm) < 0)
      LOG_ERROR(FATAL, "VIDIOC_G_PARM");
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = fps;
    if (xioctl(fd, VIDIOC_S_PARM, &parm) < 0)
      LOG_ERROR(FATAL, "VIDIOC_S_PARM");

    // Init memory mapped IO
    v4l2_requestbuffers req = {};
    req.count = 4;
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
      v4l2_buffer buf = {};
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
      v4l2_buffer buf = {};
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
    struct v4l2_requestbuffers req = {};
    req.count = 0;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
      if (errno == EINVAL)
        LOG(ERROR) << dev_name << " does not support memory mapping";
      else
        LOG_ERROR(WARNING, "VIDIOC_REQBUFS");
    }

    callback = nullptr;
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
      v4l2_buffer buf = {};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
        if (errno == EAGAIN)
          return;
        LOG_ERROR(FATAL, "VIDIOC_DQBUF");
      }

      if (callback) {
        callback(buffers[buf.index].start);
      }

      if (xioctl(fd, VIDIOC_QBUF, &buf) < 0)
        LOG_ERROR(FATAL, "VIDIOC_QBUF");
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
      devices.push_back(std::make_shared<device>(context, name));
    } catch (const std::exception &e) {
      LOG(INFO) << "Not a USB video device: " << e.what();
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

void get_control(
    const device &device, const extension_unit &xu, uint8_t ctrl, void *data,
    int len) {
  device.get_control(xu, ctrl, data, len);
}

void set_control(
    const device &device, const extension_unit &xu, uint8_t ctrl, void *data,
    int len) {
  device.set_control(xu, ctrl, data, len);
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
