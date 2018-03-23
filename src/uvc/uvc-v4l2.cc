#include "uvc.h"  // NOLINT

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <glog/logging.h>

#include <fstream>
#include <string>

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

struct device {
  const std::shared_ptr<context> parent;

  std::string dev_name;  // Device name (typically of the form /dev/video*)
  int busnum, devnum, parent_devnum;  // USB device bus number and device number

  int vid, pid, mi;  // Vendor ID, product ID, and multiple interface index
  int fd = -1;       // File descriptor for this device

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
  }

  ~device() {
    VLOG(2) << __func__;
    if (fd != -1) {
      LOG_IF(WARNING, close(fd) < 0) << "close error " << errno << ", "
                                     << strerror(errno);
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

int get_vendor_id(const device &device) {
  return device.vid;
}

int get_product_id(const device &device) {
  return device.pid;
}

}  // namespace uvc

MYNTEYE_END_NAMESPACE
