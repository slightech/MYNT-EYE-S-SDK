#include <glog/logging.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <chrono>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <mutex>

#include "mynteye/mynteye.h"
#include "uvc/uvc.h"

struct glog_init {
  glog_init(int /*argc*/, char *argv[]) {
    // FLAGS_logtostderr = true;
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    FLAGS_log_dir = ".";
    FLAGS_max_log_size = 1024;
    FLAGS_stop_logging_if_full_disk = true;

    // FLAGS_v = 2;

    google::InitGoogleLogging(argv[0]);

    VLOG(2) << __func__;
  }

  ~glog_init() {
    VLOG(2) << __func__;
    google::ShutdownGoogleLogging();
  }
};

struct frame {
  const void *data = nullptr;
  ~frame() {
    data = nullptr;
  }
};

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  glog_init _(argc, argv);

  std::vector<std::shared_ptr<uvc::device>> mynteye_devices;

  auto context = uvc::create_context();
  auto devices = uvc::query_devices(context);
  LOG_IF(FATAL, devices.size() <= 0) << "No devices :(";
  for (auto &&device : devices) {
    auto vid = uvc::get_vendor_id(*device);
    // auto pid = uvc::get_product_id(*device);
    // LOG(INFO) << "vid: " << vid << ", pid: " << pid;
    if (vid == MYNTEYE_VID) {
      mynteye_devices.push_back(device);
    }
  }

  // std::string dashes(80, '-');

  size_t n = mynteye_devices.size();
  LOG_IF(FATAL, n <= 0) << "No MYNT EYE devices :(";

  for (size_t i = 0; i < n; i++) {
    auto device = mynteye_devices[i];
    auto name = uvc::get_name(*device);
    auto vid = uvc::get_vendor_id(*device);
    auto pid = uvc::get_product_id(*device);
    LOG(INFO) << i << " | name: " << name << ", vid: " << vid
              << ", pid: " << pid;
  }

  std::shared_ptr<uvc::device> device = nullptr;
  if (n <= 1) {
    device = mynteye_devices[0];
    LOG(INFO) << "Only one MYNT EYE device, select index: 0";
  } else {
    while (true) {
      size_t i;
      LOG(INFO) << "There are " << n << " MYNT EYE devices, select index: ";
      std::cin >> i;
      if (i >= n) {
        LOG(WARNING) << "Index out of range :(";
        continue;
      }
      device = mynteye_devices[i];
      break;
    }
  }

  std::mutex mtx;
  std::condition_variable cv;

  std::vector<frame> frames;
  const auto frame_ready = [&frames]() { return !frames.empty(); };
  const auto frame_empty = [&frames]() { return frames.empty(); };

  uvc::set_device_mode(
      *device, 752, 480, 0, 25,
      [&mtx, &cv, &frames, &frame_ready](const void *data) {
        // reinterpret_cast<const std::uint8_t *>(data);
        std::unique_lock<std::mutex> lock(mtx);
        frame frame;
        frame.data = data;  // not copy
        frames.push_back(frame);
        if (frame_ready())
          cv.notify_one();
      });

  cv::namedWindow("frame");
  uvc::start_streaming(*device, 0);

  double t, fps = 0;
  while (true) {
    t = static_cast<double>(cv::getTickCount());

    std::unique_lock<std::mutex> lock(mtx);

    if (frame_empty()) {
      if (!cv.wait_for(lock, std::chrono::seconds(2), frame_ready))
        throw std::runtime_error("Timeout waiting for frame.");
    }

    auto frame = frames.back();  // only last one is valid

    cv::Mat img(480, 752, CV_8UC2, const_cast<void *>(frame.data));
    cv::cvtColor(img, img, cv::COLOR_YUV2BGR_YUY2);
    cv::imshow("frame", img);

    frames.clear();

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }

    t = static_cast<double>(cv::getTickCount() - t);
    fps = cv::getTickFrequency() / t;
  }
  UNUSED(fps)

  uvc::stop_streaming(*device);
  // cv::destroyAllWindows();
  return 0;
}
