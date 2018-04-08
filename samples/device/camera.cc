#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "glog_init.h"  // NOLINT

#include "device/context.h"
#include "device/device.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  glog_init _(argc, argv);

  Context context;
  auto &&devices = context.devices();

  size_t n = devices.size();
  LOG_IF(FATAL, n <= 0) << "No MYNT EYE devices :(";

  LOG(INFO) << "MYNT EYE devices: ";
  for (size_t i = 0; i < n; i++) {
    auto &&device = devices[i];
    auto &&name = device->GetInfo(Info::DEVICE_NAME);
    LOG(INFO) << "  index: " << i << ", name: " << name;
  }

  std::shared_ptr<Device> device = nullptr;
  if (n <= 1) {
    device = devices[0];
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
      device = devices[i];
      break;
    }
  }

  std::size_t left_count = 0;
  device->SetStreamCallback(
      Stream::LEFT, [&left_count](const device::StreamData &data) {
        ++left_count;
        VLOG(2) << Stream::LEFT << ", count: " << left_count;
        UNUSED(data)
      });
  std::size_t right_count = 0;
  device->SetStreamCallback(
      Stream::RIGHT, [&right_count](const device::StreamData &data) {
        ++right_count;
        VLOG(2) << Stream::RIGHT << ", count: " << right_count;
        UNUSED(data)
      });

  device->Start(Source::ALL);

  cv::namedWindow("frame");

  while (true) {
    device->WaitForStreams();

    device::StreamData left_data = device->GetLatestStreamData(Stream::LEFT);
    device::StreamData right_data = device->GetLatestStreamData(Stream::RIGHT);

    cv::Mat left_img(
        left_data.frame->height(), left_data.frame->width(), CV_8UC1,
        left_data.frame->data());
    cv::Mat right_img(
        right_data.frame->height(), right_data.frame->width(), CV_8UC1,
        right_data.frame->data());

    cv::Mat img;
    cv::hconcat(left_img, right_img, img);
    cv::imshow("frame", img);

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  device->Stop(Source::ALL);
  return 0;
}
