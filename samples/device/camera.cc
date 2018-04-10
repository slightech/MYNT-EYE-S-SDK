#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye/glog_init.h"

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

  /*
  {  // auto-exposure
    device->SetOptionValue(Option::EXPOSURE_MODE, 0);
    device->SetOptionValue(Option::MAX_GAIN, 40);  // [0.48]
    device->SetOptionValue(Option::MAX_EXPOSURE_TIME, 120);  // [0,240]
    device->SetOptionValue(Option::DESIRED_BRIGHTNESS, 200);  // [0,255]
  }
  */
  /*
  {  // manual-exposure
    device->SetOptionValue(Option::EXPOSURE_MODE, 1);
    device->SetOptionValue(Option::GAIN, 20);  // [0.48]
    device->SetOptionValue(Option::BRIGHTNESS, 20);  // [0,240]
    device->SetOptionValue(Option::CONTRAST, 20);  // [0,255]
  }
  device->SetOptionValue(Option::IR_CONTROL, 80);
  */
  device->LogOptionInfos();

  std::size_t left_count = 0;
  device->SetStreamCallback(
      Stream::LEFT, [&left_count](const device::StreamData &data) {
        ++left_count;
        VLOG(2) << Stream::LEFT << ", count: " << left_count;
        if (data.img) {
          VLOG(2) << "  frame_id: " << data.img->frame_id
                  << ", timestamp: " << data.img->timestamp
                  << ", exposure_time: " << data.img->exposure_time;
        }
      });
  std::size_t right_count = 0;
  device->SetStreamCallback(
      Stream::RIGHT, [&right_count](const device::StreamData &data) {
        ++right_count;
        VLOG(2) << Stream::RIGHT << ", count: " << right_count;
        if (data.img) {
          VLOG(2) << "  frame_id: " << data.img->frame_id
                  << ", timestamp: " << data.img->timestamp
                  << ", exposure_time: " << data.img->exposure_time;
        }
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
