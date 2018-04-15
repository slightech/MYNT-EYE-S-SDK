#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye/glog_init.h"

#include "device/context.h"
#include "device/device.h"

#include "internal/times.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  glog_init _(argc, argv);

  LOG(INFO) << "Detecting MYNT EYE devices";
  Context context;
  auto &&devices = context.devices();

  size_t n = devices.size();
  LOG_IF(FATAL, n <= 0) << "No MYNT EYE devices :(";

  LOG(INFO) << "MYNT EYE devices:";
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
  {  // manual-exposure
    device->SetOptionValue(Option::EXPOSURE_MODE, 1);
    device->SetOptionValue(Option::GAIN, 20);  // [0.48]
    device->SetOptionValue(Option::BRIGHTNESS, 20);  // [0,240]
    device->SetOptionValue(Option::CONTRAST, 20);  // [0,255]
  }
  device->SetOptionValue(Option::IR_CONTROL, 80);
  device->SetOptionValue(Option::FRAME_RATE, 25);
  device->SetOptionValue(Option::IMU_FREQUENCY, 500);
  */
  device->LogOptionInfos();

  // Enable this will cache the motion datas until you get them.
  device->EnableMotionDatas();
  device->Start(Source::ALL);

  cv::namedWindow("frame");

  std::size_t img_count = 0;
  std::size_t imu_count = 0;
  auto &&time_beg = times::now();
  while (true) {
    device->WaitForStreams();

    auto &&left_datas = device->GetStreamDatas(Stream::LEFT);
    auto &&right_datas = device->GetStreamDatas(Stream::RIGHT);
    img_count += left_datas.size();

    auto &&motion_datas = device->GetMotionDatas();
    imu_count += motion_datas.size();

    auto &&left_frame = left_datas.back().frame;
    auto &&right_frame = right_datas.back().frame;
    cv::Mat left_img(
        left_frame->height(), left_frame->width(), CV_8UC1, left_frame->data());
    cv::Mat right_img(
        right_frame->height(), right_frame->width(), CV_8UC1,
        right_frame->data());

    cv::Mat img;
    cv::hconcat(left_img, right_img, img);
    cv::imshow("frame", img);

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }
  auto &&time_end = times::now();

  device->Stop(Source::ALL);

  float elapsed_ms =
      times::count<times::microseconds>(time_end - time_beg) * 0.001f;
  LOG(INFO) << "Time beg: " << times::to_local_string(time_beg)
            << ", end: " << times::to_local_string(time_end)
            << ", cost: " << elapsed_ms << "ms";
  LOG(INFO) << "Img count: " << img_count
            << ", fps: " << (1000.f * img_count / elapsed_ms);
  LOG(INFO) << "Imu count: " << imu_count
            << ", hz: " << (1000.f * imu_count / elapsed_ms);
  return 0;
}
