#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye/glog_init.h"

#include "device/context.h"
#include "device/device.h"

#include "internal/times.h"

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
  {  // manual-exposure
    device->SetOptionValue(Option::EXPOSURE_MODE, 1);
    device->SetOptionValue(Option::GAIN, 20);  // [0.48]
    device->SetOptionValue(Option::BRIGHTNESS, 20);  // [0,240]
    device->SetOptionValue(Option::CONTRAST, 20);  // [0,255]
  }
  device->SetOptionValue(Option::IR_CONTROL, 80);
  */
  device->LogOptionInfos();

  // device->RunOptionAction(Option::ZERO_DRIFT_CALIBRATION);

  std::size_t left_count = 0;
  device->SetStreamCallback(
      Stream::LEFT, [&left_count](const device::StreamData &data) {
        CHECK_NOTNULL(data.img);
        ++left_count;
        VLOG(2) << Stream::LEFT << ", count: " << left_count;
        VLOG(2) << "  frame_id: " << data.img->frame_id
                << ", timestamp: " << data.img->timestamp
                << ", exposure_time: " << data.img->exposure_time;
      });
  std::size_t right_count = 0;
  device->SetStreamCallback(
      Stream::RIGHT, [&right_count](const device::StreamData &data) {
        CHECK_NOTNULL(data.img);
        ++right_count;
        VLOG(2) << Stream::RIGHT << ", count: " << right_count;
        VLOG(2) << "  frame_id: " << data.img->frame_id
                << ", timestamp: " << data.img->timestamp
                << ", exposure_time: " << data.img->exposure_time;
      });

  std::size_t imu_count = 0;
  device->SetMotionCallback([&imu_count](const device::MotionData &data) {
    CHECK_NOTNULL(data.imu);
    ++imu_count;
    VLOG(2) << "Imu count: " << imu_count;
    VLOG(2) << "  frame_id: " << data.imu->frame_id
            << ", timestamp: " << data.imu->timestamp
            << ", accel_x: " << data.imu->accel[0]
            << ", accel_y: " << data.imu->accel[1]
            << ", accel_z: " << data.imu->accel[2]
            << ", gyro_x: " << data.imu->gyro[0]
            << ", gyro_y: " << data.imu->gyro[1]
            << ", gyro_z: " << data.imu->gyro[2]
            << ", temperature: " << data.imu->temperature;
  });

  // Enable this will cache the motion datas until you get them.
  device->EnableMotionDatas();
  device->Start(Source::ALL);

  cv::namedWindow("frame");

  std::size_t motion_count = 0;
  auto &&time_beg = times::now();
  while (true) {
    device->WaitForStreams();

    device::StreamData left_data = device->GetLatestStreamData(Stream::LEFT);
    device::StreamData right_data = device->GetLatestStreamData(Stream::RIGHT);

    auto &&motion_datas = device->GetMotionDatas();
    motion_count += motion_datas.size();
    for (auto &&data : motion_datas) {
      LOG(INFO) << "  frame_id: " << data.imu->frame_id
                << ", timestamp: " << data.imu->timestamp
                << ", accel_x: " << data.imu->accel[0]
                << ", accel_y: " << data.imu->accel[1]
                << ", accel_z: " << data.imu->accel[2]
                << ", gyro_x: " << data.imu->gyro[0]
                << ", gyro_y: " << data.imu->gyro[1]
                << ", gyro_z: " << data.imu->gyro[2]
                << ", temperature: " << data.imu->temperature;
    }

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
  auto &&time_end = times::now();

  device->Stop(Source::ALL);

  float elapsed_ms =
      times::count<times::microseconds>(time_end - time_beg) * 0.001f;
  LOG(INFO) << "Time beg: " << times::to_local_string(time_beg)
            << ", end: " << times::to_local_string(time_end)
            << ", cost: " << elapsed_ms << "ms";
  LOG(INFO) << "Left count: " << left_count
            << ", fps: " << (1000.f * left_count / elapsed_ms);
  LOG(INFO) << "Right count: " << right_count
            << ", fps: " << (1000.f * right_count / elapsed_ms);
  LOG(INFO) << "Imu count: " << imu_count
            << ", hz: " << (1000.f * imu_count / elapsed_ms);
  // LOG(INFO) << "Motion count: " << motion_count
  //           << ", hz: " << (1000.f * motion_count / elapsed_ms);
  return 0;
}
