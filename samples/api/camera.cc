#include <opencv2/highgui/highgui.hpp>

#include "mynteye/glog_init.h"

#include "mynteye/api.h"
#include "mynteye/times.h"
#include "mynteye/utils.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  glog_init _(argc, argv);

  auto &&api = API::Create();
  api->LogOptionInfos();

  std::size_t left_count = 0;
  api->SetStreamCallback(
      Stream::LEFT, [&left_count](const api::StreamData &data) {
        CHECK_NOTNULL(data.img);
        ++left_count;
        VLOG(2) << Stream::LEFT << ", count: " << left_count;
        VLOG(2) << "  frame_id: " << data.img->frame_id
                << ", timestamp: " << data.img->timestamp
                << ", exposure_time: " << data.img->exposure_time;
      });
  std::size_t right_count = 0;
  api->SetStreamCallback(
      Stream::RIGHT, [&right_count](const api::StreamData &data) {
        CHECK_NOTNULL(data.img);
        ++right_count;
        VLOG(2) << Stream::RIGHT << ", count: " << right_count;
        VLOG(2) << "  frame_id: " << data.img->frame_id
                << ", timestamp: " << data.img->timestamp
                << ", exposure_time: " << data.img->exposure_time;
      });

  std::size_t imu_count = 0;
  api->SetMotionCallback([&imu_count](const api::MotionData &data) {
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

  api->EnableStreamData(Stream::LEFT_RECTIFIED);
  api->EnableStreamData(Stream::RIGHT_RECTIFIED);
  // Enable this will cache the motion datas until you get them.
  api->EnableMotionDatas();
  api->Start(Source::ALL);

  cv::namedWindow("frame");

  std::size_t motion_count = 0;
  auto &&time_beg = times::now();
  while (true) {
    api->WaitForStreams();

    // auto &&left_data = api->GetStreamData(Stream::LEFT);
    // auto &&right_data = api->GetStreamData(Stream::RIGHT);
    auto &&left_data = api->GetStreamData(Stream::LEFT_RECTIFIED);
    auto &&right_data = api->GetStreamData(Stream::RIGHT_RECTIFIED);
    if (left_data.frame.empty() || right_data.frame.empty()) {
      continue;
    }

    auto &&motion_datas = api->GetMotionDatas();
    motion_count += motion_datas.size();
    for (auto &&data : motion_datas) {
      LOG(INFO) << "Imu frame_id: " << data.imu->frame_id
                << ", timestamp: " << data.imu->timestamp
                << ", accel_x: " << data.imu->accel[0]
                << ", accel_y: " << data.imu->accel[1]
                << ", accel_z: " << data.imu->accel[2]
                << ", gyro_x: " << data.imu->gyro[0]
                << ", gyro_y: " << data.imu->gyro[1]
                << ", gyro_z: " << data.imu->gyro[2]
                << ", temperature: " << data.imu->temperature;
    }

    cv::Mat img;
    cv::hconcat(left_data.frame, right_data.frame, img);
    cv::imshow("frame", img);

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }
  auto &&time_end = times::now();

  api->Stop(Source::ALL);

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
