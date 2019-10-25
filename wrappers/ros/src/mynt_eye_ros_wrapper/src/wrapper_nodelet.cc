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
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/distortion_models.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <opencv2/calib3d/calib3d.hpp>

#include <mynt_eye_ros_wrapper/GetInfo.h>

#define _USE_MATH_DEFINES
#include <cmath>
#include <map>
#include <string>

#include "mynteye/logger.h"
#include "mynteye/api/api.h"
#include "mynteye/device/context.h"
#include "mynteye/device/device.h"
#define CONFIGURU_IMPLEMENTATION 1
#include "configuru.hpp"
using namespace configuru;  // NOLINT

#define PIE 3.1416
#define MATCH_CHECK_THRESHOLD 3

#define FULL_PRECISION \
  std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)

static const std::size_t MAXSIZE = 4;

MYNTEYE_BEGIN_NAMESPACE

namespace enc = sensor_msgs::image_encodings;
inline double compute_time(const double end, const double start) {
  return end - start;
}

class ROSWrapperNodelet : public nodelet::Nodelet {
 public:
  ROSWrapperNodelet() :
  mesh_position_x(0.),
  mesh_position_y(-0.176),
  mesh_position_z(0.),
  mesh_rotation_x(PIE/2),
  mesh_rotation_y(0.0),
  mesh_rotation_z(PIE/2),
  skip_tag(-1),
  skip_tmp_left_tag(0),
  skip_tmp_right_tag(0) {
    unit_hard_time *= 10;
  }

  ~ROSWrapperNodelet() {
    // std::cout << __func__ << std::endl;
    if (api_) {
      api_->Stop(Source::ALL);
    }
    if (time_beg_ != -1) {
      double time_end = ros::Time::now().toSec();

      LOG(INFO) << "Time elapsed: " << compute_time(time_end, time_beg_)
                << " s";
      if (left_time_beg_ != -1) {
        LOG(INFO) << "Left count: " << left_count_ << ", fps: "
                  << (left_count_ / compute_time(time_end, left_time_beg_));
      }
      if (right_time_beg_ != -1) {
        LOG(INFO) << "Right count: " << right_count_ << ", fps: "
                  << (right_count_ / compute_time(time_end, right_time_beg_));
      }
      if (imu_time_beg_ != -1) {
          if (model_ == Model::STANDARD) {
            LOG(INFO) << "Imu count: " << imu_count_ << ", hz: "
                      << (imu_count_ /
                      compute_time(time_end, imu_time_beg_));
          } else {
          if (publish_imu_by_sync_) {
            LOG(INFO) << "imu_sync_count: " << imu_sync_count_ << ", hz: "
                      << (imu_sync_count_ /
                      compute_time(time_end, imu_time_beg_));
          } else {
            LOG(INFO) << "Imu count: " << imu_count_ << ", hz: "
                      << (imu_count_ /
                      compute_time(time_end, imu_time_beg_));
          }
        }
      }

      // ROS messages could not be reliably printed here, using glog instead :(
      // ros::Duration(1).sleep();  // 1s
      // https://answers.ros.org/question/35163/how-to-perform-an-action-at-nodelet-unload-shutdown/
    }
  }

  ros::Time hardTimeToSoftTime(std::uint64_t _hard_time) {
    static bool isInited = false;
    static double soft_time_begin(0);
    static std::uint64_t hard_time_begin(0);

    if (false == isInited) {
      soft_time_begin = ros::Time::now().toSec();
      hard_time_begin = _hard_time;
      isInited = true;
    }

    std::uint64_t time_ns_detal = (_hard_time - hard_time_begin);
    std::uint64_t time_ns_detal_s = time_ns_detal / 1000000;
    std::uint64_t time_ns_detal_ns = time_ns_detal % 1000000;
    double time_sec_double =
      ros::Time(time_ns_detal_s, time_ns_detal_ns * 1000).toSec();

    return ros::Time(soft_time_begin + time_sec_double);
  }

  // ros::Time hardTimeToSoftTime(std::uint64_t _hard_time) {
  //   static bool isInited = false;
  //   static double soft_time_begin(0);
  //   static std::uint64_t hard_time_begin(0);

  //   if (false == isInited) {
  //     soft_time_begin = ros::Time::now().toSec();
  //     hard_time_begin = _hard_time;
  //     isInited = true;
  //   }

  //   return ros::Time(
  //       static_cast<double>(soft_time_begin +
  //       static_cast<double>(_hard_time - hard_time_begin) * 0.000001f));
  // }

  inline bool is_overflow(std::uint64_t now,
      std::uint64_t pre) {

    return (now < pre) && ((pre - now) > (unit_hard_time / 2));
  }

  inline bool is_repeated(std::uint64_t now,
      std::uint64_t pre) {
    return now == pre;
  }

  inline bool is_abnormal(std::uint32_t now,
      std::uint32_t pre) {

    return (now < pre) && ((pre - now) < (unit_hard_time / 4));
  }

  ros::Time checkUpTimeStamp(std::uint64_t _hard_time,
      const Stream &stream) {
    static std::map<Stream, std::uint64_t> hard_time_now;
    static std::map<Stream, std::uint64_t> acc;

    if (is_overflow(_hard_time, hard_time_now[stream])) {
      acc[stream]++;
    }

    hard_time_now[stream] = _hard_time;

    return hardTimeToSoftTime(
        acc[stream] * unit_hard_time + _hard_time);
  }

  ros::Time checkUpImuTimeStamp(std::uint64_t _hard_time) {
    static std::uint64_t hard_time_now(0), acc(0);

    if (is_overflow(_hard_time, hard_time_now)) {
      acc++;
    }

    hard_time_now = _hard_time;

    return hardTimeToSoftTime(
        acc * unit_hard_time + _hard_time);
  }

  void onInit() override {
    nh_ = getMTNodeHandle();
    private_nh_ = getMTPrivateNodeHandle();

    initDevice();
    NODELET_FATAL_COND(api_ == nullptr, "No MYNT EYE device selected :(");

    pthread_mutex_init(&mutex_data_, nullptr);

    // node params

    std::map<Stream, std::string> stream_names{
        {Stream::LEFT, "left"},
        {Stream::RIGHT, "right"},
        {Stream::LEFT_RECTIFIED, "left_rect"},
        {Stream::RIGHT_RECTIFIED, "right_rect"},
        {Stream::DISPARITY, "disparity"},
        {Stream::DISPARITY_NORMALIZED, "disparity_norm"},
        {Stream::DEPTH, "depth"},
        {Stream::POINTS, "points"}};

    std::map<Stream, std::string> stream_topics{};
    for (auto &&it = stream_names.begin(); it != stream_names.end(); ++it) {
      stream_topics[it->first] = it->second;
      private_nh_.getParamCached(it->second + "_topic", stream_topics[it->first]);

      // if published init
      is_published_[it->first] = false;
    }
    is_motion_published_ = false;
    is_started_ = false;

    std::map<Stream, std::string> mono_names{{Stream::LEFT, "left_mono"},
                                             {Stream::RIGHT, "right_mono"},
                                             {Stream::LEFT_RECTIFIED, "left_rect_mono"},
                                             {Stream::RIGHT_RECTIFIED, "right_rect_mono"}};

    std::map<Stream, std::string> mono_topics{};
    for (auto &&it = mono_names.begin(); it != mono_names.end(); ++it) {
      mono_topics[it->first] = it->second;
      private_nh_.getParamCached(it->second + "_topic", mono_topics[it->first]);
    }

    std::string imu_topic = "imu";
    std::string temperature_topic = "temperature";
    private_nh_.getParamCached("imu_topic", imu_topic);
    private_nh_.getParamCached("temperature_topic", temperature_topic);

    base_frame_id_ = "camera_link";
    private_nh_.getParamCached("base_frame_id", base_frame_id_);

    for (auto &&it = stream_names.begin(); it != stream_names.end(); ++it) {
      frame_ids_[it->first] = "camera_" + it->second + "_frame";
      private_nh_.getParamCached(it->second + "_frame_id", frame_ids_[it->first]);
    }

    imu_frame_id_ = "camera_imu_frame";
    temperature_frame_id_ = "camera_temperature_frame";
    private_nh_.getParamCached("imu_frame_id", imu_frame_id_);
    private_nh_.getParamCached("temperature_frame_id", temperature_frame_id_);

    gravity_ = 9.8;
    private_nh_.getParamCached("gravity", gravity_);

    int tmp_disparity_type_ = 0;
    disparity_type_ = DisparityComputingMethod::BM;
    private_nh_.getParamCached("disparity_computing_method", tmp_disparity_type_);
    disparity_type_ = (DisparityComputingMethod)tmp_disparity_type_;
    api_->SetDisparityComputingMethodType(disparity_type_);

    // device options of standard210a
    if (model_ == Model::STANDARD210A) {
      option_names_ = {
          {Option::BRIGHTNESS, "standard210a/brightness"},
          {Option::EXPOSURE_MODE, "standard210a/exposure_mode"},
          {Option::MAX_GAIN, "standard210a/max_gain"},
          {Option::MAX_EXPOSURE_TIME, "standard210a/max_exposure_time"},
          {Option::DESIRED_BRIGHTNESS, "standard210a/desired_brightness"},
          {Option::MIN_EXPOSURE_TIME, "standard210a/min_exposure_time"},
          {Option::ACCELEROMETER_RANGE, "standard210a/accel_range"},
          {Option::GYROSCOPE_RANGE, "standard210a/gyro_range"},
          {Option::ACCELEROMETER_LOW_PASS_FILTER,
                  "standard210a/accel_low_filter"},
          {Option::GYROSCOPE_LOW_PASS_FILTER, "standard210a/gyro_low_filter"},
          {Option::IIC_ADDRESS_SETTING, "standard210a/iic_address_setting"}};
    }

    // device options of standard2
    if (model_ == Model::STANDARD2) {
      option_names_ = {
          {Option::BRIGHTNESS, "standard2/brightness"},
          {Option::EXPOSURE_MODE, "standard2/exposure_mode"},
          {Option::MAX_GAIN, "standard2/max_gain"},
          {Option::MAX_EXPOSURE_TIME, "standard2/max_exposure_time"},
          {Option::DESIRED_BRIGHTNESS, "standard2/desired_brightness"},
          {Option::MIN_EXPOSURE_TIME, "standard2/min_exposure_time"},
          {Option::IR_CONTROL, "STANDARD/ir_control"},
          {Option::ACCELEROMETER_RANGE, "standard2/accel_range"},
          {Option::GYROSCOPE_RANGE, "standard2/gyro_range"},
          {Option::ACCELEROMETER_LOW_PASS_FILTER, "standard2/accel_low_filter"},
          {Option::GYROSCOPE_LOW_PASS_FILTER, "standard2/gyro_low_filter"}};
    }
    // device options of standard
    if (model_ == Model::STANDARD) {
      option_names_ = {
          {Option::GAIN, "standard/gain"},
          {Option::BRIGHTNESS, "standard/brightness"},
          {Option::CONTRAST, "standard/contrast"},
          {Option::FRAME_RATE, "standard/frame_rate"},
          {Option::IMU_FREQUENCY, "standard/imu_frequency"},
          {Option::EXPOSURE_MODE, "standard/exposure_mode"},
          {Option::MAX_GAIN, "standard/max_gain"},
          {Option::MAX_EXPOSURE_TIME, "standard/max_exposure_time"},
          {Option::DESIRED_BRIGHTNESS, "standard/desired_brightness"},
          {Option::IR_CONTROL, "standard/ir_control"},
          {Option::HDR_MODE, "standard/hdr_mode"},
          {Option::ACCELEROMETER_RANGE, "standard/accel_range"},
          {Option::GYROSCOPE_RANGE, "standard/gyro_range"}};
    }

    // device options of standard200b
    if (model_ == Model::STANDARD200B) {
    option_names_ = {
        {Option::BRIGHTNESS, "standard200b/brightness"},
        {Option::EXPOSURE_MODE, "standard200b/exposure_mode"},
        {Option::MAX_GAIN, "standard200b/max_gain"},
        {Option::MAX_EXPOSURE_TIME, "standard200b/max_exposure_time"},
        {Option::DESIRED_BRIGHTNESS, "standard200b/desired_brightness"},
        {Option::MIN_EXPOSURE_TIME, "standard200b/min_exposure_time"},
        {Option::ACCELEROMETER_RANGE, "standard200b/accel_range"},
        {Option::GYROSCOPE_RANGE, "standard200b/gyro_range"},
        {Option::ACCELEROMETER_LOW_PASS_FILTER, "standard200b/accel_low_filter"},
        {Option::GYROSCOPE_LOW_PASS_FILTER, "standard200b/gyro_low_filter"}};
    }

    for (auto &&it = option_names_.begin(); it != option_names_.end(); ++it) {
      if (!api_->Supports(it->first))
        continue;
      int value = -1;
      private_nh_.getParamCached(it->second, value);
      if (value != -1) {
        NODELET_INFO_STREAM("Set " << it->second << " to " << value);
        api_->SetOptionValue(it->first, value);
      }
      NODELET_INFO_STREAM(it->first << ": " << api_->GetOptionValue(it->first));
    }

    // publishers

    image_transport::ImageTransport it_mynteye(nh_);

    for (auto &&it = stream_names.begin(); it != stream_names.end(); ++it) {
      auto &&topic = stream_topics[it->first];
      if (it->first == Stream::POINTS) {  // pointcloud
        points_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic, 1);
      } else {  // camera
        camera_publishers_[it->first] = it_mynteye.advertiseCamera(topic, 1);
      }
      NODELET_INFO_STREAM("Advertized on topic " << topic);
    }

    // Only STANDARD2/STANDARD210A/STANDARD200B need publish mono_topics
    if (model_ == Model::STANDARD2 ||
        model_ == Model::STANDARD210A || model_ == Model::STANDARD200B) {
      for (auto &&it = mono_topics.begin(); it != mono_topics.end(); ++it) {
        auto &&topic = mono_topics[it->first];
        if (it->first == Stream::LEFT ||
            it->first == Stream::RIGHT ||
            it->first == Stream::RIGHT_RECTIFIED ||
            it->first == Stream::LEFT_RECTIFIED) {
          mono_publishers_[it->first] = it_mynteye.advertise(topic, 1);
        }
        NODELET_INFO_STREAM("Advertized on topic " << topic);
      }
    }

    int depth_type = 0;
    private_nh_.getParamCached("depth_type", depth_type);
    if (model_ == Model::STANDARD2 ||
        model_ == Model::STANDARD210A || model_ == Model::STANDARD200B) {
      camera_encodings_ = {{Stream::LEFT, enc::BGR8},
                          {Stream::RIGHT, enc::BGR8},
                          {Stream::LEFT_RECTIFIED, enc::BGR8},
                          {Stream::RIGHT_RECTIFIED, enc::BGR8},
                          {Stream::DISPARITY, enc::MONO8},  // float
                          {Stream::DISPARITY_NORMALIZED, enc::MONO8},
                          {Stream::DEPTH, enc::MONO16}};
    }
    if (model_ == Model::STANDARD) {
      if (depth_type == 0) {
        camera_encodings_ = {{Stream::LEFT, enc::MONO8},
          {Stream::RIGHT, enc::MONO8},
          {Stream::LEFT_RECTIFIED, enc::MONO8},
          {Stream::RIGHT_RECTIFIED, enc::MONO8},
          {Stream::DISPARITY, enc::MONO8},  // float
          {Stream::DISPARITY_NORMALIZED, enc::MONO8},
          {Stream::DEPTH, enc::MONO16}};
      } else if (depth_type == 1) {
        camera_encodings_ = {{Stream::LEFT, enc::MONO8},
          {Stream::RIGHT, enc::MONO8},
          {Stream::LEFT_RECTIFIED, enc::MONO8},
          {Stream::RIGHT_RECTIFIED, enc::MONO8},
          {Stream::DISPARITY, enc::MONO8},  // float
          {Stream::DISPARITY_NORMALIZED, enc::MONO8},
          {Stream::DEPTH, enc::TYPE_16UC1}};
      }
    }
    pub_imu_ = nh_.advertise<sensor_msgs::Imu>(imu_topic, 100);
    NODELET_INFO_STREAM("Advertized on topic " << imu_topic);

    pub_temperature_ = nh_.advertise<
                        sensor_msgs::Temperature>(temperature_topic, 100);
    NODELET_INFO_STREAM("Advertized on topic " << temperature_topic);

    pub_mesh_ = nh_.advertise<visualization_msgs::Marker>("camera_mesh", 0 );
    // where to get the mesh from
    std::string mesh_file;
    if (private_nh_.getParamCached("s1030_mesh_file", mesh_file)) {
      mesh_msg_.mesh_resource = "package://mynt_eye_ros_wrapper/mesh/"+mesh_file;
    } else {
      LOG(INFO) << "no mesh found for visualisation, set ros param mesh_file, if desired";
      mesh_msg_.mesh_resource = "";
    }

    // stream toggles

    for (auto &&it = stream_names.begin(); it != stream_names.end(); ++it) {
      if (it->first == Stream::LEFT || it->first == Stream::RIGHT) {
        continue;  // native streams
      } else {
        if (!api_->Supports(it->first))
          continue;
        bool enabled = false;
        private_nh_.getParamCached("enable_" + it->second, enabled);
        if (enabled) {
          api_->EnableStreamData(it->first);
          NODELET_INFO_STREAM("Enable stream data of " << it->first);
        }
      }
    }

    int ros_output_framerate = -1;
    private_nh_.getParamCached("ros_output_framerate_cut", ros_output_framerate);
    if (ros_output_framerate > 0 && ros_output_framerate < 7) {
      skip_tag = ros_output_framerate;
    }

    // services

    const std::string DEVICE_INFO_SERVICE = "get_info";
    get_info_service_ = nh_.advertiseService(
        DEVICE_INFO_SERVICE, &ROSWrapperNodelet::getInfo, this);
    NODELET_INFO_STREAM("Advertized service " << DEVICE_INFO_SERVICE);

    publishStaticTransforms();
    ros::Rate loop_rate(frame_rate_);
    while (private_nh_.ok()) {
      publishTopics();
      loop_rate.sleep();
    }
  }

  bool getInfo(
      mynt_eye_ros_wrapper::GetInfo::Request &req,     // NOLINT
      mynt_eye_ros_wrapper::GetInfo::Response &res) {  // NOLINT
    using Request = mynt_eye_ros_wrapper::GetInfo::Request;
    switch (req.key) {
      case Request::DEVICE_NAME:
        res.value = api_->GetInfo(Info::DEVICE_NAME);
        break;
      case Request::SERIAL_NUMBER:
        res.value = api_->GetInfo(Info::SERIAL_NUMBER);
        break;
      case Request::FIRMWARE_VERSION:
        res.value = api_->GetInfo(Info::FIRMWARE_VERSION);
        break;
      case Request::HARDWARE_VERSION:
        res.value = api_->GetInfo(Info::HARDWARE_VERSION);
        break;
      case Request::SPEC_VERSION:
        res.value = api_->GetInfo(Info::SPEC_VERSION);
        break;
      case Request::LENS_TYPE:
        res.value = api_->GetInfo(Info::LENS_TYPE);
        break;
      case Request::IMU_TYPE:
        res.value = api_->GetInfo(Info::IMU_TYPE);
        break;
      case Request::NOMINAL_BASELINE:
        res.value = api_->GetInfo(Info::NOMINAL_BASELINE);
        break;
      case Request::AUXILIARY_CHIP_VERSION:
        res.value = api_->GetInfo(Info::AUXILIARY_CHIP_VERSION);
        break;
      case Request::ISP_VERSION:
        res.value = api_->GetInfo(Info::ISP_VERSION);
        break;
      case Request::IMG_INTRINSICS:
      {
        auto intri_left = api_->GetIntrinsicsBase(Stream::LEFT);
        auto calib_model = intri_left->calib_model();
        if (calib_model == CalibrationModel::PINHOLE) {
          auto intri_left =
              api_->GetIntrinsics<IntrinsicsPinhole>(Stream::LEFT);
          auto intri_right =
              api_->GetIntrinsics<IntrinsicsPinhole>(Stream::RIGHT);
          Config intrinsics{
            {"calib_model", "pinhole"},
            {"left", {
              {"width", intri_left.width},
              {"height", intri_left.height},
              {"fx", intri_left.fx},
              {"fy", intri_left.fy},
              {"cx", intri_left.cx},
              {"cy", intri_left.cy},
              {"model", intri_left.model},
              {"coeffs", Config::array(
                  {intri_left.coeffs[0],
                   intri_left.coeffs[1],
                   intri_left.coeffs[2],
                   intri_left.coeffs[3],
                   intri_left.coeffs[4]})}
            }},
            {"right", {
              {"width", intri_right.width},
              {"height", intri_right.height},
              {"fx", intri_right.fx},
              {"fy", intri_right.fy},
              {"cx", intri_right.cx},
              {"cy", intri_right.cy},
              {"model", intri_right.model},
              {"coeffs", Config::array(
                  {intri_right.coeffs[0],
                   intri_right.coeffs[1],
                   intri_right.coeffs[2],
                   intri_right.coeffs[3],
                   intri_right.coeffs[4]})}
            }}
          };
          std::string json = dump_string(intrinsics, configuru::JSON);
          res.value = json;
        } else if (calib_model == CalibrationModel::KANNALA_BRANDT) {
          auto intri_left =
              api_->GetIntrinsics<IntrinsicsEquidistant>(Stream::LEFT);
          auto intri_right =
              api_->GetIntrinsics<IntrinsicsEquidistant>(Stream::RIGHT);
          Config intrinsics{
            {"calib_model", "kannala_brandt"},
            {"left", {
              {"width", intri_left.width},
              {"height", intri_left.height},
              {"coeffs", Config::array(
                  {intri_left.coeffs[0],
                   intri_left.coeffs[1],
                   intri_left.coeffs[2],
                   intri_left.coeffs[3],
                   intri_left.coeffs[4],
                   intri_left.coeffs[5],
                   intri_left.coeffs[6],
                   intri_left.coeffs[7]})
              }
            }},
            {"right", {
              {"width", intri_right.width},
              {"height", intri_right.height},
              {"coeffs", Config::array(
                  {intri_right.coeffs[0],
                   intri_right.coeffs[1],
                   intri_right.coeffs[2],
                   intri_right.coeffs[3],
                   intri_right.coeffs[4],
                   intri_right.coeffs[5],
                   intri_right.coeffs[6],
                   intri_right.coeffs[7]})
              }
            }}
          };
          std::string json = dump_string(intrinsics, configuru::JSON);
          res.value = json;
        } else {
          NODELET_INFO_STREAM("INVALID CALIB INTRINSICS" << calib_model);
          res.value = "null";
        }
      }
      break;
      case Request::IMG_EXTRINSICS_RTOL:
      {
        auto extri = api_->GetExtrinsics(Stream::RIGHT, Stream::LEFT);
        Config extrinsics{
          {"rotation",     Config::array({extri.rotation[0][0], extri.rotation[0][1], extri.rotation[0][2],   // NOLINT
                                          extri.rotation[1][0], extri.rotation[1][1], extri.rotation[1][2],   // NOLINT
                                          extri.rotation[2][0], extri.rotation[2][1], extri.rotation[2][2]})},// NOLINT
          {"translation",  Config::array({extri.translation[0], extri.translation[1], extri.translation[2]})} // NOLINT
        };
        std::string json = dump_string(extrinsics, configuru::JSON);
        res.value = json;
      }
      break;
      case Request::IMU_INTRINSICS:
      {
        bool is_ok;
        auto intri = api_->GetMotionIntrinsics();
        Config intrinsics {
          {"accel", {
            {"scale",     Config::array({ intri.accel.scale[0][0], intri.accel.scale[0][1],  intri.accel.scale[0][2],   // NOLINT
                                          intri.accel.scale[1][0], intri.accel.scale[1][1],  intri.accel.scale[1][2],   // NOLINT
                                          intri.accel.scale[2][0], intri.accel.scale[2][1],  intri.accel.scale[2][2]})},// NOLINT
            {"drift",     Config::array({ intri.accel.drift[0],    intri.accel.drift[1],     intri.accel.drift[2]})}, // NOLINT
            {"noise",     Config::array({ intri.accel.noise[0],    intri.accel.noise[1],     intri.accel.noise[2]})}, // NOLINT
            {"bias",      Config::array({ intri.accel.bias[0],     intri.accel.bias[1],      intri.accel.bias[2]})} // NOLINT
          }},
          {"gyro", {
            {"scale",     Config::array({ intri.gyro.scale[0][0], intri.gyro.scale[0][1],  intri.gyro.scale[0][2],   // NOLINT
                                          intri.gyro.scale[1][0], intri.gyro.scale[1][1],  intri.gyro.scale[1][2],   // NOLINT
                                          intri.gyro.scale[2][0], intri.gyro.scale[2][1],  intri.gyro.scale[2][2]})},// NOLINT
            {"drift",     Config::array({ intri.gyro.drift[0],    intri.gyro.drift[1],     intri.gyro.drift[2]})}, // NOLINT
            {"noise",     Config::array({ intri.gyro.noise[0],    intri.gyro.noise[1],     intri.gyro.noise[2]})}, // NOLINT
            {"bias",      Config::array({ intri.gyro.bias[0],     intri.gyro.bias[1],      intri.gyro.bias[2]})} // NOLINT
          }}
        };
        std::string json = dump_string(intrinsics, JSON);
        res.value = json;
      }
      break;
      case Request::IMU_EXTRINSICS:
      {
        auto extri = api_->GetMotionExtrinsics(Stream::LEFT);
        Config extrinsics{
          {"rotation",     Config::array({extri.rotation[0][0], extri.rotation[0][1], extri.rotation[0][2],   // NOLINT
                                          extri.rotation[1][0], extri.rotation[1][1], extri.rotation[1][2],   // NOLINT
                                          extri.rotation[2][0], extri.rotation[2][1], extri.rotation[2][2]})},// NOLINT
          {"translation",  Config::array({extri.translation[0], extri.translation[1], extri.translation[2]})} // NOLINT
        };
        std::string json = dump_string(extrinsics, configuru::JSON);
        res.value = json;
      }
      break;
      default:
        NODELET_WARN_STREAM("Info of key " << req.key << " not exist");
        return false;
    }
    return true;
  }

  void publishData(
      const Stream &stream, const api::StreamData &data, std::uint32_t seq,
      ros::Time stamp) {
    if (stream == Stream::LEFT || stream == Stream::RIGHT) {
      return;
    } else if (stream == Stream::POINTS) {
      publishPoints(data, seq, stamp);
    } else if (stream == Stream::LEFT_RECTIFIED ||
        stream == Stream::RIGHT_RECTIFIED) {
      publishMono(stream, data, seq, stamp);
      publishCamera(stream, data, seq, stamp);
    } else {
      publishCamera(stream, data, seq, stamp);
    }
  }

  int getStreamSubscribers(const Stream &stream) {
    if (stream == Stream::POINTS) {
      return points_publisher_.getNumSubscribers();
    }
    auto pub = camera_publishers_[stream];
    if (pub)
      return pub.getNumSubscribers();
    return -1;
  }

  void publishOthers(const Stream &stream) {
    if ((getStreamSubscribers(stream) > 0 && !is_published_[stream]) ||
        mono_publishers_[stream].getNumSubscribers() > 0) {
      api_->EnableStreamData(stream);
      api_->SetStreamCallback(
          stream, [this, stream](const api::StreamData &data) {
            ros::Time stamp = checkUpTimeStamp(
                data.img->timestamp, stream);
            static std::size_t count = 0;
            ++count;
            publishData(stream, data, count, stamp);
          });
      is_published_[stream] = true;
      return;
    }

    int disable_tag = 0;
    api_->DisableStreamData(stream, [&](const Stream &stream) {
            disable_tag += getStreamSubscribers(stream);
          }, true);
    if (disable_tag == 0 && is_published_[stream]) {
      api_->DisableStreamData(stream, [&](const Stream &stream) {
            api_->SetStreamCallback(stream, nullptr);
            is_published_[stream] = false;
          });
      return;
    }
  }

  void publishTopics() {
    publishMesh();
    if ((camera_publishers_[Stream::LEFT].getNumSubscribers() > 0 ||
        mono_publishers_[Stream::LEFT].getNumSubscribers() > 0) &&
        !is_published_[Stream::LEFT]) {
      api_->SetStreamCallback(
          Stream::LEFT, [&](const api::StreamData &data) {
            ++left_count_;
            if (left_count_ > 10) {
              // ros::Time stamp = hardTimeToSoftTime(data.img->timestamp);
              ros::Time stamp = checkUpTimeStamp(
                  data.img->timestamp, Stream::LEFT);
              if (skip_tag > 0) {
                if (skip_tmp_left_tag == 0) {
                  skip_tmp_left_tag = skip_tag;
                } else {
                  skip_tmp_left_tag--;
                  return;
                }
                if (left_timestamps.size() < MATCH_CHECK_THRESHOLD) {
                  left_timestamps.insert(left_timestamps.begin(), data.img->timestamp);
                } else {
                  left_timestamps.insert(left_timestamps.begin(), data.img->timestamp);
                  left_timestamps.pop_back();
                }
              }
              publishCamera(Stream::LEFT, data, left_count_, stamp);
              publishMono(Stream::LEFT, data, left_count_, stamp);
              NODELET_DEBUG_STREAM(
                  Stream::LEFT << ", count: " << left_count_
                      << ", frame_id: " << data.img->frame_id
                      << ", timestamp: " << data.img->timestamp
                      << ", is_ets: " << std::boolalpha << data.img->is_ets
                      << ", exposure_time: " << data.img->exposure_time);
            }
          });
      left_time_beg_ = ros::Time::now().toSec();
      is_published_[Stream::LEFT] = true;
    }

    if ((camera_publishers_[Stream::RIGHT].getNumSubscribers() > 0 ||
        mono_publishers_[Stream::RIGHT].getNumSubscribers() > 0) &&
        !is_published_[Stream::RIGHT]) {
      api_->SetStreamCallback(
          Stream::RIGHT, [&](const api::StreamData &data) {
            ++right_count_;
            if (right_count_ > 10) {
              // ros::Time stamp = hardTimeToSoftTime(data.img->timestamp);
              ros::Time stamp = checkUpTimeStamp(
                  data.img->timestamp, Stream::RIGHT);
              if (skip_tag > 0) {
                if (skip_tmp_right_tag == 0) {
                  skip_tmp_right_tag = skip_tag;
                } else {
                  skip_tmp_right_tag--;
                  return;
                }
                if (right_timestamps.size() < MATCH_CHECK_THRESHOLD) {
                  right_timestamps.insert(right_timestamps.begin(), data.img->timestamp);
                } else {
                  right_timestamps.insert(right_timestamps.begin(), data.img->timestamp);
                  right_timestamps.pop_back();
                  bool is_match = false;
                  for (size_t i = 0; i < right_timestamps.size(); i++) {
                    for (size_t j = 0; j < left_timestamps.size(); j++) {
                      if (right_timestamps[i] == left_timestamps[j]) {
                        is_match = true;
                        break;
                      }
                    }
                    if (is_match) {
                      break;
                    }
                  }

                  if (!is_match) {
                    std::cout << "find the output stamp can't matched try to fix with one skip step." << std::endl;
                    skip_tmp_right_tag++;
                  }
                }
              }
              publishCamera(Stream::RIGHT, data, right_count_, stamp);
              publishMono(Stream::RIGHT, data, right_count_, stamp);
              NODELET_DEBUG_STREAM(
                  Stream::RIGHT << ", count: " << right_count_
                      << ", frame_id: " << data.img->frame_id
                      << ", timestamp: " << data.img->timestamp
                      << ", is_ets: " << std::boolalpha << data.img->is_ets
                      << ", exposure_time: " << data.img->exposure_time);
            }
          });
      right_time_beg_ = ros::Time::now().toSec();
      is_published_[Stream::RIGHT] = true;
    }

    std::vector<Stream> other_streams{
        Stream::LEFT_RECTIFIED, Stream::RIGHT_RECTIFIED,
        Stream::DISPARITY,      Stream::DISPARITY_NORMALIZED,
        Stream::POINTS,         Stream::DEPTH
        };
    for (auto &&stream : other_streams) {
      publishOthers(stream);
    }

    if (!is_motion_published_) {
      api_->SetMotionCallback([this](const api::MotionData &data) {
      ros::Time stamp = checkUpImuTimeStamp(data.imu->timestamp);

      // static double imu_time_prev = -1;
      // NODELET_INFO_STREAM("ros_time_beg: " << FULL_PRECISION << ros_time_beg
      //     << ", imu_time_elapsed: " << FULL_PRECISION
      //     << ((data.imu->timestamp - imu_time_beg) * 0.00001f)
      //     << ", imu_time_diff: " << FULL_PRECISION
      //     << ((imu_time_prev < 0) ? 0
      //         : (data.imu->timestamp - imu_time_prev) * 0.01f) << " ms");
      // imu_time_prev = data.imu->timestamp;
      ++imu_count_;
      if (imu_count_ > 50) {
        if (publish_imu_by_sync_) {
          if (data.imu) {
            if (data.imu->flag == 1) {  // accelerometer
              imu_accel_ = data.imu;
              publishImuBySync();
            } else if (data.imu->flag == 2) {  // gyroscope
              imu_gyro_ = data.imu;
              publishImuBySync();
            } else {
              publishImu(data, imu_count_, stamp);
              publishTemperature(data.imu->temperature, imu_count_, stamp);
            }
          } else {
            NODELET_WARN_STREAM("Motion data is empty");
          }
        } else {
          publishImu(data, imu_count_, stamp);
          publishTemperature(data.imu->temperature, imu_count_, stamp);
        }
        NODELET_DEBUG_STREAM(
            "Imu count: " << imu_count_
                          << ", timestamp: " << data.imu->timestamp
                          << ", is_ets: " << std::boolalpha << data.imu->is_ets
                          << ", accel_x: " << data.imu->accel[0]
                          << ", accel_y: " << data.imu->accel[1]
                          << ", accel_z: " << data.imu->accel[2]
                          << ", gyro_x: " << data.imu->gyro[0]
                          << ", gyro_y: " << data.imu->gyro[1]
                          << ", gyro_z: " << data.imu->gyro[2]
                          << ", temperature: " << data.imu->temperature);
        // Sleep 1ms, otherwise publish may drop some datas.
        ros::Duration(0.001).sleep();
      }
      });
      imu_time_beg_ = ros::Time::now().toSec();
      is_motion_published_ = true;
    }

    if (!is_started_) {
      time_beg_ = ros::Time::now().toSec();
      api_->Start(Source::ALL);
      is_started_ = true;
    }
  }

  void publishCamera(
      const Stream &stream, const api::StreamData &data, std::uint32_t seq,
      ros::Time stamp) {
    // if (camera_publishers_[stream].getNumSubscribers() == 0)
    //   return;
    std_msgs::Header header;
    header.seq = seq;
    header.stamp = stamp;
    header.frame_id = frame_ids_[stream];
    pthread_mutex_lock(&mutex_data_);
    cv::Mat img = data.frame;
    if (stream == Stream::DISPARITY) {  // 32FC1 > 8UC1 = MONO8
      img.convertTo(img, CV_8UC1);
    }
    auto &&msg =
        cv_bridge::CvImage(header, camera_encodings_[stream], img).toImageMsg();
    pthread_mutex_unlock(&mutex_data_);
    auto &&info = getCameraInfo(stream);
    info->header.stamp = msg->header.stamp;
    info->header.frame_id = frame_ids_[stream];
    camera_publishers_[stream].publish(msg, info);
  }

  /*
  void publishImage(
      const Stream &stream, const api::StreamData &data, std::uint32_t seq,
      ros::Time stamp) {
    if (image_publishers_[stream].getNumSubscribers() == 0)
      return;
    std_msgs::Header header;
    header.seq = seq;
    header.stamp = stamp;
    header.frame_id = frame_ids_[stream];
    cv::Mat img = data.frame;
    if (stream == Stream::DISPARITY) {  // 32FC1 > 8UC1 = MONO8
      img.convertTo(img, CV_8UC1);
    }
    auto &&msg =
        cv_bridge::CvImage(header, image_encodings_[stream], img).toImageMsg();
    image_publishers_[stream].publish(msg);
  }
  */

  void publishMono(
      const Stream &stream, const api::StreamData &data, std::uint32_t seq,
      ros::Time stamp) {
    if (mono_publishers_[stream].getNumSubscribers() == 0)
      return;
    std_msgs::Header header;
    header.seq = seq;
    header.stamp = stamp;
    header.frame_id = frame_ids_[stream];
    pthread_mutex_lock(&mutex_data_);
    cv::Mat mono;
    cv::cvtColor(data.frame, mono, CV_RGB2GRAY);
    auto &&msg = cv_bridge::CvImage(header, enc::MONO8, mono).toImageMsg();
    pthread_mutex_unlock(&mutex_data_);
    mono_publishers_[stream].publish(msg);
  }

  void publishPoints(
      const api::StreamData &data, std::uint32_t seq, ros::Time stamp) {
    // if (points_publisher_.getNumSubscribers() == 0)
    //   return;

    auto &&in = api_->GetIntrinsicsBase(Stream::LEFT);

    sensor_msgs::PointCloud2 msg;
    msg.header.seq = seq;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_ids_[Stream::POINTS];
    msg.width = in->width;
    msg.height = in->height;
    msg.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(msg);

    modifier.setPointCloud2Fields(
        4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
        sensor_msgs::PointField::FLOAT32, "z", 1,
        sensor_msgs::PointField::FLOAT32, "rgb", 1,
        sensor_msgs::PointField::FLOAT32);

    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg, "b");

    for (std::size_t y = 0; y < in->height; ++y) {
      for (std::size_t x = 0; x < in->width; ++x) {
        auto &&point = data.frame.at<cv::Vec3f>(y, x);

        *iter_x = point[2] * 0.001;
        *iter_y = 0.f - point[0] * 0.001;
        *iter_z = 0.f - point[1] * 0.001;

        *iter_r = static_cast<uint8_t>(255);
        *iter_g = static_cast<uint8_t>(255);
        *iter_b = static_cast<uint8_t>(255);

        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_r;
        ++iter_g;
        ++iter_b;
      }
    }

    points_publisher_.publish(msg);
  }

  void publishImu(
      const api::MotionData &data, std::uint32_t seq, ros::Time stamp) {
    if (pub_imu_.getNumSubscribers() == 0)
      return;

    sensor_msgs::Imu msg;

    msg.header.seq = seq;
    msg.header.stamp = stamp;
    msg.header.frame_id = imu_frame_id_;

    // acceleration should be in m/s^2 (not in g's)
    msg.linear_acceleration.x = data.imu->accel[0] * gravity_;
    msg.linear_acceleration.y = data.imu->accel[1] * gravity_;
    msg.linear_acceleration.z = data.imu->accel[2] * gravity_;

    msg.linear_acceleration_covariance[0] = 0;
    msg.linear_acceleration_covariance[1] = 0;
    msg.linear_acceleration_covariance[2] = 0;

    msg.linear_acceleration_covariance[3] = 0;
    msg.linear_acceleration_covariance[4] = 0;
    msg.linear_acceleration_covariance[5] = 0;

    msg.linear_acceleration_covariance[6] = 0;
    msg.linear_acceleration_covariance[7] = 0;
    msg.linear_acceleration_covariance[8] = 0;

    // velocity should be in rad/sec
    msg.angular_velocity.x = data.imu->gyro[0] * M_PI / 180;
    msg.angular_velocity.y = data.imu->gyro[1] * M_PI / 180;
    msg.angular_velocity.z = data.imu->gyro[2] * M_PI / 180;

    msg.angular_velocity_covariance[0] = 0;
    msg.angular_velocity_covariance[1] = 0;
    msg.angular_velocity_covariance[2] = 0;

    msg.angular_velocity_covariance[3] = 0;
    msg.angular_velocity_covariance[4] = 0;
    msg.angular_velocity_covariance[5] = 0;

    msg.angular_velocity_covariance[6] = 0;
    msg.angular_velocity_covariance[7] = 0;
    msg.angular_velocity_covariance[8] = 0;

    pub_imu_.publish(msg);
  }

  void timestampAlign() {
    static std::vector<ImuData> acc_buf;
    static std::vector<ImuData> gyro_buf;

    if (imu_accel_ != nullptr) {
      acc_buf.push_back(*imu_accel_);
    }

    if (imu_gyro_ != nullptr) {
      gyro_buf.push_back(*imu_gyro_);
    }

    imu_accel_ = nullptr;
    imu_gyro_ = nullptr;

    imu_align_.clear();

    if (acc_buf.empty() || gyro_buf.empty()) {
      return;
    }

    ImuData imu_temp;
    auto itg = gyro_buf.end();
    auto ita = acc_buf.end();
    for (auto it_gyro = gyro_buf.begin();
        it_gyro != gyro_buf.end(); it_gyro++) {
      for (auto it_acc = acc_buf.begin();
          it_acc+1 != acc_buf.end(); it_acc++) {
        if (it_gyro->timestamp >= it_acc->timestamp
            && it_gyro->timestamp <= (it_acc+1)->timestamp) {
          double k = static_cast<double>((it_acc+1)->timestamp - it_acc->timestamp);
          k = static_cast<double>(it_gyro->timestamp - it_acc->timestamp) / k;

          imu_temp = *it_gyro;
          imu_temp.accel[0] = it_acc->accel[0] + ((it_acc+1)->accel[0] - it_acc->accel[0]) * k;
          imu_temp.accel[1] = it_acc->accel[1] + ((it_acc+1)->accel[1] - it_acc->accel[1]) * k;
          imu_temp.accel[2] = it_acc->accel[2] + ((it_acc+1)->accel[2] - it_acc->accel[2]) * k;

          imu_align_.push_back(imu_temp);

          itg = it_gyro;
          ita = it_acc;
        }
      }
    }

    if (itg != gyro_buf.end()) {
      gyro_buf.erase(gyro_buf.begin(), itg + 1);
    }

    if (ita != acc_buf.end()) {
      acc_buf.erase(acc_buf.begin(), ita);
    }
  }

  void publishImuBySync() {
    timestampAlign();

    for (int i = 0; i < imu_align_.size(); i++) {
      sensor_msgs::Imu msg;

      msg.header.seq = imu_sync_count_;
      ros::Time stamp = checkUpImuTimeStamp(imu_align_[i].timestamp);
      msg.header.stamp = stamp;
      msg.header.frame_id = imu_frame_id_;

      // acceleration should be in m/s^2 (not in g's)
      msg.linear_acceleration.x = imu_align_[i].accel[0] * gravity_;
      msg.linear_acceleration.y = imu_align_[i].accel[1] * gravity_;
      msg.linear_acceleration.z = imu_align_[i].accel[2] * gravity_;

      msg.linear_acceleration_covariance[0] = 0;
      msg.linear_acceleration_covariance[1] = 0;
      msg.linear_acceleration_covariance[2] = 0;

      msg.linear_acceleration_covariance[3] = 0;
      msg.linear_acceleration_covariance[4] = 0;
      msg.linear_acceleration_covariance[5] = 0;

      msg.linear_acceleration_covariance[6] = 0;
      msg.linear_acceleration_covariance[7] = 0;
      msg.linear_acceleration_covariance[8] = 0;

      // velocity should be in rad/sec
      msg.angular_velocity.x = imu_align_[i].gyro[0] * M_PI / 180;
      msg.angular_velocity.y = imu_align_[i].gyro[1] * M_PI / 180;
      msg.angular_velocity.z = imu_align_[i].gyro[2] * M_PI / 180;

      msg.angular_velocity_covariance[0] = 0;
      msg.angular_velocity_covariance[1] = 0;
      msg.angular_velocity_covariance[2] = 0;

      msg.angular_velocity_covariance[3] = 0;
      msg.angular_velocity_covariance[4] = 0;
      msg.angular_velocity_covariance[5] = 0;

      msg.angular_velocity_covariance[6] = 0;
      msg.angular_velocity_covariance[7] = 0;
      msg.angular_velocity_covariance[8] = 0;

      pub_imu_.publish(msg);

      publishTemperature(imu_align_[i].temperature, imu_sync_count_, stamp);

      ++imu_sync_count_;
    }
  }

  void publishTemperature(
    float temperature, std::uint32_t seq, ros::Time stamp) {
    if (pub_temperature_.getNumSubscribers() == 0)
      return;
    sensor_msgs::Temperature msg;
    msg.header.seq = seq;
    msg.header.stamp = stamp;
    msg.header.frame_id = temperature_frame_id_;
    msg.temperature = temperature;
    msg.variance = 0;
    pub_temperature_.publish(msg);
  }

 private:
  void initDevice() {
    std::shared_ptr<Device> device = nullptr;

    device = selectDevice();

    api_ = API::Create(device);
    auto &&requests = device->GetStreamRequests();
    std::size_t m = requests.size();
    int request_index = 0;

    model_ = api_->GetModel();
    if (model_ == Model::STANDARD2 ||
        model_ == Model::STANDARD210A || model_ == Model::STANDARD200B) {
      private_nh_.getParamCached("standard2/request_index", request_index);
      switch (request_index) {
        case 0:
        case 4:
          frame_rate_ = 10;
          break;
        case 1:
        case 5:
          frame_rate_ = 20;
          break;
        case 2:
        case 6:
          frame_rate_ = 30;
          break;
        case 3:
          frame_rate_ = 60;
          break;
      }
    }
    if (model_ == Model::STANDARD) {
      private_nh_.getParamCached("standard/request_index", request_index);
      frame_rate_ = api_->GetOptionValue(Option::FRAME_RATE);
    }

    std::int32_t process_mode = 0;
    if (model_ == Model::STANDARD2 ||
        model_ == Model::STANDARD210A || model_ == Model::STANDARD200B) {
      private_nh_.getParamCached("standard2/imu_process_mode", process_mode);
      api_->EnableProcessMode(process_mode);
    }

    NODELET_FATAL_COND(m <= 0, "No MYNT EYE devices :(");
    if (m <= 1) {
      NODELET_INFO_STREAM("Only one stream request, select index: 0");
      api_->ConfigStreamRequest(requests[0]);
    } else {
      if (request_index >= m) {
        NODELET_WARN_STREAM("Resquest_index out of range");
        api_->ConfigStreamRequest(requests[0]);
      } else {
        api_->ConfigStreamRequest(requests[request_index]);
      }
    }

    computeRectTransforms();
  }

  std::shared_ptr<Device> selectDevice() {
    NODELET_INFO_STREAM("Detecting MYNT EYE devices");

    Context context;
    auto &&devices = context.devices();

    size_t n = devices.size();
    NODELET_FATAL_COND(n <= 0, "No MYNT EYE devices :(");

    NODELET_INFO_STREAM("MYNT EYE devices:");
    for (size_t i = 0; i < n; i++) {
      auto &&device = devices[i];
      auto &&name = device->GetInfo(Info::DEVICE_NAME);
      auto &&serial_number = device->GetInfo(Info::SERIAL_NUMBER);
      NODELET_INFO_STREAM("  index: " << i << ", name: " <<
          name << ", serial number: " << serial_number);
    }

    bool is_multiple = false;
    private_nh_.getParam("is_multiple", is_multiple);
    if (is_multiple) {
      std::string sn;
      private_nh_.getParam("serial_number", sn);
      NODELET_FATAL_COND(sn.empty(), "Must set serial_number "
          "in mynteye_1.launch and mynteye_2.launch.");

      for (size_t i = 0; i < n; i++) {
        auto &&device = devices[i];
        auto &&name = device->GetInfo(Info::DEVICE_NAME);
        auto &&serial_number = device->GetInfo(Info::SERIAL_NUMBER);
        if (sn == serial_number)
          return device;
        NODELET_FATAL_COND(i == (n - 1), "No corresponding device was found,"
            " check the serial_number configuration. ");
      }
    } else {
      if (n <= 1) {
        NODELET_INFO_STREAM("Only one MYNT EYE device, select index: 0");
        return devices[0];
      } else {
        while (true) {
          size_t i;
          NODELET_INFO_STREAM(
              "There are " << n << " MYNT EYE devices, select index: ");
          std::cin >> i;
          if (i >= n) {
            NODELET_WARN_STREAM("Index out of range :(");
            continue;
          }
          return devices[i];
        }
      }
    }

    return nullptr;
  }

  std::shared_ptr<IntrinsicsBase> getDefaultIntrinsics() {
    auto res = std::make_shared<IntrinsicsPinhole>();
    res->width = 640;
    res->height = 400;
    res->model = 0;
    res->fx = 3.6220059643202876e+02;
    res->fy = 3.6350065250745848e+02;
    res->cx = 4.0658699068023441e+02;
    res->cy = 2.3435161110061483e+02;
    double codffs[5] = {
      -2.5034765682756088e-01,
      5.0579399202897619e-02,
      -7.0536676161976066e-04,
      -8.5255451307033846e-03,
      0.
    };
    for (unsigned int i = 0; i < 5; i++) {
      res->coeffs[i] = codffs[i];
    }
    return res;
  }

  std::shared_ptr<Extrinsics> getDefaultExtrinsics() {
    auto res = std::make_shared<Extrinsics>();
    double rotation[9] = {
      9.9867908939669447e-01,  -6.3445566137485428e-03, 5.0988459509619687e-02,
      5.9890316389333252e-03,  9.9995670037792639e-01,  7.1224201868366971e-03,
      -5.1031440326695092e-02, -6.8076406092671274e-03, 9.9867384471984544e-01
    };
    double translation[3] = {-1.2002489764113250e+02, -1.1782637409050747e+00,
        -5.2058205159996538e+00};
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 3; j++) {
        res->rotation[i][j] = rotation[i*3 + j];
      }
    }
    for (unsigned int i = 0; i < 3; i++) {
      res->translation[i] = translation[i];
    }
    return res;
  }

  void publishMesh() {
    mesh_msg_.header.frame_id = base_frame_id_;
    mesh_msg_.header.stamp = ros::Time::now();
    mesh_msg_.type = visualization_msgs::Marker::MESH_RESOURCE;
    geometry_msgs::Quaternion q;
    private_nh_.getParamCached("model_rotation_x",
        mesh_rotation_x);
    private_nh_.getParamCached("model_rotation_y",
        mesh_rotation_y);
    private_nh_.getParamCached("model_rotation_z",
        mesh_rotation_z);
    private_nh_.getParamCached("model_position_x",
        mesh_position_x);
    private_nh_.getParamCached("model_position_y",
        mesh_position_y);
    private_nh_.getParamCached("model_position_z",
        mesh_position_z);

    q = tf::createQuaternionMsgFromRollPitchYaw(
        mesh_rotation_x,
        mesh_rotation_y,
        mesh_rotation_z);

    // fill orientation
    mesh_msg_.pose.orientation.x = q.x;
    mesh_msg_.pose.orientation.y = q.y;
    mesh_msg_.pose.orientation.z = q.z;
    mesh_msg_.pose.orientation.w = q.w;

    // fill position
    mesh_msg_.pose.position.x = mesh_position_x;
    mesh_msg_.pose.position.y = mesh_position_y;
    mesh_msg_.pose.position.z = mesh_position_z;

    // scale -- needed
    mesh_msg_.scale.x = 0.003;
    mesh_msg_.scale.y = 0.003;
    mesh_msg_.scale.z = 0.003;

    mesh_msg_.action = visualization_msgs::Marker::ADD;
    mesh_msg_.color.a = 0.5;  // Don't forget to set the alpha!
    mesh_msg_.color.r = 1.0;
    mesh_msg_.color.g = 1.0;
    mesh_msg_.color.b = 1.0;

    // embedded material / colour
    // mesh_msg_.mesh_use_embedded_materials = true;
    if (!mesh_msg_.mesh_resource.empty())
      pub_mesh_.publish(mesh_msg_);  // publish stamped mesh
  }


  void computeRectTransforms() {
    ROS_ASSERT(api_);
    auto in_left_base = api_->GetIntrinsicsBase(Stream::LEFT);
    auto in_right_base = api_->GetIntrinsicsBase(Stream::RIGHT);
    is_intrinsics_enable_ = in_left_base && in_right_base;
    if (is_intrinsics_enable_) {
      if (in_left_base->calib_model() != CalibrationModel::PINHOLE ||
          in_right_base->calib_model() != CalibrationModel::PINHOLE) {
        return;
      }
    } else {
      in_left_base = getDefaultIntrinsics();
      in_right_base = getDefaultIntrinsics();
    }

    auto in_left = *std::dynamic_pointer_cast<IntrinsicsPinhole>(in_left_base);
    auto in_right = *std::dynamic_pointer_cast<IntrinsicsPinhole>(
        in_right_base);
    auto ex_right_to_left = api_->GetExtrinsics(Stream::RIGHT, Stream::LEFT);
    if (!is_intrinsics_enable_) {
      ex_right_to_left = *(getDefaultExtrinsics());
    }

    cv::Size size{in_left.width, in_left.height};
    cv::Mat M1 =
        (cv::Mat_<double>(3, 3) << in_left.fx, 0, in_left.cx, 0, in_left.fy,
         in_left.cy, 0, 0, 1);
    cv::Mat M2 =
        (cv::Mat_<double>(3, 3) << in_right.fx, 0, in_right.cx, 0, in_right.fy,
         in_right.cy, 0, 0, 1);
    cv::Mat D1(1, 5, CV_64F, in_left.coeffs);
    cv::Mat D2(1, 5, CV_64F, in_right.coeffs);
    cv::Mat R =
        (cv::Mat_<double>(3, 3) << ex_right_to_left.rotation[0][0],
         ex_right_to_left.rotation[0][1], ex_right_to_left.rotation[0][2],
         ex_right_to_left.rotation[1][0], ex_right_to_left.rotation[1][1],
         ex_right_to_left.rotation[1][2], ex_right_to_left.rotation[2][0],
         ex_right_to_left.rotation[2][1], ex_right_to_left.rotation[2][2]);
    cv::Mat T(3, 1, CV_64F, ex_right_to_left.translation);

    cv::stereoRectify(
        M1, D1, M2, D2, size, R, T, left_r_, right_r_, left_p_, right_p_, q_,
        cv::CALIB_ZERO_DISPARITY, 0, size, &left_roi_, &right_roi_);

    NODELET_DEBUG_STREAM("left_r: " << left_r_);
    NODELET_DEBUG_STREAM("right_r: " << right_r_);
    NODELET_DEBUG_STREAM("left_p: " << left_p_);
    NODELET_DEBUG_STREAM("right_p: " << right_p_);
    NODELET_DEBUG_STREAM("q: " << q_);
  }

  sensor_msgs::CameraInfoPtr getCameraInfo(const Stream &stream) {
    if (camera_info_ptrs_.find(stream) != camera_info_ptrs_.end()) {
      return camera_info_ptrs_[stream];
    }
    ROS_ASSERT(api_);
    // http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html
    sensor_msgs::CameraInfo *camera_info = new sensor_msgs::CameraInfo();
    camera_info_ptrs_[stream] = sensor_msgs::CameraInfoPtr(camera_info);
    auto info_pair = api_->GetCameraROSMsgInfoPair();
    camera_info->width = info_pair->left.width;
    camera_info->height = info_pair->left.height;
    if (is_intrinsics_enable_) {
      if (stream == Stream::RIGHT ||
          stream == Stream::RIGHT_RECTIFIED) {
        if (info_pair->right.distortion_model == "KANNALA_BRANDT") {
          camera_info->distortion_model = "KANNALA_BRANDT";
          for (size_t i = 0; i < 5; i++) {
            camera_info->D.push_back(info_pair->right.D[i]);
          }
        } else if (info_pair->right.distortion_model == "PINHOLE") {
          camera_info->distortion_model = "plumb_bob";
          for (size_t i = 0; i < 5; i++) {
            camera_info->D.push_back(info_pair->right.D[i]);
          }
        }
        for (size_t i = 0; i < 9; i++) {
          camera_info->K.at(i) = info_pair->right.K[i];
        }
        for (size_t i = 0; i < 9; i++) {
          camera_info->R.at(i) = info_pair->right.R[i];
        }
        for (size_t i = 0; i < 12; i++) {
          camera_info->P.at(i) = info_pair->right.P[i];
        }
      } else {
        if (info_pair->left.distortion_model == "KANNALA_BRANDT") {
          // compatible laserscan
          bool is_laserscan = false;
          private_nh_.getParamCached("is_laserscan", is_laserscan);
          if (!is_laserscan) {
            camera_info->distortion_model = "KANNALA_BRANDT";
            for (size_t i = 0; i < 5; i++) {
              camera_info->D.push_back(info_pair->left.D[i]);
            }
          } else {
            camera_info->distortion_model = "KANNALA_BRANDT";
            for (size_t i = 0; i < 4; i++) {
              camera_info->D.push_back(0.0);
            }
          }
        } else if (info_pair->left.distortion_model == "PINHOLE") {
          camera_info->distortion_model = "plumb_bob";
          for (size_t i = 0; i < 5; i++) {
            camera_info->D.push_back(info_pair->left.D[i]);
          }
        }
        for (size_t i = 0; i < 9; i++) {
          camera_info->K.at(i) = info_pair->left.K[i];
        }
        for (size_t i = 0; i < 9; i++) {
          camera_info->R.at(i) = info_pair->left.R[i];
        }
        for (size_t i = 0; i < 12; i++) {
          camera_info->P.at(i) = info_pair->left.P[i];
        }
      }
    }
    return camera_info_ptrs_[stream];
  }

  void publishStaticTransforms() {
    ros::Time tf_stamp = ros::Time::now();

    // The left frame is used as the base frame.
    geometry_msgs::TransformStamped b2l_msg;
    b2l_msg.header.stamp = tf_stamp;
    b2l_msg.header.frame_id = base_frame_id_;
    b2l_msg.child_frame_id = frame_ids_[Stream::LEFT];
    b2l_msg.transform.translation.x = 0;
    b2l_msg.transform.translation.y = 0;
    b2l_msg.transform.translation.z = 0;
    b2l_msg.transform.rotation.x = 0;
    b2l_msg.transform.rotation.y = 0;
    b2l_msg.transform.rotation.z = 0;
    b2l_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(b2l_msg);

    // Transform left frame to right frame
    auto &&l2r_ex = api_->GetExtrinsics(Stream::LEFT, Stream::RIGHT);
    tf::Quaternion l2r_q;
    tf::Matrix3x3 l2r_r(
        l2r_ex.rotation[0][0], l2r_ex.rotation[0][1], l2r_ex.rotation[0][2],
        l2r_ex.rotation[1][0], l2r_ex.rotation[1][1], l2r_ex.rotation[1][2],
        l2r_ex.rotation[2][0], l2r_ex.rotation[2][1], l2r_ex.rotation[2][2]);
    l2r_r.getRotation(l2r_q);
    geometry_msgs::TransformStamped l2r_msg;
    l2r_msg.header.stamp = tf_stamp;
    l2r_msg.header.frame_id = frame_ids_[Stream::LEFT];
    l2r_msg.child_frame_id = frame_ids_[Stream::RIGHT];
    l2r_msg.transform.translation.x = l2r_ex.translation[0] / 1000;
    l2r_msg.transform.translation.y = l2r_ex.translation[1] / 1000;
    l2r_msg.transform.translation.z = l2r_ex.translation[2] / 1000;
    l2r_msg.transform.rotation.x = l2r_q.getX();
    l2r_msg.transform.rotation.y = l2r_q.getY();
    l2r_msg.transform.rotation.z = l2r_q.getZ();
    l2r_msg.transform.rotation.w = l2r_q.getW();
    static_tf_broadcaster_.sendTransform(l2r_msg);

    // Transform left frame to left_rect frame
    geometry_msgs::TransformStamped l2rect_msg;
    l2rect_msg.header.stamp = tf_stamp;
    l2rect_msg.header.frame_id = frame_ids_[Stream::LEFT];
    l2rect_msg.child_frame_id = frame_ids_[Stream::LEFT_RECTIFIED];
    l2rect_msg.transform.translation.x = 0;
    l2rect_msg.transform.translation.y = 0;
    l2rect_msg.transform.translation.z = 0;
    l2rect_msg.transform.rotation.x = 0;
    l2rect_msg.transform.rotation.y = 0;
    l2rect_msg.transform.rotation.z = 0;
    l2rect_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(l2rect_msg);

    // Transform right frame to right_rect frame
    geometry_msgs::TransformStamped r2rect_msg;
    r2rect_msg.header.stamp = tf_stamp;
    r2rect_msg.header.frame_id = frame_ids_[Stream::RIGHT];
    r2rect_msg.child_frame_id = frame_ids_[Stream::RIGHT_RECTIFIED];
    r2rect_msg.transform.translation.x = 0;
    r2rect_msg.transform.translation.y = 0;
    r2rect_msg.transform.translation.z = 0;
    r2rect_msg.transform.rotation.x = 0;
    r2rect_msg.transform.rotation.y = 0;
    r2rect_msg.transform.rotation.z = 0;
    r2rect_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(r2rect_msg);

    // Transform left frame to disparity frame
    geometry_msgs::TransformStamped l2disp_msg;
    l2disp_msg.header.stamp = tf_stamp;
    l2disp_msg.header.frame_id = frame_ids_[Stream::LEFT];
    l2disp_msg.child_frame_id = frame_ids_[Stream::DISPARITY];
    l2disp_msg.transform.translation.x = 0;
    l2disp_msg.transform.translation.y = 0;
    l2disp_msg.transform.translation.z = 0;
    l2disp_msg.transform.rotation.x = 0;
    l2disp_msg.transform.rotation.y = 0;
    l2disp_msg.transform.rotation.z = 0;
    l2disp_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(l2disp_msg);

    // Transform disparity frame to disparity_norm frame
    geometry_msgs::TransformStamped disp2norm_msg;
    disp2norm_msg.header.stamp = tf_stamp;
    disp2norm_msg.header.frame_id = frame_ids_[Stream::DISPARITY];
    disp2norm_msg.child_frame_id = frame_ids_[Stream::DISPARITY_NORMALIZED];
    disp2norm_msg.transform.translation.x = 0;
    disp2norm_msg.transform.translation.y = 0;
    disp2norm_msg.transform.translation.z = 0;
    disp2norm_msg.transform.rotation.x = 0;
    disp2norm_msg.transform.rotation.y = 0;
    disp2norm_msg.transform.rotation.z = 0;
    disp2norm_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(disp2norm_msg);

    // Transform left frame to depth frame
    geometry_msgs::TransformStamped b2d_msg;
    b2d_msg.header.stamp = tf_stamp;
    b2d_msg.header.frame_id = frame_ids_[Stream::LEFT];
    b2d_msg.child_frame_id = frame_ids_[Stream::DEPTH];
    b2d_msg.transform.translation.x = 0;
    b2d_msg.transform.translation.y = 0;
    b2d_msg.transform.translation.z = 0;
    b2d_msg.transform.rotation.x = 0;
    b2d_msg.transform.rotation.y = 0;
    b2d_msg.transform.rotation.z = 0;
    b2d_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(b2d_msg);

    // Transform left frame to points frame
    geometry_msgs::TransformStamped b2p_msg;
    b2p_msg.header.stamp = tf_stamp;
    b2p_msg.header.frame_id = frame_ids_[Stream::LEFT];
    b2p_msg.child_frame_id = frame_ids_[Stream::POINTS];
    b2p_msg.transform.translation.x = 0;
    b2p_msg.transform.translation.y = 0;
    b2p_msg.transform.translation.z = 0;
    b2p_msg.transform.rotation.x = 0;
    b2p_msg.transform.rotation.y = 0;
    b2p_msg.transform.rotation.z = 0;
    b2p_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(b2p_msg);

    // Transform left frame to imu frame
    auto &&l2i_ex = api_->GetMotionExtrinsics(Stream::LEFT);
    geometry_msgs::TransformStamped l2i_msg;
    l2i_msg.header.stamp = tf_stamp;
    l2i_msg.header.frame_id = frame_ids_[Stream::LEFT];
    l2i_msg.child_frame_id = imu_frame_id_;
    bool is_data_use_mm_instead_of_m = abs(l2i_ex.translation[0]) > 1.0 ||
                                       abs(l2i_ex.translation[1]) > 1.0 ||
                                       abs(l2i_ex.translation[2]) > 1.0;
    if (is_data_use_mm_instead_of_m) {
      l2i_msg.transform.translation.x = l2i_ex.translation[0] * 0.001;
      l2i_msg.transform.translation.y = l2i_ex.translation[1] * 0.001;
      l2i_msg.transform.translation.z = l2i_ex.translation[2] * 0.001;
    } else {
      l2i_msg.transform.translation.x = l2i_ex.translation[0];
      l2i_msg.transform.translation.y = l2i_ex.translation[1];
      l2i_msg.transform.translation.z = l2i_ex.translation[2];
    }
    // LOG(INFO) << std::endl << "l2i_msg.transform.translation.x: "
    //           << l2i_msg.transform.translation.x << std::endl
    //           << "l2i_msg.transform.translation.y: "
    //           << l2i_msg.transform.translation.y << std::endl
    //           << "l2i_msg.transform.translation.z: "
    //           << l2i_msg.transform.translation.z << std::endl;
    if (l2i_ex.rotation[0][0] == 0 && l2i_ex.rotation[2][2] == 0) {
      l2i_msg.transform.rotation.x = 0;
      l2i_msg.transform.rotation.y = 0;
      l2i_msg.transform.rotation.z = 0;
      l2i_msg.transform.rotation.w = 1;
    } else {
      tf::Quaternion l2i_q;
      tf::Matrix3x3 l2i_r(
          l2i_ex.rotation[0][0], l2i_ex.rotation[0][1], l2i_ex.rotation[0][2],
          l2i_ex.rotation[1][0], l2i_ex.rotation[1][1], l2i_ex.rotation[1][2],
          l2i_ex.rotation[2][0], l2i_ex.rotation[2][1], l2i_ex.rotation[2][2]);
      l2i_r.getRotation(l2i_q);
      l2i_msg.transform.rotation.x = l2i_q.getX();
      l2i_msg.transform.rotation.y = l2i_q.getY();
      l2i_msg.transform.rotation.z = l2i_q.getZ();
      l2i_msg.transform.rotation.w = l2i_q.getW();
    }
    static_tf_broadcaster_.sendTransform(l2i_msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  pthread_mutex_t mutex_data_;

  Model model_;
  std::map<Option, std::string> option_names_;
  // camera:
  //   LEFT, RIGHT, LEFT_RECTIFIED, RIGHT_RECTIFIED,
  //   DISPARITY, DISPARITY_NORMALIZED,
  //   DEPTH
  std::map<Stream, image_transport::CameraPublisher> camera_publishers_;
  std::map<Stream, sensor_msgs::CameraInfoPtr> camera_info_ptrs_;
  std::map<Stream, std::string> camera_encodings_;

  // image: LEFT_RECTIFIED, RIGHT_RECTIFIED, DISPARITY, DISPARITY_NORMALIZED,
  // DEPTH
  std::map<Stream, image_transport::Publisher> image_publishers_;
  std::map<Stream, std::string> image_encodings_;

  // mono: LEFT, RIGHT
  std::map<Stream, image_transport::Publisher> mono_publishers_;

  // pointcloud: POINTS
  ros::Publisher points_publisher_;

  ros::Publisher pub_imu_;
  ros::Publisher pub_temperature_;

  ros::Publisher pub_mesh_;  // < The publisher for camera mesh.
  visualization_msgs::Marker mesh_msg_;  // < Mesh message.

  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  ros::ServiceServer get_info_service_;

  // node params

  std::string base_frame_id_;
  std::string imu_frame_id_;
  std::string temperature_frame_id_;
  std::map<Stream, std::string> frame_ids_;

  double gravity_;

  // disparity type
  DisparityComputingMethod disparity_type_;
  // api

  std::shared_ptr<API> api_;

  // rectification transforms
  cv::Mat left_r_, right_r_, left_p_, right_p_, q_;
  cv::Rect left_roi_, right_roi_;

  double time_beg_ = -1;
  double left_time_beg_ = -1;
  double right_time_beg_ = -1;
  double imu_time_beg_ = -1;
  std::size_t left_count_ = 0;
  std::size_t right_count_ = 0;
  std::size_t imu_count_ = 0;
  std::size_t imu_sync_count_ = 0;
  std::shared_ptr<ImuData> imu_accel_;
  std::shared_ptr<ImuData> imu_gyro_;
  bool publish_imu_by_sync_ = true;
  std::map<Stream, bool> is_published_;
  bool is_motion_published_;
  bool is_started_;
  int frame_rate_;
  bool is_intrinsics_enable_;
  std::vector<ImuData> imu_align_;
  int skip_tag;
  int skip_tmp_left_tag;
  int skip_tmp_right_tag;
  double mesh_rotation_x;
  double mesh_rotation_y;
  double mesh_rotation_z;
  double mesh_position_x;
  double mesh_position_y;
  double mesh_position_z;
  std::vector<int64_t> left_timestamps;
  std::vector<int64_t> right_timestamps;

  std::uint64_t unit_hard_time = std::numeric_limits<std::uint32_t>::max();
};

MYNTEYE_END_NAMESPACE

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(mynteye::ROSWrapperNodelet, nodelet::Nodelet);
