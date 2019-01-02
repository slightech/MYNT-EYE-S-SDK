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
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <opencv2/calib3d/calib3d.hpp>

#include <mynt_eye_ros_wrapper/GetInfo.h>
#include <mynt_eye_ros_wrapper/Temp.h>

#define _USE_MATH_DEFINES
#include <cmath>
#include <map>
#include <string>

#include "mynteye/logger.h"
#include "mynteye/api/api.h"
#include "mynteye/device/context.h"
#include "mynteye/device/device.h"

#define FULL_PRECISION \
  std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)

MYNTEYE_BEGIN_NAMESPACE

namespace enc = sensor_msgs::image_encodings;
inline double compute_time(const double end, const double start) {
  return end - start;
}

class ROSWrapperNodelet : public nodelet::Nodelet {
 public:
  ROSWrapperNodelet() {
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

    return ros::Time(
        static_cast<double>(soft_time_begin +
        static_cast<double>(_hard_time - hard_time_begin) * 0.000001f));
  }

  inline bool is_overflow(std::uint64_t now,
      std::uint64_t pre) {
    static std::uint64_t unit =
      std::numeric_limits<std::uint32_t>::max();

    return (now < pre) && ((pre - now) > (unit / 2));
  }

  inline bool is_repeated(std::uint64_t now,
      std::uint64_t pre) {
    return now == pre;
  }

  inline bool is_abnormal(std::uint32_t now,
      std::uint32_t pre) {
    static std::uint64_t unit =
      std::numeric_limits<std::uint32_t>::max();

    return (now < pre) && ((pre - now) < (unit / 4));
  }

  ros::Time checkUpTimeStamp(std::uint64_t _hard_time,
      const Stream &stream) {
    static std::map<Stream, std::uint64_t> hard_time_now;
    static std::map<Stream, std::uint64_t> acc;
    static std::uint64_t unit_hard_time =
      std::numeric_limits<std::uint32_t>::max();

    if (is_overflow(_hard_time, hard_time_now[stream])) {
      acc[stream]++;
    }

    hard_time_now[stream] = _hard_time;

    return hardTimeToSoftTime(
        acc[stream] * unit_hard_time + _hard_time);
  }

  ros::Time checkUpImuTimeStamp(std::uint64_t _hard_time) {
    static std::uint64_t hard_time_now(0), acc(0);
    static std::uint64_t unit_hard_time =
      std::numeric_limits<std::uint32_t>::max();

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
      private_nh_.getParam(it->second + "_topic", stream_topics[it->first]);

      // if published init
      is_published_[it->first] = false;
    }
    is_motion_published_ = false;
    is_started_ = false;

    std::map<Stream, std::string> mono_names{{Stream::LEFT, "left_mono"},
                                             {Stream::RIGHT, "right_mono"}};

    std::map<Stream, std::string> mono_topics{};
    for (auto &&it = mono_names.begin(); it != mono_names.end(); ++it) {
      mono_topics[it->first] = it->second;
      private_nh_.getParam(it->second + "_topic", mono_topics[it->first]);
    }

    std::string imu_topic = "imu";
    std::string temp_topic = "temp";
    private_nh_.getParam("imu_topic", imu_topic);
    private_nh_.getParam("temp_topic", temp_topic);

    base_frame_id_ = "camera_link";
    private_nh_.getParam("base_frame_id", base_frame_id_);

    for (auto &&it = stream_names.begin(); it != stream_names.end(); ++it) {
      frame_ids_[it->first] = "camera_" + it->second + "_frame";
      private_nh_.getParam(it->second + "_frame_id", frame_ids_[it->first]);
    }

    imu_frame_id_ = "camera_imu_frame";
    temp_frame_id_ = "camera_temp_frame";
    private_nh_.getParam("imu_frame_id", imu_frame_id_);
    private_nh_.getParam("temp_frame_id", temp_frame_id_);

    gravity_ = 9.8;
    private_nh_.getParam("gravity", gravity_);

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
          {Option::GYROSCOPE_LOW_PASS_FILTER, "standard210a/gyro_low_filter"}};
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

    for (auto &&it = option_names_.begin(); it != option_names_.end(); ++it) {
      if (!api_->Supports(it->first))
        continue;
      int value = -1;
      private_nh_.getParam(it->second, value);
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

    // Only STANDARD2/STANDARD210A need publish mono_topics
    if (model_ == Model::STANDARD2 || model_ == Model::STANDARD210A) {
      for (auto &&it = mono_topics.begin(); it != mono_topics.end(); ++it) {
        auto &&topic = mono_topics[it->first];
        if (it->first == Stream::LEFT || it->first == Stream::RIGHT) {
          mono_publishers_[it->first] = it_mynteye.advertiseCamera(topic, 1);
        }
        NODELET_INFO_STREAM("Advertized on topic " << topic);
      }
    }

    if (model_ == Model::STANDARD2 || model_ == Model::STANDARD210A) {
      camera_encodings_ = {{Stream::LEFT, enc::BGR8},
                          {Stream::RIGHT, enc::BGR8},
                          {Stream::LEFT_RECTIFIED, enc::BGR8},
                          {Stream::RIGHT_RECTIFIED, enc::BGR8},
                          {Stream::DISPARITY, enc::MONO8},  // float
                          {Stream::DISPARITY_NORMALIZED, enc::MONO8},
                          {Stream::DEPTH, enc::MONO16}};
    }
    if (model_ == Model::STANDARD) {
      camera_encodings_ = {{Stream::LEFT, enc::MONO8},
                          {Stream::RIGHT, enc::MONO8},
                          {Stream::LEFT_RECTIFIED, enc::MONO8},
                          {Stream::RIGHT_RECTIFIED, enc::MONO8},
                          {Stream::DISPARITY, enc::MONO8},  // float
                          {Stream::DISPARITY_NORMALIZED, enc::MONO8},
                          {Stream::DEPTH, enc::MONO16}};
    }
    pub_imu_ = nh_.advertise<sensor_msgs::Imu>(imu_topic, 100);
    NODELET_INFO_STREAM("Advertized on topic " << imu_topic);

    pub_temp_ = nh_.advertise<mynt_eye_ros_wrapper::Temp>(temp_topic, 100);
    NODELET_INFO_STREAM("Advertized on topic " << temp_topic);

    // stream toggles

    for (auto &&it = stream_names.begin(); it != stream_names.end(); ++it) {
      if (it->first == Stream::LEFT || it->first == Stream::RIGHT) {
        continue;  // native streams
      } else {
        if (!api_->Supports(it->first))
          continue;
        bool enabled = false;
        private_nh_.getParam("enable_" + it->second, enabled);
        if (enabled) {
          api_->EnableStreamData(it->first);
          NODELET_INFO_STREAM("Enable stream data of " << it->first);
        }
      }
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
      default:
        NODELET_WARN_STREAM("Info of key " << req.key << " not exist");
        return false;
    }
    return true;
  }

  void SetIsPublished(const Stream &stream) {
    is_published_[stream] = false;
    switch (stream) {
      case Stream::LEFT_RECTIFIED: {
        if (is_published_[Stream::RIGHT_RECTIFIED]) {
          SetIsPublished(Stream::RIGHT_RECTIFIED);
        }
        if (is_published_[Stream::DISPARITY]) {
          SetIsPublished(Stream::DISPARITY);
        }
      } break;
      case Stream::RIGHT_RECTIFIED: {
        if (is_published_[Stream::LEFT_RECTIFIED]) {
          SetIsPublished(Stream::RIGHT_RECTIFIED);
        }
        if (is_published_[Stream::DISPARITY]) {
          SetIsPublished(Stream::DISPARITY);
        }
      } break;
      case Stream::DISPARITY: {
        if (is_published_[Stream::DISPARITY_NORMALIZED]) {
          SetIsPublished(Stream::DISPARITY_NORMALIZED);
        }
        if (is_published_[Stream::POINTS]) {
          SetIsPublished(Stream::POINTS);
        }
      } break;
      case Stream::DISPARITY_NORMALIZED: {
      } break;
      case Stream::POINTS: {
        if (is_published_[Stream::DEPTH]) {
          SetIsPublished(Stream::DEPTH);
        }
      } break;
      case Stream::DEPTH: {
      } break;
      default:
        return;
    }
  }

  void publishPoint(const Stream &stream) {
    auto &&points_num = points_publisher_.getNumSubscribers();
    if (points_num == 0 && is_published_[stream]) {
      SetIsPublished(stream);
      api_->DisableStreamData(stream);
    } else if (points_num > 0 && !is_published_[Stream::POINTS]) {
      api_->EnableStreamData(Stream::POINTS);
      api_->SetStreamCallback(
          Stream::POINTS, [this](const api::StreamData &data) {
            // ros::Time stamp = hardTimeToSoftTime(data.img->timestamp);
            ros::Time stamp = checkUpTimeStamp(
                data.img->timestamp, Stream::POINTS);
            static std::size_t count = 0;
            ++count;
            publishPoints(data, count, stamp);
          });
      is_published_[Stream::POINTS] = true;
    }
  }

  void publishOthers(const Stream &stream) {
    auto stream_num = camera_publishers_[stream].getNumSubscribers();
    if (stream_num == 0 && is_published_[stream]) {
      // Stop computing when was not subcribed
      SetIsPublished(stream);
      api_->DisableStreamData(stream);
    } else if (stream_num > 0 && !is_published_[stream]) {
      // Start computing and publishing when was subcribed
      api_->EnableStreamData(stream);
      api_->SetStreamCallback(
          stream, [this, stream](const api::StreamData &data) {
            // ros::Time stamp = hardTimeToSoftTime(data.img->timestamp);
            ros::Time stamp = checkUpTimeStamp(
                data.img->timestamp, stream);
            static std::size_t count = 0;
            ++count;
            publishCamera(stream, data, count, stamp);
          });
      is_published_[stream] = true;
    }
  }

  void publishTopics() {
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

              // static double img_time_prev = -1;
              // NODELET_INFO_STREAM("ros_time_beg: " << FULL_PRECISION <<
              // ros_time_beg
              //     << ", img_time_elapsed: " << FULL_PRECISION
              //     << ((data.img->timestamp - img_time_beg) * 0.00001f)
              //     << ", img_time_diff: " << FULL_PRECISION
              //     << ((img_time_prev < 0) ? 0
              //         : (data.img->timestamp - img_time_prev) * 0.01f) << "
              //         ms");
              // img_time_prev = data.img->timestamp;
              publishCamera(Stream::LEFT, data, left_count_, stamp);
              publishMono(Stream::LEFT, data, left_count_, stamp);
              NODELET_DEBUG_STREAM(
                  Stream::LEFT << ", count: " << left_count_
                      << ", frame_id: " << data.img->frame_id
                      << ", timestamp: " << data.img->timestamp
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
              publishCamera(Stream::RIGHT, data, right_count_, stamp);
              publishMono(Stream::RIGHT, data, right_count_, stamp);
              NODELET_DEBUG_STREAM(
                  Stream::RIGHT << ", count: " << right_count_
                      << ", frame_id: " << data.img->frame_id
                      << ", timestamp: " << data.img->timestamp
                      << ", exposure_time: " << data.img->exposure_time);
            }
          });
      right_time_beg_ = ros::Time::now().toSec();
      is_published_[Stream::RIGHT] = true;
    }

    std::vector<Stream> other_streams{
        Stream::LEFT_RECTIFIED, Stream::RIGHT_RECTIFIED,
        Stream::DISPARITY,      Stream::DISPARITY_NORMALIZED,
        Stream::POINTS,         Stream::DEPTH};

    for (auto &&stream : other_streams) {
      if (stream != Stream::POINTS) {
        publishOthers(stream);
      } else {
        publishPoint(stream);
      }
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
              publishImuBySync(stamp);
            } else if (data.imu->flag == 2) {  // gyroscope
              imu_gyro_ = data.imu;
              publishImuBySync(stamp);
            } else {
              publishImu(data, imu_count_, stamp);
              publishTemp(data.imu->temperature, imu_count_, stamp);
            }
          } else {
            NODELET_WARN_STREAM("Motion data is empty");
          }
        } else {
          publishImu(data, imu_count_, stamp);
          publishTemp(data.imu->temperature, imu_count_, stamp);
        }
        NODELET_DEBUG_STREAM(
            "Imu count: " << imu_count_
                          << ", timestamp: " << data.imu->timestamp
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
    cv::Mat mono;
    cv::cvtColor(data.frame, mono, CV_RGB2GRAY);
    auto &&msg = cv_bridge::CvImage(header, enc::MONO8, mono).toImageMsg();
    auto &&info = getCameraInfo(stream);
    info->header.stamp = msg->header.stamp;
    mono_publishers_[stream].publish(msg, info);
  }

  void publishPoints(
      const api::StreamData &data, std::uint32_t seq, ros::Time stamp) {
    // if (points_publisher_.getNumSubscribers() == 0)
    //   return;

    auto &&in = api_->GetIntrinsics(Stream::LEFT);

    sensor_msgs::PointCloud2 msg;
    msg.header.seq = seq;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_ids_[Stream::POINTS];
    msg.width = in.width;
    msg.height = in.height;
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

    for (std::size_t y = 0; y < in.height; ++y) {
      for (std::size_t x = 0; x < in.width; ++x) {
        auto &&point = data.frame.at<cv::Vec3f>(y, x);

        *iter_x = point[0] * 0.001;
        *iter_y = point[1] * 0.001;
        *iter_z = point[2] * 0.001;

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

  void publishImuBySync(ros::Time stamp) {
    if (imu_accel_ == nullptr || imu_gyro_ == nullptr) {
      return;
    }
    sensor_msgs::Imu msg;

    msg.header.seq = imu_sync_count_;
    msg.header.stamp = stamp;
    msg.header.frame_id = imu_frame_id_;

    // acceleration should be in m/s^2 (not in g's)
    msg.linear_acceleration.x = imu_accel_->accel[0] * gravity_;
    msg.linear_acceleration.y = imu_accel_->accel[1] * gravity_;
    msg.linear_acceleration.z = imu_accel_->accel[2] * gravity_;

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
    msg.angular_velocity.x = imu_gyro_->gyro[0] * M_PI / 180;
    msg.angular_velocity.y = imu_gyro_->gyro[1] * M_PI / 180;
    msg.angular_velocity.z = imu_gyro_->gyro[2] * M_PI / 180;

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

    publishTemp(imu_accel_->temperature, imu_sync_count_, stamp);

    ++imu_sync_count_;
    imu_accel_ = nullptr;
    imu_gyro_ = nullptr;
  }

  void publishTemp(float temperature, std::uint32_t seq, ros::Time stamp) {
    if (pub_temp_.getNumSubscribers() == 0)
      return;
    mynt_eye_ros_wrapper::Temp msg;
    msg.header.seq = seq;
    msg.header.stamp = stamp;
    msg.header.frame_id = temp_frame_id_;
    msg.data = temperature;
    pub_temp_.publish(msg);
  }

 private:
  void initDevice() {
    NODELET_INFO_STREAM("Detecting MYNT EYE devices");

    Context context;
    auto &&devices = context.devices();

    size_t n = devices.size();
    NODELET_FATAL_COND(n <= 0, "No MYNT EYE devices :(");

    NODELET_INFO_STREAM("MYNT EYE devices:");
    for (size_t i = 0; i < n; i++) {
      auto &&device = devices[i];
      auto &&name = device->GetInfo(Info::DEVICE_NAME);
      NODELET_INFO_STREAM("  index: " << i << ", name: " << name);
    }

    std::shared_ptr<Device> device = nullptr;
    if (n <= 1) {
      device = devices[0];
      NODELET_INFO_STREAM("Only one MYNT EYE device, select index: 0");
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
        device = devices[i];
        break;
      }
    }

    api_ = API::Create(device);
    auto &&requests = device->GetStreamRequests();
    std::size_t m = requests.size();
    int request_index = 0;

    model_ = api_->GetModel();
    if (model_ == Model::STANDARD2 || model_ == Model::STANDARD210A) {
      private_nh_.getParam("request_index", request_index);
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
      request_index = 0;
      frame_rate_ = api_->GetOptionValue(Option::FRAME_RATE);
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

  void computeRectTransforms() {
    ROS_ASSERT(api_);
    auto &&in_left = api_->GetIntrinsics(Stream::LEFT);
    auto &&in_right = api_->GetIntrinsics(Stream::RIGHT);
    auto &&ex_right_to_left = api_->GetExtrinsics(Stream::RIGHT, Stream::LEFT);

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

    // http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html
    sensor_msgs::CameraInfo *camera_info = new sensor_msgs::CameraInfo();
    camera_info_ptrs_[stream] = sensor_msgs::CameraInfoPtr(camera_info);

    Intrinsics in;
    if (stream == Stream::RIGHT || stream == Stream::RIGHT_RECTIFIED) {
      in = api_->GetIntrinsics(Stream::RIGHT);
    } else {
      in = api_->GetIntrinsics(Stream::LEFT);
    }

    camera_info->header.frame_id = frame_ids_[stream];
    camera_info->width = in.width;
    camera_info->height = in.height;

    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    camera_info->K.at(0) = in.fx;
    camera_info->K.at(2) = in.cx;
    camera_info->K.at(4) = in.fy;
    camera_info->K.at(5) = in.cy;
    camera_info->K.at(8) = 1;

    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    cv::Mat p = left_p_;
    if (stream == Stream::RIGHT || stream == Stream::RIGHT_RECTIFIED) {
      p = right_p_;
    }
    for (int i = 0; i < p.rows; i++) {
      for (int j = 0; j < p.cols; j++) {
        camera_info->P.at(i * p.cols + j) = p.at<double>(i, j);
      }
    }

    camera_info->distortion_model = "plumb_bob";

    // D of plumb_bob: (k1, k2, t1, t2, k3)
    for (int i = 0; i < 5; i++) {
      camera_info->D.push_back(in.coeffs[i]);
    }

    // R to identity matrix
    camera_info->R.at(0) = 1.0;
    camera_info->R.at(1) = 0.0;
    camera_info->R.at(2) = 0.0;
    camera_info->R.at(3) = 0.0;
    camera_info->R.at(4) = 1.0;
    camera_info->R.at(5) = 0.0;
    camera_info->R.at(6) = 0.0;
    camera_info->R.at(7) = 0.0;
    camera_info->R.at(8) = 1.0;

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
    l2r_msg.transform.translation.x = l2r_ex.translation[0];
    l2r_msg.transform.translation.y = l2r_ex.translation[1];
    l2r_msg.transform.translation.z = l2r_ex.translation[2];
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
    l2i_msg.transform.translation.x = l2i_ex.translation[0];
    l2i_msg.transform.translation.y = l2i_ex.translation[1];
    l2i_msg.transform.translation.z = l2i_ex.translation[2];
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
  std::map<Stream, image_transport::CameraPublisher> mono_publishers_;

  // pointcloud: POINTS
  ros::Publisher points_publisher_;

  ros::Publisher pub_imu_;
  ros::Publisher pub_temp_;

  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  ros::ServiceServer get_info_service_;

  // node params

  std::string base_frame_id_;
  std::string imu_frame_id_;
  std::string temp_frame_id_;
  std::map<Stream, std::string> frame_ids_;

  double gravity_;

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
  bool publish_imu_by_sync_ = false;
  std::map<Stream, bool> is_published_;
  bool is_motion_published_;
  bool is_started_;
  int frame_rate_;
};

MYNTEYE_END_NAMESPACE

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(mynteye::ROSWrapperNodelet, nodelet::Nodelet);
