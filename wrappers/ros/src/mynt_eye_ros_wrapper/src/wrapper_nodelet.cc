#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <mynt_eye_ros_wrapper/Temp.h>

#include <glog/logging.h>

#define _USE_MATH_DEFINES
#include <cmath>
#include <map>
#include <string>

#include "device/context.h"
#include "device/device.h"

#define FULL_PRECISION \
  std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)

MYNTEYE_BEGIN_NAMESPACE

namespace enc = sensor_msgs::image_encodings;

class ROSWrapperNodelet : public nodelet::Nodelet {
 public:
  ROSWrapperNodelet() {}

  ~ROSWrapperNodelet() {
    // std::cout << __func__ << std::endl;
    if (device_) {
      device_->Stop(Source::ALL);
    }
    if (time_beg_ != -1) {
      double time_end = ros::Time::now().toSec();
      double time_elapsed = time_end - time_beg_;

      LOG(INFO) << "Time elapsed: " << time_elapsed << "s";
      LOG(INFO) << "Left count: " << left_count_
                << ", fps: " << (left_count_ / time_elapsed);
      LOG(INFO) << "Right count: " << right_count_
                << ", fps: " << (right_count_ / time_elapsed);
      LOG(INFO) << "Imu count: " << imu_count_
                << ", hz: " << (imu_count_ / time_elapsed);

      // ROS messages could not be reliably printed here, using glog instead :(
      // ros::Duration(1).sleep();  // 1s
      // https://answers.ros.org/question/35163/how-to-perform-an-action-at-nodelet-unload-shutdown/
    }
  }

  void onInit() override {
    initDevice();
    NODELET_FATAL_COND(device_ == nullptr, "No MYNT EYE device selected :(");

    nh_ = getMTNodeHandle();
    private_nh_ = getMTPrivateNodeHandle();

    // node params

    std::string left_topic = "left";
    std::string right_topic = "right";
    std::string imu_topic = "imu";
    std::string temp_topic = "temp";
    private_nh_.getParam("left_topic", left_topic);
    private_nh_.getParam("right_topic", right_topic);
    private_nh_.getParam("imu_topic", imu_topic);
    private_nh_.getParam("temp_topic", temp_topic);

    base_frame_id_ = "camera_link";
    left_frame_id_ = "camera_left_frame";
    right_frame_id_ = "camera_right_frame";
    imu_frame_id_ = "camera_imu_frame";
    temp_frame_id_ = "camera_temp_frame";
    private_nh_.getParam("base_frame_id", base_frame_id_);
    private_nh_.getParam("left_frame_id", left_frame_id_);
    private_nh_.getParam("right_frame_id", right_frame_id_);
    private_nh_.getParam("imu_frame_id", imu_frame_id_);
    private_nh_.getParam("temp_frame_id", temp_frame_id_);

    gravity_ = 9.8;
    private_nh_.getParam("gravity", gravity_);

    // device options

    std::map<std::string, Option> option_map = {
        {"gain", Option::GAIN},
        {"brightness", Option::BRIGHTNESS},
        {"contrast", Option::CONTRAST},
        {"frame_rate", Option::FRAME_RATE},
        {"imu_frequency", Option::IMU_FREQUENCY},
        {"exposure_mode", Option::EXPOSURE_MODE},
        {"max_gain", Option::MAX_GAIN},
        {"max_exposure_time", Option::MAX_EXPOSURE_TIME},
        {"desired_brightness", Option::DESIRED_BRIGHTNESS},
        {"ir_control", Option::IR_CONTROL},
        {"hdr_mode", Option::HDR_MODE},
    };
    int value;
    for (auto &&it = option_map.begin(); it != option_map.end(); it++) {
      value = -1;
      private_nh_.getParam(it->first, value);
      if (value != -1) {
        NODELET_INFO_STREAM("Set " << it->first << " to " << value);
        device_->SetOptionValue(it->second, value);
      }
      NODELET_INFO_STREAM(
          it->second << ": " << device_->GetOptionValue(it->second));
    }

    // image publishers

    image_transport::ImageTransport it_mynteye(nh_);

    pub_left_ = it_mynteye.advertise(left_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << left_topic);
    pub_right_ = it_mynteye.advertise(right_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << right_topic);

    // imu publisher

    pub_imu_ = nh_.advertise<sensor_msgs::Imu>(imu_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << imu_topic);

    // temp publisher

    pub_temp_ = nh_.advertise<mynt_eye_ros_wrapper::Temp>(temp_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << temp_topic);

    publishStaticTransforms();
    publishTopics();
  }

  void publishStaticTransforms() {
    ros::Time tf_stamp = ros::Time::now();

    // The left frame is used as the base frame.
    geometry_msgs::TransformStamped b2l_msg;
    b2l_msg.header.stamp = tf_stamp;
    b2l_msg.header.frame_id = base_frame_id_;
    b2l_msg.child_frame_id = left_frame_id_;
    b2l_msg.transform.translation.x = 0;
    b2l_msg.transform.translation.y = 0;
    b2l_msg.transform.translation.z = 0;
    b2l_msg.transform.rotation.x = 0;
    b2l_msg.transform.rotation.y = 0;
    b2l_msg.transform.rotation.z = 0;
    b2l_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(b2l_msg);

    // Transform base frame to right frame
    geometry_msgs::TransformStamped b2r_msg;
    b2r_msg.header.stamp = tf_stamp;
    b2r_msg.header.frame_id = base_frame_id_;
    b2r_msg.child_frame_id = right_frame_id_;
    b2r_msg.transform.translation.x = 0;
    b2r_msg.transform.translation.y = 0;
    b2r_msg.transform.translation.z = 0;
    b2r_msg.transform.rotation.x = 0;
    b2r_msg.transform.rotation.y = 0;
    b2r_msg.transform.rotation.z = 0;
    b2r_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(b2r_msg);

    // Transform base frame to imu frame
    geometry_msgs::TransformStamped b2i_msg;
    b2i_msg.header.stamp = tf_stamp;
    b2i_msg.header.frame_id = base_frame_id_;
    b2i_msg.child_frame_id = imu_frame_id_;
    b2i_msg.transform.translation.x = 0;
    b2i_msg.transform.translation.y = 0;
    b2i_msg.transform.translation.z = 0;
    b2i_msg.transform.rotation.x = 0;
    b2i_msg.transform.rotation.y = 0;
    b2i_msg.transform.rotation.z = 0;
    b2i_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(b2i_msg);
  }

  void publishTopics() {
    device_->SetStreamCallback(
        Stream::LEFT, [this](const device::StreamData &data) {
          static double ros_time_beg = ros::Time::now().toSec();
          static double img_time_beg = data.img->timestamp;
          ros::Time stamp(
              ros_time_beg + (data.img->timestamp - img_time_beg) * 0.00001f);

          // static double img_time_prev = -1;
          // NODELET_INFO_STREAM("ros_time_beg: " << FULL_PRECISION <<
          // ros_time_beg
          //     << ", img_time_elapsed: " << FULL_PRECISION
          //     << ((data.img->timestamp - img_time_beg) * 0.00001f)
          //     << ", img_time_diff: " << FULL_PRECISION
          //     << ((img_time_prev < 0) ? 0
          //         : (data.img->timestamp - img_time_prev) * 0.01f) << " ms");
          // img_time_prev = data.img->timestamp;

          ++left_count_;
          publishLeft(data, left_count_, stamp);
          NODELET_DEBUG_STREAM(
              Stream::LEFT << ", count: " << left_count_
                           << ", frame_id: " << data.img->frame_id
                           << ", timestamp: " << data.img->timestamp
                           << ", exposure_time: " << data.img->exposure_time);
        });

    device_->SetStreamCallback(
        Stream::RIGHT, [this](const device::StreamData &data) {
          static double ros_time_beg = ros::Time::now().toSec();
          static double img_time_beg = data.img->timestamp;
          ros::Time stamp(
              ros_time_beg + (data.img->timestamp - img_time_beg) * 0.00001f);

          ++right_count_;
          publishRight(data, right_count_, stamp);
          NODELET_DEBUG_STREAM(
              Stream::RIGHT << ", count: " << right_count_
                            << ", frame_id: " << data.img->frame_id
                            << ", timestamp: " << data.img->timestamp
                            << ", exposure_time: " << data.img->exposure_time);
        });

    device_->SetMotionCallback([this](const device::MotionData &data) {
      static double ros_time_beg = ros::Time::now().toSec();
      static double imu_time_beg = data.imu->timestamp;
      ros::Time stamp(
          ros_time_beg + (data.imu->timestamp - imu_time_beg) * 0.00001f);

      // static double imu_time_prev = -1;
      // NODELET_INFO_STREAM("ros_time_beg: " << FULL_PRECISION << ros_time_beg
      //     << ", imu_time_elapsed: " << FULL_PRECISION
      //     << ((data.imu->timestamp - imu_time_beg) * 0.00001f)
      //     << ", imu_time_diff: " << FULL_PRECISION
      //     << ((imu_time_prev < 0) ? 0
      //         : (data.imu->timestamp - imu_time_prev) * 0.01f) << " ms");
      // imu_time_prev = data.imu->timestamp;

      ++imu_count_;
      publishImu(data, imu_count_, stamp);
      publishTemp(data.imu->temperature, imu_count_, stamp);
      NODELET_DEBUG_STREAM(
          "Imu count: " << imu_count_ << ", frame_id: " << data.imu->frame_id
                        << ", timestamp: " << data.imu->timestamp
                        << ", accel_x: " << data.imu->accel[0]
                        << ", accel_y: " << data.imu->accel[1]
                        << ", accel_z: " << data.imu->accel[2]
                        << ", gyro_x: " << data.imu->gyro[0]
                        << ", gyro_y: " << data.imu->gyro[1]
                        << ", gyro_z: " << data.imu->gyro[2]
                        << ", temperature: " << data.imu->temperature);
    });

    time_beg_ = ros::Time::now().toSec();
    device_->Start(Source::ALL);
  }

  void publishLeft(
      const device::StreamData &data, std::uint32_t seq, ros::Time stamp) {
    std_msgs::Header header;
    header.seq = seq;
    header.stamp = stamp;
    header.frame_id = left_frame_id_;
    cv::Mat img(
        data.frame->height(), data.frame->width(), CV_8UC1, data.frame->data());
    pub_left_.publish(cv_bridge::CvImage(header, enc::MONO8, img).toImageMsg());
  }

  void publishRight(
      const device::StreamData &data, std::uint32_t seq, ros::Time stamp) {
    std_msgs::Header header;
    header.seq = seq;
    header.stamp = stamp;
    header.frame_id = right_frame_id_;
    cv::Mat img(
        data.frame->height(), data.frame->width(), CV_8UC1, data.frame->data());
    pub_right_.publish(
        cv_bridge::CvImage(header, enc::MONO8, img).toImageMsg());
  }

  void publishImu(
      const device::MotionData &data, std::uint32_t seq, ros::Time stamp) {
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

  void publishTemp(float temperature, std::uint32_t seq, ros::Time stamp) {
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

    device_ = device;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  image_transport::Publisher pub_left_;
  image_transport::Publisher pub_right_;

  ros::Publisher pub_imu_;
  ros::Publisher pub_temp_;

  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  // node params

  std::string base_frame_id_;
  std::string left_frame_id_;
  std::string right_frame_id_;
  std::string imu_frame_id_;
  std::string temp_frame_id_;

  double gravity_;

  // device

  std::shared_ptr<Device> device_;

  double time_beg_ = -1;
  std::size_t left_count_ = 0;
  std::size_t right_count_ = 0;
  std::size_t imu_count_ = 0;
};

MYNTEYE_END_NAMESPACE

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(mynteye::ROSWrapperNodelet, nodelet::Nodelet);
