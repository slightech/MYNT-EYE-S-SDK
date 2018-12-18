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
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <camodocal/camera_models/Camera.h>
#include <camodocal/camera_models/CameraFactory.h>
#include <camodocal/camera_models/CataCamera.h>
#include <camodocal/camera_models/EquidistantCamera.h>
#include <camodocal/camera_models/PinholeCamera.h>
#include <camodocal/gpl/gpl.h>
#include <mynteye_image_pipeline/tools/tools.h>
#include <mynteye_image_pipeline/camera_manager.h>
#include <camodocal/camera_models/Camera.h>
namespace mynteye_image_pipeline {
class Rectify : public nodelet::Nodelet {
 public:
  Rectify() : _square_size(5), _approx_sync(false),
    _left_rect_frame_id("left_rect_frame_id"),
    _right_rect_frame_id("right_rect_frame_id"),
    _project_model("pinhole") {}
  virtual ~Rectify() {}
  virtual void onInit() {
    ros::NodeHandle &pnh = getPrivateNodeHandle();
    ros::NodeHandle &nh = getNodeHandle();
    std::string left_topic = "left_topic";
    std::string right_topic = "right_topic";
    std::string left_rect_topic = "left_rect_topic";
    std::string right_rect_topic = "right_rect_topic";
    std::string left_camera_info = "left_camera_info";
    std::string right_camera_info = "right_camera_info";
    std::string left_rect_camerainfo_topic = "left_rect_camerainfo_topic";
    std::string right_rect_camerainfo_topic = "right_rect_camerainfo_topic";
    pnh.param("square_size", _square_size, _square_size);
    pnh.param("approx_sync", _approx_sync, _approx_sync);
    pnh.param("left_topic", left_topic, left_topic);
    pnh.param("right_topic", right_topic, right_topic);
    pnh.param("left_rect_topic", left_rect_topic, left_rect_topic);
    pnh.param("right_rect_topic", right_rect_topic, right_rect_topic);
    pnh.param("left_camera_info", left_camera_info, left_camera_info);
    pnh.param("right_camera_info", right_camera_info, right_camera_info);
    pnh.param("right_rect_camerainfo_topic",
              right_rect_camerainfo_topic, right_rect_camerainfo_topic);
    pnh.param("project_model", _project_model, _project_model);
    pnh.param("left_rect_camerainfo_topic",
              left_rect_camerainfo_topic, left_rect_camerainfo_topic);
    pnh.param("left_rect_frame_id", _left_rect_frame_id, _left_rect_frame_id);
    pnh.param("right_rect_frame_id",
              _right_rect_frame_id, _right_rect_frame_id);

    {
      NODELET_INFO_STREAM("Rectify: _square_size: " << _square_size);
      NODELET_INFO_STREAM("Rectify: _approx_sync: " << _approx_sync);
      NODELET_INFO_STREAM("Rectify: left_topic: " << left_topic);
      NODELET_INFO_STREAM("Rectify: right_topic: " << right_topic);
      NODELET_INFO_STREAM("Rectify: left_rect_topic: " << left_rect_topic);
      NODELET_INFO_STREAM("Rectify: right_rect_topic: " << right_rect_topic);
      NODELET_INFO_STREAM("Rectify: left_camera_info: " << left_camera_info);
      NODELET_INFO_STREAM("Rectify: right_camera_info: " << right_camera_info);
      NODELET_INFO_STREAM("Rectify: left_rect_camerainfo_topic: "
                            << left_rect_camerainfo_topic);
      NODELET_INFO_STREAM("Rectify: right_rect_camerainfo_topic: "
                            << right_rect_camerainfo_topic);
      NODELET_INFO_STREAM("Rectify: left_rect_frame_id: "
                            << _left_rect_frame_id);
      NODELET_INFO_STREAM("Rectify: right_rect_frame_id: "
                            << _right_rect_frame_id);
      NODELET_INFO_STREAM("Rectify: project_model: " << _project_model);
    }

    image_transport::ImageTransport it(nh);
    _img_left_sub.subscribe(it, left_topic, 1);
    _img_right_sub.subscribe(it, right_topic, 1);
    _camerainfo_left_sub.subscribe(nh, left_camera_info, 1);
    _camerainfo_right_sub.subscribe(nh, right_camera_info, 1);

    _rectify_left_pub.reset(
      new image_transport::Publisher(
        it.advertise(left_rect_topic, 1)));
    _rectify_right_pub.reset(
      new image_transport::Publisher(
        it.advertise(right_rect_topic, 1)));
    _rectify_left_camerainfo_pub.reset(
      new ros::Publisher(nh.advertise<sensor_msgs::CameraInfo>(
        left_rect_camerainfo_topic, 1)));
    _rectify_right_camerainfo_pub.reset(
      new ros::Publisher(nh.advertise<sensor_msgs::CameraInfo>(
        right_rect_camerainfo_topic, 1)));

    if (_approx_sync) {
      _approximate_sync.reset(
          new StereoApproSync(StereoApproximatePolicy(
            _square_size), _img_left_sub, _camerainfo_left_sub,
            _img_right_sub, _camerainfo_right_sub));
      _approximate_sync->registerCallback(
        boost::bind(&Rectify::process, this, _1, _2, _3, _4));
    } else {
      _exact_sync.reset(
          new StereoExactSync(
            StereoExactPolicy(
              _square_size), _img_left_sub,
              _camerainfo_left_sub, _img_right_sub,
              _camerainfo_right_sub));
      _exact_sync->registerCallback(
        boost::bind(&Rectify::process, this, _1, _2, _3, _4));
    }
  }

  void process(
      const sensor_msgs::ImageConstPtr &img_left_ptr,
      const sensor_msgs::CameraInfoConstPtr &left_camera_info,
      const sensor_msgs::ImageConstPtr &img_right_ptr,
      const sensor_msgs::CameraInfoConstPtr &right_camera_info) {
      ROS_ASSERT(
        img_left_ptr->encoding.compare(
          sensor_msgs::image_encodings::MONO8) == 0 &&
        img_right_ptr->encoding.compare(
          sensor_msgs::image_encodings::MONO8) == 0);
      cv::Mat left_img = cv_bridge::toCvShare(img_left_ptr, "8UC1")->image;
      cv::Mat right_img = cv_bridge::toCvShare(img_right_ptr, "8UC1")->image;
      if (_camera_odo_ptr_left.get() == 0 || _camera_odo_ptr_right.get() == 0) {
        if (_camera_odo_ptr_left.get() == 0) {
          _camera_odo_ptr_left =
            CameraManagerTools::getCamOdoCameraPtr(left_camera_info);
        }
        if (_camera_odo_ptr_right.get() == 0) {
          _camera_odo_ptr_right =
            CameraManagerTools::getCamOdoCameraPtr(right_camera_info);
        }

      cv::Mat rect_R_l = cv::Mat::eye(3, 3, CV_32F),
                rect_R_r = cv::Mat::eye(3, 3, CV_32F);
      for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
          rect_R_l.at<float>(i, j) = left_camera_info->R[i*3+j];
          rect_R_r.at<float>(i, j) = right_camera_info->R[i*3+j];
        }
      }

      double left_f[] = {left_camera_info->P[0], left_camera_info->P[5]};
      double left_center[] = {left_camera_info->P[2], left_camera_info->P[6]};
      double right_f[] = {right_camera_info->P[0], right_camera_info->P[5]};
      double right_center[] = {right_camera_info->P[2],
                                right_camera_info->P[6]};
      _camera_odo_ptr_left->initUndistortRectifyMap(
        _mapx_l, _mapy_l, left_f[0], left_f[1], cv::Size(0, 0),
        left_center[0], left_center[1], rect_R_l);
      _camera_odo_ptr_right->initUndistortRectifyMap(
        _mapx_r, _mapy_r, right_f[0], right_f[1], cv::Size(0, 0),
        right_center[0], right_center[1], rect_R_r);
      }

    cv::remap(
        left_img, left_img, _mapx_l, _mapy_l,
        cv::INTER_LINEAR);
    cv::remap(
        right_img, right_img, _mapx_r, _mapy_r,
        cv::INTER_LINEAR);

    auto &&left_msg = cv_bridge::CvImage(
      setHeader(img_left_ptr->header, _left_rect_frame_id),
      sensor_msgs::image_encodings::MONO8, left_img);
    auto &&right_msg = cv_bridge::CvImage(
      setHeader(img_right_ptr->header, _right_rect_frame_id),
      sensor_msgs::image_encodings::MONO8, right_img);

    _rectify_left_pub->publish(left_msg.toImageMsg());
    _rectify_right_pub->publish(right_msg.toImageMsg());

    _rectify_left_camerainfo_pub->publish(
      setDistortionToZereo(left_camera_info, _left_rect_frame_id));
    _rectify_right_camerainfo_pub->publish(
      setDistortionToZereo(right_camera_info, _right_rect_frame_id));
  }

 private:
  sensor_msgs::CameraInfoPtr setDistortionToZereo(
    const sensor_msgs::CameraInfoConstPtr& camera_info_ptr,
    const std::string& frame_id) {
    sensor_msgs::CameraInfoPtr target_camera_info_ptr =
      boost::make_shared<sensor_msgs::CameraInfo>(*camera_info_ptr);
    for (auto& elem : target_camera_info_ptr->D)
      elem = 0;
    target_camera_info_ptr->header.frame_id  = frame_id;
    if (_project_model.compare("pinhole") == 0) {
      target_camera_info_ptr->distortion_model =
        sensor_msgs::distortion_models::PLUMB_BOB;
    } else {
      ROS_ERROR("it is not support.");
    }
    return target_camera_info_ptr;
  }

  std_msgs::Header setHeader(
    const std_msgs::Header& header,
    const std::string& frame_id) {
    std_msgs::Header local_header;
    local_header.seq = header.seq;
    local_header.stamp = header.stamp;
    local_header.frame_id = frame_id;
    return local_header;
  }

 private:
  typedef image_transport::SubscriberFilter ImageSubscriber;
  typedef message_filters::Subscriber<sensor_msgs::CameraInfo>
    CameraInfoSubscriber;
  typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::Image, sensor_msgs::CameraInfo,
            sensor_msgs::Image, sensor_msgs::CameraInfo>
    StereoApproximatePolicy;
  typedef message_filters::sync_policies::ExactTime<
            sensor_msgs::Image, sensor_msgs::CameraInfo,
            sensor_msgs::Image, sensor_msgs::CameraInfo>
    StereoExactPolicy;
  typedef message_filters::Synchronizer<StereoApproximatePolicy>
      StereoApproSync;
  typedef message_filters::Synchronizer<StereoExactPolicy> StereoExactSync;

  ImageSubscriber _img_left_sub;
  ImageSubscriber _img_right_sub;
  CameraInfoSubscriber _camerainfo_left_sub;
  CameraInfoSubscriber _camerainfo_right_sub;

  boost::shared_ptr<StereoApproSync> _approximate_sync;
  boost::shared_ptr<StereoExactSync> _exact_sync;
  // publish image and camerainfo
  boost::shared_ptr<image_transport::Publisher> _rectify_left_pub;
  boost::shared_ptr<image_transport::Publisher> _rectify_right_pub;
  boost::shared_ptr<ros::Publisher> _rectify_left_camerainfo_pub;
  boost::shared_ptr<ros::Publisher> _rectify_right_camerainfo_pub;
  int _square_size;
  bool _approx_sync;
  std::string _left_rect_frame_id;
  std::string _right_rect_frame_id;
  std::string _project_model;

  camodocal::CameraPtr _camera_odo_ptr_left;
  camodocal::CameraPtr _camera_odo_ptr_right;
  cv::Mat _mapx_l, _mapy_l, _mapx_r, _mapy_r;
};
PLUGINLIB_EXPORT_CLASS(mynteye_image_pipeline::Rectify, nodelet::Nodelet);
}  // namespace mynteye_image_pipeline
