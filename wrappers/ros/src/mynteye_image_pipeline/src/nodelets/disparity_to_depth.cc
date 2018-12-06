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
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace mynteye_image_pipeline {
class DisparityToDepth : public nodelet::Nodelet {
 public:
  DisparityToDepth():use_filer(false), ksize(5),
    _depth_frame_id("depth_frame_id") {}

  virtual ~DisparityToDepth() {}

  virtual void onInit() {
    ros::NodeHandle & nh = getNodeHandle();
    ros::NodeHandle & pnh = getPrivateNodeHandle();

    // load params
    std::string sub_left_rect_camerainfo_topic =
      "sub_left_rect_camerainfo_topic";
    std::string sub_disparity_topic = "sub_disparity_topic";
    std::string pub_depth_topic = "depth/data_raw";
    std::string pub_depth16_topic = "pub_depth16_topic";
    std::string pub_depth_camera_info_topic = "pub_depth_camera_info_topic";
    int square_size = 5;
    bool approx_sync = false;
    pnh.param("use_filer", use_filer, use_filer);
    pnh.param("ksize", ksize, ksize);
    pnh.param("square_size", square_size, square_size);
    pnh.param("approx_sync", approx_sync, approx_sync);
    pnh.param("sub_left_rect_camerainfo_topic",
        sub_left_rect_camerainfo_topic, sub_left_rect_camerainfo_topic);
    pnh.param("sub_disparity_topic", sub_disparity_topic, sub_disparity_topic);
    pnh.param("pub_depth_topic", pub_depth_topic, pub_depth_topic);
    pnh.param("pub_depth_camera_info_topic",
        pub_depth_camera_info_topic, pub_depth_camera_info_topic);
    pnh.param("delay_time", delay_time, delay_time);
    pnh.param("depth_frame_id", _depth_frame_id, _depth_frame_id);
    pnh.param("pub_depth16_topic", pub_depth16_topic, pub_depth16_topic);

    {
      NODELET_INFO_STREAM("depth: use_filer: " << use_filer);
      NODELET_INFO_STREAM("depth: ksize: " << ksize);
      NODELET_INFO_STREAM("depth: square_size: " << square_size);
      NODELET_INFO_STREAM("depth: approx_sync: " << approx_sync);
      NODELET_INFO_STREAM("depth: sub_left_rect_camerainfo_topic: "
        << sub_left_rect_camerainfo_topic);
      NODELET_INFO_STREAM("depth: sub_disparity_topic: "
        << sub_disparity_topic);
      NODELET_INFO_STREAM("depth: pub_depth_topic: " << pub_depth_topic);
      NODELET_INFO_STREAM("depth: pub_depth_camera_info_topic: "
        << pub_depth_camera_info_topic);
      NODELET_INFO_STREAM("depth: depth_frame_id: " << _depth_frame_id);
      NODELET_INFO_STREAM("depth: pub_depth16_topic: " << pub_depth16_topic);
      NODELET_INFO_STREAM("depth: delay_time: " << delay_time);
    }

    image_transport::ImageTransport it(nh);
    pub32f_ = it.advertise(pub_depth_topic, 1);
    pub16u_ = it.advertise(pub_depth16_topic, 1);
    pub_camera_info.reset(new ros::Publisher(
      nh.advertise<sensor_msgs::CameraInfo>(pub_depth_camera_info_topic, 1)));

    sub_camera_info.subscribe(nh, sub_left_rect_camerainfo_topic, 1);
    sub_disparity_img.subscribe(nh, sub_disparity_topic, 1);

  if (approx_sync) {
    _approxSync.reset(new StereoApproSync(StereoApproximatePolicy(square_size),
      sub_disparity_img, sub_camera_info));
    _approxSync->registerCallback(
      boost::bind(&DisparityToDepth::callback, this, _1, _2));
  } else {
    _exactTimeSync.reset(new StereoExactSync(StereoExactPolicy(square_size),
       sub_disparity_img, sub_camera_info));
    _exactTimeSync->registerCallback(
       boost::bind(&DisparityToDepth::callback, this, _1, _2));
  }
}

 private:
  void callback(const stereo_msgs::DisparityImageConstPtr& disparityMsg,
    const sensor_msgs::CameraInfoConstPtr& camera_info_ptr) {
    if (disparityMsg->image.encoding.compare(
      sensor_msgs::image_encodings::TYPE_32FC1) !=0) {
      NODELET_ERROR("Input type must be disparity=32FC1");
      return;
    }

    bool publish32f = pub32f_.getNumSubscribers();
    bool publish16u = pub16u_.getNumSubscribers();
    bool pub_cam_flag = pub_camera_info->getNumSubscribers();

    if (publish32f || publish16u) {
          // sensor_msgs::image_encodings::TYPE_32FC1
      cv::Mat disparity(disparityMsg->image.height, disparityMsg->image.width,
        CV_32FC1, const_cast<uchar*>(disparityMsg->image.data.data()));
      cv::Mat depth32f;
      cv::Mat depth16u;
      if (publish32f) {
        depth32f = cv::Mat::zeros(disparity.rows, disparity.cols, CV_32F);
      }
      if (publish16u) {
        depth16u = cv::Mat::zeros(disparity.rows, disparity.cols, CV_16U);
      }
      for (int i = 0; i < disparity.rows; i++) {
        for (int j = 0; j < disparity.cols; j++) {
          float disparity_value = disparity.at<float>(i, j);
          if (disparity_value > disparityMsg->min_disparity &&
            disparity_value < disparityMsg->max_disparity) {
            // baseline * focal / disparity
            float depth = disparityMsg->T * disparityMsg->f / disparity_value;
            if (publish32f) {
              depth32f.at<float>(i, j) = depth;
            }
            if (publish16u) {
              depth16u.at<unsigned short>(i, j) =
                static_cast<unsigned short>(depth * 1000.0f);
            }
          }
        }
      }

      if (publish32f) {
        // convert to ROS sensor_msg::Image
        std_msgs::Header header;
        header.seq = disparityMsg->header.seq;
        header.stamp = disparityMsg->header.stamp +
          ros::Duration(delay_time*0.001);
        header.frame_id = _depth_frame_id;
        cv_bridge::CvImage cvDepth(
          header, sensor_msgs::image_encodings::TYPE_32FC1, depth32f);
        if (use_filer) {
          cv::medianBlur(depth32f, depth32f, ksize);
        }
        sensor_msgs::Image depthMsg;
        cvDepth.toImageMsg(depthMsg);
        // publish the message
        pub32f_.publish(depthMsg);

        // pub_camera_info->publish(camera_info_ptr);
      }

      if (publish16u) {
        // convert to ROS sensor_msg::Image
        std_msgs::Header header;
        header.seq = disparityMsg->header.seq;
        header.stamp = disparityMsg->header.stamp +
          ros::Duration(delay_time*0.001);
        header.frame_id = "depth_frame_id";
        cv_bridge::CvImage cvDepth(
          header, sensor_msgs::image_encodings::TYPE_16UC1, depth16u);
        if (use_filer) {
          cv::medianBlur(depth16u, depth16u, ksize);
        }
        sensor_msgs::Image depthMsg;
        cvDepth.toImageMsg(depthMsg);
        // publish the message
        pub16u_.publish(depthMsg);
      }
    }

    if (pub_cam_flag) {
      sensor_msgs::CameraInfoPtr l_cam_info_ptr =
        boost::make_shared<sensor_msgs::CameraInfo>(*camera_info_ptr);
      l_cam_info_ptr->header.seq = camera_info_ptr->header.seq;
      l_cam_info_ptr->header.stamp =
        camera_info_ptr->header.stamp + ros::Duration(delay_time * 0.001);
      l_cam_info_ptr->header.frame_id = _depth_frame_id;
      pub_camera_info->publish(l_cam_info_ptr);
    }
  }

 private:
  typedef message_filters::Subscriber<sensor_msgs::CameraInfo>
    CameraInfoSubscriber;
  typedef message_filters::Subscriber<stereo_msgs::DisparityImage>
    DisparitySubscriber;
  typedef message_filters::sync_policies::ApproximateTime<
    stereo_msgs::DisparityImage,
    sensor_msgs::CameraInfo> StereoApproximatePolicy;
  typedef message_filters::sync_policies::ExactTime<
    stereo_msgs::DisparityImage, sensor_msgs::CameraInfo> StereoExactPolicy;
  typedef message_filters::Synchronizer<
    StereoApproximatePolicy> StereoApproSync;
  typedef message_filters::Synchronizer<StereoExactPolicy> StereoExactSync;

  boost::shared_ptr<StereoApproSync> _approxSync;
  boost::shared_ptr<StereoExactSync> _exactTimeSync;

  image_transport::Publisher pub32f_;
  image_transport::Publisher pub16u_;
  boost::shared_ptr<ros::Publisher> pub_camera_info;

  CameraInfoSubscriber sub_camera_info;
  DisparitySubscriber  sub_disparity_img;
  bool use_filer;
  int ksize;
  std::string _depth_frame_id;
  double delay_time;
};

PLUGINLIB_EXPORT_CLASS(mynteye_image_pipeline::DisparityToDepth,
  nodelet::Nodelet);
}  // namespace mynteye_image_pipeline
