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
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <image_transport/image_transport.h>
#include <mynteye_image_pipeline/depth_conversions.h>

namespace mynteye_image_pipeline {
class PointClouldXyz : public nodelet::Nodelet{
 public:
  PointClouldXyz():_points_frame_id("points_frame_id") {}
  virtual ~PointClouldXyz() {}
  virtual void onInit() {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& pnh = getPrivateNodeHandle();
    std::string sub_depth_topic = "sub_depth_topic";
    std::string sub_left_camerainfo = "sub_depth_topic";
    std::string pub_pointcloud_topic = "pub_pointcloud_topic";
    bool approx_sync = false;
    int square_size = 5;
    pnh.param("sub_depth_topic", sub_depth_topic, sub_depth_topic);
    pnh.param("sub_left_camerainfo",
      sub_left_camerainfo, sub_left_camerainfo);
    pnh.param("approx_sync", approx_sync, approx_sync);
    pnh.param("square_size", square_size, square_size);
    pnh.param("points_frame_id", _points_frame_id, _points_frame_id);
    pnh.param("pub_pointcloud_topic",
      pub_pointcloud_topic, pub_pointcloud_topic);

    {
      NODELET_INFO_STREAM("sub_depth_topic: " << sub_depth_topic);
      NODELET_INFO_STREAM("sub_left_camerainfo: "<< sub_left_camerainfo);
      NODELET_INFO_STREAM("pointcloud: approx_sync: " << approx_sync);
      NODELET_INFO_STREAM("pointcloud: square_size: "<< square_size);
      NODELET_INFO_STREAM("pub_pointcloud_topic: "<< pub_pointcloud_topic);
    }

    image_transport::ImageTransport it(nh);
    _sub_left_depth_img.subscribe(it, sub_depth_topic, 1);
    _sub_left_pinhole_camerainfo.subscribe(nh, sub_left_camerainfo, 1);
    _pub_point_cloud = nh.advertise<PointCloud>(pub_pointcloud_topic, 1);

    if (approx_sync) {
      _approxSyncPinhole.reset(new ApproxSyncPinhole(
          ApproxPolicyPinhole(square_size),
          _sub_left_depth_img,
          _sub_left_pinhole_camerainfo));
      _approxSyncPinhole->registerCallback(
        boost::bind(&PointClouldXyz::process, this, _1, _2));
    } else {
    _exactTimeSyncPinhole.reset(new ExactTimeSyncPinhole(
      ExactPolicyPinhole(square_size),
      _sub_left_depth_img,
      _sub_left_pinhole_camerainfo));
    _exactTimeSyncPinhole->registerCallback(
      boost::bind(&PointClouldXyz::process, this, _1, _2));
    }
  }

 private:
  void process(const sensor_msgs::ImageConstPtr& depth_img_ptr,
                const sensor_msgs::CameraInfoConstPtr& camera_info_ptr) {
    PointCloud::Ptr cloud_msg(new PointCloud());
    cloud_msg->header = depth_img_ptr->header;
    cloud_msg->header.frame_id = _points_frame_id;
    cloud_msg->height = depth_img_ptr->height;
    cloud_msg->width = depth_img_ptr->width;
    cloud_msg->is_dense = false;
    cloud_msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    _stereo_model.fromCameraInfo(camera_info_ptr);

    if (depth_img_ptr->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
      convert<uint16_t>(depth_img_ptr, cloud_msg,
                        _stereo_model.fx(), _stereo_model.fy(),
                        _stereo_model.cx(), _stereo_model.cy());
    } else if (depth_img_ptr->encoding ==
                sensor_msgs::image_encodings::TYPE_32FC1) {
      convert<float>(depth_img_ptr, cloud_msg,
                      _stereo_model.fx(), _stereo_model.fy(),
                      _stereo_model.cx(), _stereo_model.cy());
    } else {
      NODELET_ERROR_STREAM("Depth image is not support type.");
    }

    _pub_point_cloud.publish(cloud_msg);
  }

 private:
  typedef image_transport::SubscriberFilter ImageSubscriber;
  typedef sensor_msgs::PointCloud2 PointCloud;
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::CameraInfo> ApproxPolicyPinhole;
  typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::Image, sensor_msgs::CameraInfo> ExactPolicyPinhole;
  typedef message_filters::Synchronizer<
    ApproxPolicyPinhole> ApproxSyncPinhole;
  typedef message_filters::Synchronizer<
    ExactPolicyPinhole> ExactTimeSyncPinhole;
  typedef message_filters::Subscriber<
    sensor_msgs::CameraInfo> CameraInfoPinholeSubscriber;

    boost::shared_ptr<ApproxSyncPinhole> _approxSyncPinhole;
    boost::shared_ptr<ExactTimeSyncPinhole> _exactTimeSyncPinhole;
    // subscribe depth and camerainfo based on left image
    ImageSubscriber _sub_left_depth_img;
    CameraInfoPinholeSubscriber _sub_left_pinhole_camerainfo;
    image_geometry::PinholeCameraModel _stereo_model;
    // publish pointcloud
    ros::Publisher _pub_point_cloud;
    std::string _points_frame_id;
};
  PLUGINLIB_EXPORT_CLASS(
    mynteye_image_pipeline::PointClouldXyz, nodelet::Nodelet);
}  // namespace mynteye_image_pipeline
