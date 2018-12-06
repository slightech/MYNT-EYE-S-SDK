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
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_geometry/stereo_camera_model.h>
#include <nodelet/nodelet.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/cv_bridge.h>
#include <map>

namespace mynteye_image_pipeline {
class Disparity : public nodelet::Nodelet{
 public:
    Disparity(): _square_size(5), _approx_sync(false),
      _prefilter_size(9), _prefilter_cap(31),
      _correlation_window_size(15),
      _min_disparity(0), _disparity_range(64),
      _uniqueness_ratio(15), _texture_threshold(10),
      _speckle_size(100), _speckle_range(4),
      _bm_matcher(cv::StereoBM::create()),
      _disp12MaxDiff(1), _sgbmWindsize(3),
      _disparity_image_alg(true),
      _sgbm_matcher(cv::StereoSGBM::create(0, 16, 3)) {
    }
    virtual ~Disparity() {}
    virtual void onInit() {
      ros::NodeHandle &pnh = getPrivateNodeHandle();
      ros::NodeHandle &nh = getNodeHandle();
      std::string sub_left_rect_topic = "sub_left_rect_topic";
      std::string sub_right_rect_topic = "sub_right_rect_topic";
      std::string sub_left_rect_camerainfo_topic =
          "sub_left_rect_camerainfo_topic";
      std::string sub_right_rect_camerainfo_topic =
          "sub_right_rect_camerainfo_topic";
      std::string disparity = "disparity";


      pnh.param("sub_left_rect_topic", sub_left_rect_topic,
                sub_left_rect_topic);
      pnh.param("sub_right_rect_topic", sub_right_rect_topic,
                sub_right_rect_topic);
      pnh.param("sub_right_rect_camerainfo_topic",
                sub_right_rect_camerainfo_topic, sub_right_rect_topic);
      pnh.param("sub_left_rect_camerainfo_topic",
                sub_left_rect_camerainfo_topic, sub_left_rect_camerainfo_topic);
      pnh.param("disparity", disparity, disparity);
      pnh.param("square_size", _square_size, _square_size);
      pnh.param("approx_sync", _approx_sync, _approx_sync);

      // read BM parameters

      pnh.param("prefilter_size", _prefilter_size, _prefilter_size);
      pnh.param("prefilter_cap", _prefilter_cap, _prefilter_cap);
      pnh.param("correlation_window_size", _correlation_window_size,
                _correlation_window_size);
      pnh.param("min_disparity", _min_disparity, _min_disparity);
      pnh.param("disparity_range", _disparity_range, _disparity_range);
      pnh.param("uniqueness_ratio", _uniqueness_ratio, _uniqueness_ratio);
      pnh.param("texture_threshold", _texture_threshold, _texture_threshold);
      pnh.param("speckle_size", _speckle_size, _speckle_size);
      pnh.param("speckle_range", _speckle_range, _speckle_range);

      pnh.param("use_bm", _disparity_image_alg, _disparity_image_alg);
      // SGBM paramters
      pnh.param("disp12MaxDiff", _disp12MaxDiff, _disp12MaxDiff);
      pnh.param("sgbmWindsize", _sgbmWindsize, _sgbmWindsize);


      // debug
      {
        NODELET_INFO_STREAM("sub_left_rect_topic: " << sub_left_rect_topic);
        NODELET_INFO_STREAM("sub_right_rect_topic: " << sub_right_rect_topic);
        NODELET_INFO_STREAM("sub_left_rect_camerainfo_topic: " <<
                            sub_left_rect_camerainfo_topic);
        NODELET_INFO_STREAM("sub_right_rect_camerainfo_topic: " <<
                            sub_right_rect_camerainfo_topic);
        NODELET_INFO_STREAM("disparity: "<< disparity);
        NODELET_INFO_STREAM("disparity: _square_size: "<< _square_size);
        NODELET_INFO_STREAM("disparity: approx_sync: "<< _approx_sync);

        // BM parameters
        NODELET_INFO_STREAM("prefilter_size: " << _prefilter_size);
        NODELET_INFO_STREAM("prefilter_cap: " << _prefilter_cap);
        NODELET_INFO_STREAM("correlation_window_size: " <<
                            _correlation_window_size);
        NODELET_INFO_STREAM("min_disparity: " << _min_disparity);
        NODELET_INFO_STREAM("disparity_range: " << _disparity_range);
        NODELET_INFO_STREAM("uniqueness_ratio: " << _uniqueness_ratio);
        NODELET_INFO_STREAM("texture_threshold: " << _texture_threshold);
        NODELET_INFO_STREAM("speckle_size: " << _speckle_size);
        NODELET_INFO_STREAM("speckle_range: " << _speckle_range);
        // SGBM paramters
        NODELET_INFO_STREAM("_sgbmWindsize: " << _sgbmWindsize);
        NODELET_INFO_STREAM("_disp12MaxDiff: " << _disp12MaxDiff);
        NODELET_INFO_STREAM("_disparity_image_alg: " << _disparity_image_alg);
      }

      image_transport::ImageTransport it(nh);
      _sub_left_image.subscribe(it, sub_left_rect_topic, 1);
      _sub_right_image.subscribe(it, sub_right_rect_topic, 1);
      _pub_disparity = nh.advertise<stereo_msgs::DisparityImage>(disparity, 1);

      _sub_left_pinhole_camerainfo.subscribe(
          nh, sub_left_rect_camerainfo_topic, 1);
      _sub_right_pinhole_camerainfo.subscribe(
          nh, sub_right_rect_camerainfo_topic, 1);
      if (_approx_sync) {
        _approxSyncPinhole.reset(new ApproxSyncPinhole(ApproxPolicyPinhole(
          _square_size), _sub_left_image,
          _sub_left_pinhole_camerainfo,
          _sub_right_image, _sub_right_pinhole_camerainfo));
        _approxSyncPinhole->registerCallback(
          boost::bind(&Disparity::processPinhole, this, _1, _2, _3, _4));
      } else {
        _exactTimeSyncPinhole.reset(new ExactTimeSyncPinhole(ExactPolicyPinhole(
          _square_size), _sub_left_image,
          _sub_left_pinhole_camerainfo,
          _sub_right_image, _sub_right_pinhole_camerainfo));
        _exactTimeSyncPinhole->registerCallback(
          boost::bind(&Disparity::processPinhole, this, _1, _2, _3, _4));
      }
    }
 private :
    void processPinhole(const sensor_msgs::ImageConstPtr& sub_left_img,
          const sensor_msgs::CameraInfoConstPtr& sub_left_camera_info,
          const sensor_msgs::ImageConstPtr& sub_right_img,
          const sensor_msgs::CameraInfoConstPtr& sub_right_camera_info) {
      // compute window of (patentially) valid disparities
      int border = 15 / 2;
      int left = 64 + 0 + border - 1;
      int wtf = border + 0;
      if (_disparity_image_alg) {
        setBMParameters();
        border = _bm_matcher->getBlockSize() / 2;
        left = _bm_matcher->getNumDisparities() +
               _bm_matcher->getMinDisparity() + border -1;
        wtf = (_bm_matcher->getMinDisparity() >= 0) ?
              border + _bm_matcher->getMinDisparity() :
              std::max(border, -_bm_matcher->getMinDisparity());
      } else {
        setSGBMParameters();
        border = _sgbm_matcher->getBlockSize() / 2;
        left = _sgbm_matcher->getNumDisparities() +
              _sgbm_matcher->getMinDisparity() + border -1;
        wtf = (_sgbm_matcher->getMinDisparity() >= 0) ?
              border + _sgbm_matcher->getMinDisparity() :
              std::max(border, -_sgbm_matcher->getMinDisparity());
      }
      _stereo_pin_model.fromCameraInfo(
        sub_left_camera_info, sub_right_camera_info);
      stereo_msgs::DisparityImagePtr disparity_msg =
        boost::make_shared<stereo_msgs::DisparityImage>();
      disparity_msg->header = sub_left_camera_info->header;
      disparity_msg->image.header = sub_left_img->header;

      int right = disparity_msg->image.width - 1 -wtf;
      int top = border;
      int bottom = disparity_msg->image.height - 1 - border;
      disparity_msg->valid_window.x_offset = left;
      disparity_msg->valid_window.y_offset = top;
      disparity_msg->valid_window.width = right - left;
      disparity_msg->valid_window.height = bottom - top;

      const cv::Mat_<uint8_t> left_img = cv_bridge::toCvShare(
        sub_left_img, sensor_msgs::image_encodings::MONO8)->image;
      const cv::Mat_<uint8_t> right_img = cv_bridge::toCvShare(
        sub_right_img, sensor_msgs::image_encodings::MONO8)->image;

      processDisparity(left_img, right_img, *disparity_msg);

      double cx_left = _stereo_pin_model.left().cx();
      double cx_right = _stereo_pin_model.right().cx();

      if (cx_left != cx_right) {
        cv::Mat_<float> disp_img(disparity_msg->image.height,
          disparity_msg->image.width,
          reinterpret_cast<float*>(&disparity_msg->image.data[0]),
          disparity_msg->image.step);
        cv::subtract(disp_img, cv::Scalar(cx_left-cx_right), disp_img);
      }
      _pub_disparity.publish(disparity_msg);
    }

    void processDisparity(
      const cv::Mat& left_img,
      const cv::Mat& right_img,
      stereo_msgs::DisparityImage& disparity) {
      static const int DISPARITY_PER_PIXEL = 16;
      static const double INV_DISPARITY_PER_PIXEL = 1.0/DISPARITY_PER_PIXEL;
      cv::Mat_<int16_t> disparity16;

      if (_disparity_image_alg) {
        _bm_matcher->compute(left_img, right_img, disparity16);
        disparity.min_disparity = _bm_matcher->getMinDisparity();
        disparity.max_disparity = _bm_matcher->getMinDisparity() +
          _bm_matcher->getNumDisparities() - 1;
      } else {
        _sgbm_matcher->compute(left_img, right_img, disparity16);
        disparity.min_disparity = _sgbm_matcher->getMinDisparity();
        disparity.max_disparity = _sgbm_matcher->getMinDisparity() +
        _sgbm_matcher->getNumDisparities() - 1;
      }

      sensor_msgs::Image& dimage = disparity.image;
      dimage.height = disparity16.rows;
      dimage.width = disparity16.cols;
      dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      dimage.step = dimage.width * sizeof(float);
      dimage.data.resize(dimage.step * dimage.height);
      cv::Mat_<float> dmat(dimage.height, dimage.width,
        (float*)&dimage.data[0], dimage.step);
      disparity16.convertTo(dmat, dmat.type(),
        INV_DISPARITY_PER_PIXEL,
        -(_stereo_pin_model.left().cx() -
        _stereo_pin_model.right().cx()));
      ROS_ASSERT(dmat.data &dimage.data[0]);

      disparity.f = _stereo_pin_model.right().fx();
      disparity.T = _stereo_pin_model.baseline();
      disparity.delta_d = INV_DISPARITY_PER_PIXEL;
    }

    void setBMParameters() {
     _bm_matcher->setPreFilterSize(_prefilter_size);
     _bm_matcher->setPreFilterCap(_prefilter_cap);
     _bm_matcher->setBlockSize(_correlation_window_size);
     _bm_matcher->setMinDisparity(_min_disparity);
     _bm_matcher->setNumDisparities(_disparity_range);
     _bm_matcher->setUniquenessRatio(_uniqueness_ratio);
     _bm_matcher->setTextureThreshold(_texture_threshold);
     _bm_matcher->setSpeckleWindowSize(_speckle_size);
     _bm_matcher->setSpeckleRange(_speckle_range);
     _bm_matcher->setPreFilterType(cv::StereoBM::PREFILTER_XSOBEL);
     }

    void setSGBMParameters() {
     _sgbm_matcher->setPreFilterCap(_prefilter_cap);
     _sgbm_matcher->setBlockSize(_correlation_window_size);
     _sgbm_matcher->setMinDisparity(_min_disparity);
     _sgbm_matcher->setNumDisparities(_disparity_range);
     _sgbm_matcher->setUniquenessRatio(_uniqueness_ratio);
     _sgbm_matcher->setSpeckleRange(_speckle_range);
     _sgbm_matcher->setSpeckleWindowSize(_speckle_size);
     _sgbm_matcher->setP1(8 * 1 *  _sgbmWindsize*_sgbmWindsize);
     _sgbm_matcher->setP2(32 * 1 * _sgbmWindsize * _sgbmWindsize);
     _sgbm_matcher->setDisp12MaxDiff(_disp12MaxDiff);
     _sgbm_matcher->setMode(cv::StereoSGBM::MODE_HH);
    }

    void processDisparity(const sensor_msgs::ImageConstPtr& left_img_rect_ptr,
    const sensor_msgs::ImageConstPtr& right_img_rect_ptr,
    double camera_data[4],
    stereo_msgs::DisparityImage& disparity) {
      const cv::Mat_<uint8_t> left_img =
        cv_bridge::toCvShare(left_img_rect_ptr,
        sensor_msgs::image_encodings::MONO8)->image;
      const cv::Mat_<uint8_t> right_img =
        cv_bridge::toCvShare(right_img_rect_ptr,
        sensor_msgs::image_encodings::MONO8)->image;
      static const int DISPARITY_PER_PIXEL = 16;
      static const double INV_DISPARITY_PER_PIXEL = 1.0/DISPARITY_PER_PIXEL;
      cv::Mat_<int16_t> disparity16;

      if (_disparity_image_alg) {
        _bm_matcher->compute(left_img, right_img, disparity16);
        disparity.min_disparity = _bm_matcher->getMinDisparity();
        disparity.max_disparity = _bm_matcher->getMinDisparity() +
          _bm_matcher->getNumDisparities() - 1;
      } else {
        _sgbm_matcher->compute(left_img, right_img, disparity16);
        disparity.min_disparity = _sgbm_matcher->getMinDisparity();
        disparity.max_disparity = _sgbm_matcher->getMinDisparity() +
          _sgbm_matcher->getNumDisparities() - 1;
      }
      sensor_msgs::Image& dimage = disparity.image;
      dimage.height = disparity16.rows;
      dimage.width = disparity16.cols;
      dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      dimage.step = dimage.width * sizeof(float);
      dimage.data.resize(dimage.step * dimage.height);
      cv::Mat_<float> dmat(dimage.height, dimage.width,
        (float*)&dimage.data[0], dimage.step);
      disparity16.convertTo(dmat, dmat.type(),
        INV_DISPARITY_PER_PIXEL, -(camera_data[0] - camera_data[1]));
      ROS_ASSERT(dmat.data &dimage.data[0]);

      disparity.f = camera_data[2];
      disparity.T = camera_data[3];
      disparity.delta_d = INV_DISPARITY_PER_PIXEL;
    }
 private :
    typedef image_transport::SubscriberFilter ImageSubscriber;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
      sensor_msgs::CameraInfo, sensor_msgs::Image,
      sensor_msgs::CameraInfo> ApproxPolicyPinhole;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
      sensor_msgs::CameraInfo, sensor_msgs::Image,
      sensor_msgs::CameraInfo> ExactPolicyPinhole;

    typedef message_filters::Synchronizer<ApproxPolicyPinhole>
      ApproxSyncPinhole;
    typedef message_filters::Synchronizer<ExactPolicyPinhole>
      ExactTimeSyncPinhole;
    typedef message_filters::Subscriber<sensor_msgs::CameraInfo>
      CameraInfoPinholeSubscriber;

    boost::shared_ptr<ApproxSyncPinhole> _approxSyncPinhole;
    boost::shared_ptr<ExactTimeSyncPinhole> _exactTimeSyncPinhole;

    ros::Publisher _pub_disparity;

    ImageSubscriber _sub_left_image, _sub_right_image;
    CameraInfoPinholeSubscriber _sub_left_pinhole_camerainfo,
      _sub_right_pinhole_camerainfo;
    image_geometry::StereoCameraModel _stereo_pin_model;
    std::map<std::string, double> camera_params;

    int _square_size;
    bool _approx_sync;

    int _prefilter_size;
    int _prefilter_cap;
    int _correlation_window_size;
    int _min_disparity;
    int _disparity_range;
    int _uniqueness_ratio;
    int _texture_threshold;
    int _speckle_size;
    int _speckle_range;
    cv::Ptr<cv::StereoBM> _bm_matcher;
    // SGBM paramters
    int _disp12MaxDiff;
    int _sgbmWindsize;
    bool _disparity_image_alg;
    cv::Ptr<cv::StereoSGBM> _sgbm_matcher;
};


PLUGINLIB_EXPORT_CLASS(mynteye_image_pipeline::Disparity, nodelet::Nodelet);
}  // namespace mynteye_image_pipeline
