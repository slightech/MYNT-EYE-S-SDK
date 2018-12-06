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
#include <opencv2/calib3d/calib3d.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <camodocal/camera_models/Camera.h>
#include <camodocal/camera_models/CameraFactory.h>
#include <camodocal/camera_models/CataCamera.h>
#include <camodocal/camera_models/EquidistantCamera.h>
#include <camodocal/camera_models/PinholeCamera.h>
#include <camodocal/gpl/gpl.h>
#include <mynteye_image_pipeline/tools/tools.h>
#include <mynteye_image_pipeline/camera_manager.h>
#include <camodocal/camera_models/Camera.h>
#include <opencv2/core/eigen.hpp>
namespace mynteye_image_pipeline {
  static void icvGetRectangles(const CvMat* cameraMatrix,
                                const CvMat* distCoeffs, const CvMat* R,
                                const CvMat* newCameraMatrix,
                                CvSize imgSize, cv::Rect_<float>& inner,
                                cv::Rect_<float>& outer) {
    const int N = 9;
    int x, y, k;
    cv::Ptr<CvMat> _pts(cvCreateMat(1, N*N, CV_32FC2));
    CvPoint2D32f* pts = (CvPoint2D32f*)(_pts->data.ptr);

    for (y = k = 0; y < N; y++) {
      for (x = 0; x < N; x++) {
        pts[k++] = cvPoint2D32f((float)x*imgSize.width/(N-1),
                                 (float)y*imgSize.height/(N-1));
      }
    }

    cvUndistortPoints(_pts, _pts, cameraMatrix, distCoeffs, R, newCameraMatrix);
    float iX0 = -FLT_MAX, iX1 = FLT_MAX, iY0 = -FLT_MAX, iY1 = FLT_MAX;
    float oX0 = FLT_MAX, oX1 = -FLT_MAX, oY0 = FLT_MAX, oY1 = -FLT_MAX;
    // find the inscribed rectangle.
    // the code will likely not work with extreme rotation matrices (R) (>45%)
    for (y = k = 0; y < N; y++) {
        for (x = 0; x < N; x++) {
          CvPoint2D32f p = pts[k++];
          oX0 = MIN(oX0, p.x);
          oX1 = MAX(oX1, p.x);
          oY0 = MIN(oY0, p.y);
          oY1 = MAX(oY1, p.y);

          if (x == 0)
              iX0 = MAX(iX0, p.x);
          if (x == N-1)
              iX1 = MIN(iX1, p.x);
          if (y == 0)
              iY0 = MAX(iY0, p.y);
          if (y == N-1)
              iY1 = MIN(iY1, p.y);
        }
    }
    inner = cv::Rect_<float>(iX0, iY0, iX1-iX0, iY1-iY0);
    outer = cv::Rect_<float>(oX0, oY0, oX1-oX0, oY1-oY0);
  }

class PubCamera : public nodelet::Nodelet {
 public:
  PubCamera() : _square_size(5), _approx_sync(false),
                _left_frame_id("left_frame_id"),
                _right_frame_id("right_frame_id") {}
  virtual ~PubCamera() {}
  virtual void onInit() {
    ros::NodeHandle &pnh = getPrivateNodeHandle();
    ros::NodeHandle &nh = getNodeHandle();
    std::string left_topic = "left_topic";
    std::string right_topic = "right_topic";

    // publish topic
    std::string pub_left_image_topic = "pub_left_image_topic";
    std::string pub_right_image_topic = "pub_right_image_topic";
    std::string left_camera_info = "left_camera_info";
    std::string right_camera_info = "right_camera_info";

    // calibration file
    _left_camera_yaml = "left_camera_yaml";
    _right_camera_yaml = "right_camera_yaml";
    _extrinsics_yaml = "extrinsics_yaml";

    pnh.param("square_size", _square_size, _square_size);
    pnh.param("approx_sync", _approx_sync, _approx_sync);
    pnh.param("sub_left_image_topic", left_topic, left_topic);
    pnh.param("sub_right_image_topic", right_topic, right_topic);

    pnh.param("pub_left_image_topic",
                pub_left_image_topic, pub_left_image_topic);
    pnh.param("pub_right_image_topic",
                pub_right_image_topic, pub_right_image_topic);
    pnh.param("pub_left_camera_info_topic", left_camera_info, left_camera_info);
    pnh.param("pub_right_camera_info_topic",
                right_camera_info, right_camera_info);
    pnh.param("left_camera_yaml", _left_camera_yaml, _left_camera_yaml);
    pnh.param("right_camera_yaml", _right_camera_yaml, _right_camera_yaml);
    pnh.param("extrinsics_yaml", _extrinsics_yaml, _extrinsics_yaml);

    // load frame

    pnh.param("base_frame_id",  _base_frame_id, _base_frame_id);
    pnh.param("left_frame_id",  _left_frame_id, _left_frame_id);
    pnh.param("right_frame_id", _right_frame_id, _right_frame_id);

    pnh.param("left_rect_frame_id",
                 _left_rect_frame_id, _left_rect_frame_id);
    pnh.param("right_rect_frame_id",
                _right_rect_frame_id, _right_rect_frame_id);
    pnh.param("depth_frame_id",  _depth_frame_id, _depth_frame_id);
    pnh.param("points_frame_id",  _points_frame_id, _points_frame_id);
    pnh.param("laser_scan_depth_frame_id",
                _laser_scan_depth_frame_id, _laser_scan_depth_frame_id);
    {
      NODELET_INFO_STREAM("sub_image: _square_size: " << _square_size);
      NODELET_INFO_STREAM("sub_image: _approx_sync: " << _approx_sync);
      NODELET_INFO_STREAM("sub_image: left_topic: " << left_topic);
      NODELET_INFO_STREAM("sub_image: right_topic: " << right_topic);

      NODELET_INFO_STREAM("pub_left_image_topic: " << pub_left_image_topic);
      NODELET_INFO_STREAM("pub_right_image_topic: " << pub_right_image_topic);
      NODELET_INFO_STREAM("sub_image: pub_left_camera_info_topic: "
                            << left_camera_info);
      NODELET_INFO_STREAM("sub_image: pub_right_camera_info_topic: "
                            << right_camera_info);
      NODELET_INFO_STREAM("left_camera_yaml: " << _left_camera_yaml);
      NODELET_INFO_STREAM("right_camera_yaml: " << _right_camera_yaml);
      NODELET_INFO_STREAM("extrinsics_yaml: " << _extrinsics_yaml);
      NODELET_INFO_STREAM("left_frame_id: " << _left_frame_id);
      NODELET_INFO_STREAM("right_frame_id: " << _right_frame_id);
      NODELET_INFO_STREAM("_laser_scan_depth_frame_id: "
                            << _laser_scan_depth_frame_id);
    }
    _cam_odo_ptr = camodocal::CameraFactory::instance()->
                    generateCameraFromYamlFile(_left_camera_yaml);

    stereoRectify(_left_camera_yaml, _right_camera_yaml, _extrinsics_yaml);
    publishStaticTransforms();
    image_transport::ImageTransport it(nh);
    _img_left_sub.subscribe(it, left_topic, 1);
    _img_right_sub.subscribe(it, right_topic, 1);

    left_pub.reset(new image_transport::Publisher(
                    it.advertise(pub_left_image_topic, 1)));
    right_pub.reset(new image_transport::Publisher(
                    it.advertise(pub_right_image_topic, 1)));
    left_camerainfo_pub.reset(new ros::Publisher(
                    nh.advertise<sensor_msgs::CameraInfo>(
                    left_camera_info, 1)));
    right_camerainfo_pub.reset(new ros::Publisher(
                    nh.advertise<sensor_msgs::CameraInfo>(
                    right_camera_info, 1)));

    if (_approx_sync) {
      _approximate_sync.reset(new StereoApproSync(
        StereoApproximatePolicy(_square_size),
        _img_left_sub,  _img_right_sub));
      _approximate_sync->registerCallback(
        boost::bind(&PubCamera::process, this, _1, _2));
    } else {
      _exact_sync.reset(new StereoExactSync(
        StereoExactPolicy(_square_size), _img_left_sub,  _img_right_sub));
      _exact_sync->registerCallback(
        boost::bind(&PubCamera::process, this, _1, _2));
    }
  }

  void process(
      const sensor_msgs::ImageConstPtr &img_left_ptr,
      const sensor_msgs::ImageConstPtr &img_right_ptr
      ) {
        ROS_ASSERT(img_left_ptr->
          encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 &&
          img_right_ptr->encoding.compare(
            sensor_msgs::image_encodings::MONO8) == 0);

      cv::Mat left_img = cv_bridge::toCvShare(img_left_ptr, "8UC1")->image;
      cv::Mat right_img = cv_bridge::toCvShare(img_right_ptr, "8UC1")->image;
      auto &&left_msg = cv_bridge::CvImage(setHeader(
        img_left_ptr->header, _left_frame_id),
        sensor_msgs::image_encodings::MONO8, left_img);
      auto &&right_msg = cv_bridge::CvImage(setHeader(
        img_right_ptr->header, _right_frame_id),
        sensor_msgs::image_encodings::MONO8, right_img);
      left_pub->publish(left_msg.toImageMsg());
      right_pub->publish(right_msg.toImageMsg());
      setHeader(_camera_info_ptr["left"],
        img_left_ptr->header, _left_frame_id);
      setHeader(_camera_info_ptr["right"],
        img_right_ptr->header, _right_frame_id);
      left_camerainfo_pub->publish(_camera_info_ptr["left"]);
      right_camerainfo_pub->publish(_camera_info_ptr["right"]);
  }

  void setHeader(sensor_msgs::CameraInfoPtr& camera_info_ptr,
                  const std_msgs::Header &header, const std::string& frame_id) {
    camera_info_ptr->header.seq = header.seq;
    camera_info_ptr->header.stamp = header.stamp;
    camera_info_ptr->header.frame_id = frame_id;
  }

  std_msgs::Header setHeader(const std_msgs::Header& header,
                              const std::string& frame_id) {
    std_msgs::Header local_header;
    local_header.seq = header.seq;
    local_header.stamp = header.stamp;
    local_header.frame_id = frame_id;
    return local_header;
  }

  sensor_msgs::CameraInfoPtr generateCameraInfo(
                  const std::string& frame_id, const std_msgs::Header& header) {
    if (_camera_info_ptr.find(frame_id) != _camera_info_ptr.end()) {
      _camera_info_ptr[frame_id]->header = header;
      return _camera_info_ptr[frame_id];
    }
    _camera_info_ptr[frame_id] = CameraManagerTools::generateCameraFromYamlFile(
                                  _remap_frame_yaml[frame_id]);
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
              _camera_info_ptr[frame_id]->R.data()) = _R[frame_id];
      Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
      K(0, 0) = _camera_info_ptr[frame_id]->K[0];
      K(0, 2) = _camera_info_ptr[frame_id]->K[2];
      K(1, 1) = _camera_info_ptr[frame_id]->K[4];
      K(1, 2) = _camera_info_ptr[frame_id]->K[5];
      Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(
        _camera_info_ptr[frame_id]->P.data()) = (Eigen::Matrix<
          double, 3, 4>() << K, Eigen::Vector3d(
          _baseline[frame_id]*K(0, 0), 0, 0)).finished();

      return _camera_info_ptr[frame_id];
  }

  void loadCameraMatrix(const std::string& camera_yaml,
                        cv::Mat &K, cv::Mat &D,
                        cv::Size &image_size,
                        sensor_msgs::CameraInfoPtr& camera_info_ptr) {
    K = cv::Mat(3, 3, CV_64F, camera_info_ptr->K.data());
    std::size_t d_length = camera_info_ptr->D.size();
    D = cv::Mat(1, d_length, CV_64F, camera_info_ptr->D.data());
    image_size = cv::Size(camera_info_ptr->width, camera_info_ptr->height);
  }

  void stereoRectify(const std::string& left_camera_yaml,
                      const std::string& right_camera_yaml,
                      std::string &extrinsics_yaml) {

    Eigen::Matrix4d T = loadT(extrinsics_yaml);
    Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
    Eigen::Vector3d t = T.topRightCorner<3, 1>();
    cv::Mat cv_R, cv_t;
    cv::eigen2cv(R, cv_R);
    cv::eigen2cv(t, cv_t);
    cv::Mat K1, D1, K2, D2;
    cv::Size image_size1, image_size2;

    _camera_info_ptr["left"] =
        CameraManagerTools::generateCameraFromYamlFile(left_camera_yaml);
     _camera_info_ptr["right"] =
        CameraManagerTools::generateCameraFromYamlFile(right_camera_yaml);

    loadCameraMatrix(left_camera_yaml, K1,
                      D1, image_size1,
                      _camera_info_ptr["left"]);
    loadCameraMatrix(right_camera_yaml, K2,
                      D2, image_size2,
                      _camera_info_ptr["right"]);

    cv::Mat R1 = cv::Mat(cv::Size(3, 3), CV_64F);
    cv::Mat R2 = cv::Mat(cv::Size(3, 3), CV_64F);
    cv::Mat P1 = cv::Mat(3, 4, CV_64F);
    cv::Mat P2 = cv::Mat(3, 4, CV_64F);

    CvMat c_R = cv_R, c_t = cv_t;
    CvMat c_K1 = K1, c_K2 = K2, c_D1 = D1, c_D2 = D2;
    CvMat c_R1 = R1, c_R2 = R2, c_P1 = P1, c_P2 = P2;
    stereoRectify(&c_K1, &c_K2, &c_D1,
                  &c_D2, image_size1, &c_R,
                  &c_t, &c_R1, &c_R2, &c_P1,
                  &c_P2);

    for (std::size_t i = 0; i < 3; i++) {
      for (std::size_t j = 0; j < 4; j++) {
        _camera_info_ptr["left"]->P.at(i*4+j) = P1.at<double>(i, j);
        _camera_info_ptr["right"]->P.at(i*4+j) = P2.at<double>(i, j);
      }
    }
    for (std::size_t i = 0; i < 3; i++) {
      for (std::size_t j = 0; j < 3; j++) {
        _camera_info_ptr["left"]->R.at(i*3+j) = R1.at<double>(i, j);
        _camera_info_ptr["right"]->R.at(i*3+j) = R2.at<double>(i, j);
      }
    }
  }

  void stereoRectify(
    const CvMat* K1, const CvMat* K2, const CvMat* D1,
    const CvMat* D2, CvSize imageSize, const CvMat* matR,
    const CvMat* matT, CvMat* _R1, CvMat* _R2, CvMat* _P1,
    CvMat* _P2, int flags = cv::CALIB_ZERO_DISPARITY,
    double alpha = -1, CvSize newImgSize = cv::Size()) {
    double _om[3], _t[3] = {0}, _uu[3]={0, 0, 0}, _r_r[3][3], _pp[3][4];
    double _ww[3], _wr[3][3], _z[3] = {0, 0, 0}, _ri[3][3], _w3[3];
    cv::Rect_<float> inner1, inner2, outer1, outer2;

    CvMat om  = cvMat(3, 1, CV_64F, _om);
    CvMat t   = cvMat(3, 1, CV_64F, _t);
    CvMat uu  = cvMat(3, 1, CV_64F, _uu);
    CvMat r_r = cvMat(3, 3, CV_64F, _r_r);
    CvMat pp  = cvMat(3, 4, CV_64F, _pp);
    CvMat ww  = cvMat(3, 1, CV_64F, _ww);
    CvMat w3  = cvMat(3, 1, CV_64F, _w3);
    CvMat wR  = cvMat(3, 3, CV_64F, _wr);
    CvMat Z   = cvMat(3, 1, CV_64F, _z);
    CvMat Ri  = cvMat(3, 3, CV_64F, _ri);
    double nx = imageSize.width, ny = imageSize.height;
    int i, k;
    double nt, nw;
    if (matR->rows == 3 && matR->cols == 3)
        cvRodrigues2(matR, &om);
    else
        cvConvert(matR, &om);
    cvConvertScale(&om, &om, -0.5);
    cvRodrigues2(&om, &r_r);
    cvMatMul(&r_r, matT, &t);

    int idx = fabs(_t[0]) > fabs(_t[1]) ? 0 : 1;

    _uu[2] = 1;
    cvCrossProduct(&uu, &t, &ww);
    nt = cvNorm(&t, 0, CV_L2);
    nw = cvNorm(&ww, 0, CV_L2);
    cvConvertScale(&ww, &ww, 1 / nw);
    cvCrossProduct(&t, &ww, &w3);
    nw = cvNorm(&w3, 0, CV_L2);
    cvConvertScale(&w3, &w3, 1 / nw);
    _uu[2] = 0;

    for (i = 0; i < 3; ++i) {
        _wr[idx][i] = -_t[i] / nt;
        _wr[idx ^ 1][i] = -_ww[i];
        _wr[2][i] = _w3[i] * (1 - 2 * idx);
    }

    // apply to both views
    cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, CV_GEMM_B_T);
    cvConvert(&Ri, _R1);
    cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, 0);
    cvConvert(&Ri, _R2);
    cvMatMul(&Ri, matT, &t);

    // calculate projection/camera matrices
    double fc_new = DBL_MAX;
    CvPoint2D64f cc_new[2] = {{0, 0}, {0, 0}};
    newImgSize = newImgSize.width *
                  newImgSize.height != 0 ? newImgSize : imageSize;
    const double ratio_x = (double)newImgSize.width / imageSize.width / 2;
    const double ratio_y = (double)newImgSize.height / imageSize.height / 2;
    const double ratio = idx == 1 ? ratio_x : ratio_y;
    fc_new = (cvmGet(K1, idx ^ 1, idx ^ 1) +
              cvmGet(K2, idx ^ 1, idx ^ 1)) * ratio;

    for (k = 0; k < 2; k++) {
        cc_new[k].x = (nx)/2;
        cc_new[k].y = (ny)/2;
    }

    if (flags & cv::CALIB_ZERO_DISPARITY) {
      cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x)*0.5;
      cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;
    } else if (idx == 0) {  // horizontal stereo
      cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;
    } else {  // vertical stereo
      cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x)*0.5;
    }

    cvZero( &pp );
    _pp[0][0] = _pp[1][1] = fc_new;
    _pp[0][2] = cc_new[0].x;
    _pp[1][2] = cc_new[0].y;
    _pp[2][2] = 1;
    cvConvert(&pp, _P1);

    _pp[0][2] = cc_new[1].x;
    _pp[1][2] = cc_new[1].y;
    _pp[idx][3] = _t[idx]*fc_new;  // baseline * focal length
    cvConvert(&pp, _P2);


    alpha = MIN(alpha, 1.);

    icvGetRectangles(K1, D1, _R1, _P1, imageSize, inner1, outer1);
    icvGetRectangles(K2, D2, _R2, _P2, imageSize, inner2, outer2);

    {
      newImgSize = newImgSize.width *
                    newImgSize.height != 0 ? newImgSize : imageSize;
      double cx1_0 = cc_new[0].x;
      double cy1_0 = cc_new[0].y;
      double cx2_0 = cc_new[1].x;
      double cy2_0 = cc_new[1].y;
      double cx1 = newImgSize.width*cx1_0/imageSize.width;
      double cy1 = newImgSize.height*cy1_0/imageSize.height;
      double cx2 = newImgSize.width*cx2_0/imageSize.width;
      double cy2 = newImgSize.height*cy2_0/imageSize.height;
      double s = 1.;

      if (alpha >= 0) {
          double s0 = std::max(std::max(std::max(static_cast<double>(cx1) /
                                (cx1_0 - inner1.x),
                              static_cast<double>(cy1) / (cy1_0 - inner1.y)),
                              static_cast<double>(newImgSize.width - cx1) /
                                (inner1.x + inner1.width - cx1_0)),
                              static_cast<double>(newImgSize.height - cy1) /
                                (inner1.y + inner1.height - cy1_0));
          s0 = std::max(std::max(std::max(
                                  std::max(static_cast<double>(cx2) /
                                              (cx2_0 - inner2.x),
                                            static_cast<double>(cy2) /
                                              (cy2_0 - inner2.y)),
                                  static_cast<double> (newImgSize.width - cx2) /
                                    (inner2.x + inner2.width - cx2_0)),
                                  static_cast<double> (newImgSize.height - cy2)/
                                    (inner2.y + inner2.height - cy2_0)),
                                s0);

          double s1 = std::min(std::min(std::min(static_cast<double>(cx1) /
                                (cx1_0 - outer1.x),
                              static_cast<double>(cy1) / (cy1_0 - outer1.y)),
                              static_cast<double>(newImgSize.width - cx1) /
                                (outer1.x + outer1.width - cx1_0)),
                              static_cast<double>(newImgSize.height - cy1) /
                                (outer1.y + outer1.height - cy1_0));
          s1 = std::min(std::min(std::min(std::min(
                              static_cast<double>(cx2) / (cx2_0 - outer2.x),
                              static_cast<double>(cy2) / (cy2_0 - outer2.y)),
                              static_cast<double>(newImgSize.width - cx2) /
                                (outer2.x + outer2.width - cx2_0)),
                              static_cast<double>(newImgSize.height - cy2) /
                                (outer2.y + outer2.height - cy2_0)),
                            s1);

          s = s0*(1 - alpha) + s1*alpha;
      }

      fc_new *= s;
      cc_new[0] = cvPoint2D64f(cx1, cy1);
      cc_new[1] = cvPoint2D64f(cx2, cy2);

      cvmSet(_P1, 0, 0, fc_new);
      cvmSet(_P1, 1, 1, fc_new);
      cvmSet(_P1, 0, 2, cx1);
      cvmSet(_P1, 1, 2, cy1);

      cvmSet(_P2, 0, 0, fc_new);
      cvmSet(_P2, 1, 1, fc_new);
      cvmSet(_P2, 0, 2, cx2);
      cvmSet(_P2, 1, 2, cy2);
      cvmSet(_P2, idx, 3, s*cvmGet(_P2, idx, 3));
    }
  }

  //  void (const CvMat* camera_matrix, const CvMat* distCoeffs, CvPoint2D64f)

  Eigen::Matrix4d loadT(const std::string& extrinsics_yaml) {
    cv::FileStorage fs(extrinsics_yaml, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      NODELET_INFO_STREAM("extrinsics_yaml can not open.");
    }
    double q_x = fs["transform"]["q_x"];
    double q_y = fs["transform"]["q_y"];
    double q_z = fs["transform"]["q_z"];
    double q_w = fs["transform"]["q_w"];
    double t_x = fs["transform"]["t_x"];
    double t_y = fs["transform"]["t_y"];
    double t_z = fs["transform"]["t_z"];
    fs.release();

    Eigen::Quaterniond q(q_w, q_x, q_y, q_z);
    q.normalize();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3, 3>() = q.toRotationMatrix();
    T.topRightCorner<3, 1>() << t_x, t_y, t_z;

    _q = q;
    _t << t_x, t_y, t_z;

    return T;
  }

  void publishStaticTransforms() {
    ros::Time tf_stamp = ros::Time::now();
    std::string optical_frame_id = "optical_frame";
    // base_link to left frame
    geometry_msgs::TransformStamped base_optical;
    base_optical.header.stamp = tf_stamp;
    base_optical.header.frame_id = _base_frame_id;
    base_optical.child_frame_id = optical_frame_id;
    base_optical.transform.translation.x = 0;
    base_optical.transform.translation.y = 0;
    base_optical.transform.translation.z = 0;
    tf::Quaternion q;
    // q.setRPY(-1.57, 0, 1.57);
    q.setRPY(-1.57, 0, -1.57);
    base_optical.transform.rotation.x = q.x();
    base_optical.transform.rotation.y = q.y();
    base_optical.transform.rotation.z = q.z();
    base_optical.transform.rotation.w = q.w();
    static_tf_broadcaster_.sendTransform(base_optical);

    // base_link to left frame
    geometry_msgs::TransformStamped optical2left_raw;
    optical2left_raw.header.stamp = tf_stamp;
    optical2left_raw.header.frame_id = optical_frame_id;
    optical2left_raw.child_frame_id = _left_frame_id;
    optical2left_raw.transform.translation.x = 0;
    optical2left_raw.transform.translation.y = 0;
    optical2left_raw.transform.translation.z = 0;
    optical2left_raw.transform.rotation.x = 0;
    optical2left_raw.transform.rotation.y = 0;
    optical2left_raw.transform.rotation.z = 0;
    optical2left_raw.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(optical2left_raw);

    // base_link to left_rect frame
    geometry_msgs::TransformStamped optical2left_rect;
    optical2left_rect.header.stamp = tf_stamp;
    optical2left_rect.header.frame_id = optical_frame_id;
    optical2left_rect.child_frame_id = _left_rect_frame_id;
    optical2left_rect.transform.translation.x = 0;
    optical2left_rect.transform.translation.y = 0;
    optical2left_rect.transform.translation.z = 0;
    optical2left_rect.transform.rotation.x = 0;
    optical2left_rect.transform.rotation.y = 0;
    optical2left_rect.transform.rotation.z = 0;
    optical2left_rect.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(optical2left_rect);

    // base_link to right_frame
    geometry_msgs::TransformStamped optical2right_raw;
    optical2right_raw.header.stamp = tf_stamp;
    optical2right_raw.header.frame_id = optical_frame_id;
    optical2right_raw.child_frame_id = _right_frame_id;
    optical2right_raw.transform.translation.x = _t[0];
    optical2right_raw.transform.translation.y = _t[1];
    optical2right_raw.transform.translation.z = _t[2];
    optical2right_raw.transform.rotation.x = _q.x();
    optical2right_raw.transform.rotation.y = _q.y();
    optical2right_raw.transform.rotation.z = _q.z();
    optical2right_raw.transform.rotation.w = _q.w();
    static_tf_broadcaster_.sendTransform(optical2right_raw);

    // base_link to right_rect_frame
    geometry_msgs::TransformStamped optical2right_rect;
    optical2right_rect.header.stamp = tf_stamp;
    optical2right_rect.header.frame_id = optical_frame_id;
    optical2right_rect.child_frame_id = _right_rect_frame_id;
    optical2right_rect.transform.translation.x = _t[0];
    optical2right_rect.transform.translation.y = _t[1];
    optical2right_rect.transform.translation.z = _t[2];
    optical2right_rect.transform.rotation.x = _q.x();
    optical2right_rect.transform.rotation.y = _q.y();
    optical2right_rect.transform.rotation.z = _q.z();
    optical2right_rect.transform.rotation.w = _q.w();
    static_tf_broadcaster_.sendTransform(optical2right_rect);

    // base_link to depth_frame
    geometry_msgs::TransformStamped optical2depth;
    optical2depth.header.stamp = tf_stamp;
    optical2depth.header.frame_id = optical_frame_id;
    optical2depth.child_frame_id = _depth_frame_id;
    optical2depth.transform.translation.x = 0;
    optical2depth.transform.translation.y = 0;
    optical2depth.transform.translation.z = 0;
    optical2depth.transform.rotation.x = 0;
    optical2depth.transform.rotation.y = 0;
    optical2depth.transform.rotation.z = 0;
    optical2depth.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(optical2depth);

    // base_link to points_frame
    geometry_msgs::TransformStamped optical2points;
    optical2points.header.stamp = tf_stamp;
    optical2points.header.frame_id = optical_frame_id;
    optical2points.child_frame_id = _points_frame_id;
    optical2points.transform.translation.x = 0;
    optical2points.transform.translation.y = 0;
    optical2points.transform.translation.z = 0;
    optical2points.transform.rotation.x = 0;
    optical2points.transform.rotation.y = 0;
    optical2points.transform.rotation.z = 0;
    optical2points.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(optical2points);

    // base_link to laser_scan_depth_frame
    geometry_msgs::TransformStamped optical2scan;
    optical2scan.header.stamp = tf_stamp;
    optical2scan.header.frame_id = _base_frame_id;
    optical2scan.child_frame_id = _laser_scan_depth_frame_id;
    optical2scan.transform.translation.x = 0;
    optical2scan.transform.translation.y = 0;
    optical2scan.transform.translation.z = 0;
    tf::Quaternion q_scan;
    q_scan.setRPY(0, 0, 0);
    optical2scan.transform.rotation.x = q_scan.x();
    optical2scan.transform.rotation.y = q_scan.y();
    optical2scan.transform.rotation.z = q_scan.z();
    optical2scan.transform.rotation.w = q_scan.w();
    static_tf_broadcaster_.sendTransform(optical2scan);
  }

 private:
  typedef image_transport::SubscriberFilter ImageSubscriber;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      StereoApproximatePolicy;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                    sensor_msgs::Image>
      StereoExactPolicy;
  typedef message_filters::Synchronizer<StereoApproximatePolicy>
      StereoApproSync;
  typedef message_filters::Synchronizer<StereoExactPolicy> StereoExactSync;

  std::map<std::string, sensor_msgs::CameraInfoPtr> _camera_info_ptr;
  ImageSubscriber _img_left_sub;
  ImageSubscriber _img_right_sub;

  boost::shared_ptr<StereoApproSync> _approximate_sync;
  boost::shared_ptr<StereoExactSync> _exact_sync;
  // publish image and camerainfo
  boost::shared_ptr<image_transport::Publisher> left_pub;
  boost::shared_ptr<image_transport::Publisher> right_pub;
  boost::shared_ptr<ros::Publisher> left_camerainfo_pub;
  boost::shared_ptr<ros::Publisher> right_camerainfo_pub;
  int _square_size;
  bool _approx_sync;
  std::string _left_frame_id;
  std::string _right_frame_id;
  std::string _left_rect_frame_id = "left_rect_frame_id";
  std::string _right_rect_frame_id = "right_rect_frame_id";
  std::string _depth_frame_id = "depth_frame_id";
  std::string _points_frame_id = "points_frame_id";
  std::string _laser_scan_depth_frame_id = "laser_scan_depth_frame_id";
  std::string _base_frame_id;
  std::string _left_camera_yaml;
  std::string _right_camera_yaml;
  Eigen::Quaterniond _q;
  Eigen::Vector3d _t;
  std::string _extrinsics_yaml;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  std::map<std::string, std::string> _remap_frame_yaml;
  std::map<std::string, Eigen::Matrix3d> _R;
  std::map<std::string, double> _baseline;
  camodocal::CameraPtr _cam_odo_ptr;
};
PLUGINLIB_EXPORT_CLASS(mynteye_image_pipeline::PubCamera, nodelet::Nodelet);
}  // namespace mynteye_image_pipeline
