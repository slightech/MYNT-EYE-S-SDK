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
#include "mynteye/api/processor/rectify_processor.h"

#include <utility>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "mynteye/logger.h"
#include "mynteye/device/device.h"

// #define WITH_CAM_MODELS

#ifdef WITH_CAM_MODELS

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

struct camera_info {
  unsigned int height = 0;
  unsigned int width = 0;
  std::string distortion_model = "null";
  float  D[4] = {0};
  float  K[9] = {0};
  float  R[9] = {0};
  float  P[12] = {0};
};

struct camera_mat_info_pair {
  struct camera_info left;
  struct camera_info right;
};

cv::Mat rectifyrad(const cv::Mat& R) {
  cv::Mat r_vec;
  cv::Rodrigues(R, r_vec);
  //  pi/180 = x/179 ==> x = 3.1241
  double rad = cv::norm(r_vec);
  if (rad >= 3.1241) {
    cv::Mat r_dir;
    cv::normalize(r_vec, r_dir);
    cv::Mat r = r_dir*(3.1415926 - rad);
    cv::Mat r_r;
    cv::Rodrigues(r, r_r);
    return r_r.clone();
  }
  return R.clone();
}

void stereoRectify(camodocal::CameraPtr leftOdo,
    camodocal::CameraPtr rightOdo, const CvMat* K1, const CvMat* K2,
    const CvMat* D1, const CvMat* D2, CvSize imageSize,
    const CvMat* matR, const CvMat* matT,
    CvMat* _R1, CvMat* _R2, CvMat* _P1, CvMat* _P2,
    int flags = cv::CALIB_ZERO_DISPARITY, double alpha = -1,
    CvSize newImgSize = cv::Size()) {
  double _om[3], _t[3] = {0}, _uu[3]={0, 0, 0}, _r_r[3][3], _pp[3][4];
  double _ww[3], _wr[3][3], _z[3] = {0, 0, 0}, _ri[3][3], _w3[3];
  cv::Rect_<float> inner1, inner2, outer1, outer2;

  CvMat om  = cvMat(3, 1, CV_64F, _om);
  CvMat t   = cvMat(3, 1, CV_64F, _t);
  CvMat uu  = cvMat(3, 1, CV_64F, _uu);
  CvMat r_r = cvMat(3, 3, CV_64F, _r_r);
  CvMat pp  = cvMat(3, 4, CV_64F, _pp);
  CvMat ww  = cvMat(3, 1, CV_64F, _ww);  // temps
  CvMat w3  = cvMat(3, 1, CV_64F, _w3);  // temps
  CvMat wR  = cvMat(3, 3, CV_64F, _wr);
  CvMat Z   = cvMat(3, 1, CV_64F, _z);
  CvMat Ri  = cvMat(3, 3, CV_64F, _ri);
  double nx = imageSize.width, ny = imageSize.height;
  int i, k;
  double nt, nw;
  if ( matR->rows == 3 && matR->cols == 3)
      cvRodrigues2(matR, &om);          // get vector rotation
  else
      cvConvert(matR, &om);  // it's already a rotation vector
  cvConvertScale(&om, &om, -0.5);  // get average rotation
  cvRodrigues2(&om, &r_r);  // rotate cameras to same orientation by averaging
  cvMatMul(&r_r, matT, &t);

  int idx = fabs(_t[0]) > fabs(_t[1]) ? 0 : 1;

  // if idx == 0
  //   e1 = T / ||T||
  //   e2 = e1 x [0,0,1]

  // if idx == 1
  //   e2 = T / ||T||
  //   e1 = e2 x [0,0,1]

  // e3 = e1 x e2

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
      _wr[2][i] = _w3[i] * (1 - 2 * idx); // if idx == 1 -> opposite direction
  }

  // apply to both views
  cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, CV_GEMM_B_T);
  cvConvert( &Ri, _R1 );
  cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, 0);
  cvConvert( &Ri, _R2 );
  cvMatMul(&Ri, matT, &t);

  // calculate projection/camera matrices
  // these contain the relevant rectified image internal params (fx, fy=fx, cx, cy)
  double fc_new = DBL_MAX;
  CvPoint2D64f cc_new[2] = {{0,0}, {0,0}};
  newImgSize = newImgSize.width * newImgSize.height != 0 ? newImgSize : imageSize;
  const double ratio_x = (double)newImgSize.width / imageSize.width / 2;
  const double ratio_y = (double)newImgSize.height / imageSize.height / 2;
  const double ratio = idx == 1 ? ratio_x : ratio_y;
  fc_new = (cvmGet(K1, idx ^ 1, idx ^ 1) + cvmGet(K2, idx ^ 1, idx ^ 1)) * ratio;

  for( k = 0; k < 2; k++ )
  {
      CvPoint2D32f _pts[4];
      CvPoint3D32f _pts_3[4];
      CvMat pts = cvMat(1, 4, CV_32FC2, _pts);
      CvMat pts_3 = cvMat(1, 4, CV_32FC3, _pts_3);
      Eigen::Vector2d a;
      Eigen::Vector3d b;
      for( i = 0; i < 4; i++ )
      {
          int j = (i<2) ? 0 : 1;
          a.x() = (float)((i % 2)*(nx));
          a.y() = (float)(j*(ny));
    if (0 == k) {
      leftOdo->liftProjective(a, b);
    } else {
      rightOdo->liftProjective(a, b);
    }
    _pts[i].x = b.x()/b.z();
    _pts[i].y = b.y()/b.z();
      }
      cvConvertPointsHomogeneous( &pts, &pts_3 );

      //Change camera matrix to have cc=[0,0] and fc = fc_new
      double _a_tmp[3][3];
      CvMat A_tmp  = cvMat(3, 3, CV_64F, _a_tmp);
      _a_tmp[0][0]=fc_new;
      _a_tmp[1][1]=fc_new;
      _a_tmp[0][2]=0.0;
      _a_tmp[1][2]=0.0;
      cvProjectPoints2( &pts_3, k == 0 ? _R1 : _R2, &Z, &A_tmp, 0, &pts );
      CvScalar avg = cvAvg(&pts);
      cc_new[k].x = (nx)/2 - avg.val[0];
      cc_new[k].y = (ny)/2 - avg.val[1];

      //cc_new[k].x = (nx)/2;
      //cc_new[k].y = (ny)/2;
  }

  if( flags & cv::CALIB_ZERO_DISPARITY )
  {
      cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x)*0.5;
      cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;
  }
  else if( idx == 0 ) // horizontal stereo
      cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;
  else // vertical stereo
      cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x)*0.5;

  cvZero( &pp );
  _pp[0][0] = _pp[1][1] = fc_new;
  _pp[0][2] = cc_new[0].x;
  _pp[1][2] = cc_new[0].y;
  _pp[2][2] = 1;
  cvConvert(&pp, _P1);

  _pp[0][2] = cc_new[1].x;
  _pp[1][2] = cc_new[1].y;
  _pp[idx][3] = _t[idx]*fc_new; // baseline * focal length
  cvConvert(&pp, _P2);


  alpha = MIN(alpha, 1.);

  //icvGetRectangles( K1, D1, _R1, _P1, imageSize, inner1, outer1 );
  //icvGetRectangles( K2, D2, _R2, _P2, imageSize, inner2, outer2 );

  {
  newImgSize = newImgSize.width*newImgSize.height != 0 ? newImgSize : imageSize;
  double cx1_0 = cc_new[0].x;
  double cy1_0 = cc_new[0].y;
  double cx2_0 = cc_new[1].x;
  double cy2_0 = cc_new[1].y;
  double cx1 = newImgSize.width*cx1_0/imageSize.width;
  double cy1 = newImgSize.height*cy1_0/imageSize.height;
  double cx2 = newImgSize.width*cx2_0/imageSize.width;
  double cy2 = newImgSize.height*cy2_0/imageSize.height;
  double s = 1.;

  //if( alpha >= 0 )
  //{
  //    double s0 = std::max(std::max(std::max((double)cx1/(cx1_0 - inner1.x), (double)cy1/(cy1_0 - inner1.y)),
  //                        (double)(newImgSize.width - cx1)/(inner1.x + inner1.width - cx1_0)),
  //                    (double)(newImgSize.height - cy1)/(inner1.y + inner1.height - cy1_0));
  //    s0 = std::max(std::max(std::max(std::max((double)cx2/(cx2_0 - inner2.x), (double)cy2/(cy2_0 - inner2.y)),
  //                     (double)(newImgSize.width - cx2)/(inner2.x + inner2.width - cx2_0)),
  //                 (double)(newImgSize.height - cy2)/(inner2.y + inner2.height - cy2_0)),
  //             s0); 

  //    double s1 = std::min(std::min(std::min((double)cx1/(cx1_0 - outer1.x), (double)cy1/(cy1_0 - outer1.y)),
  //                        (double)(newImgSize.width - cx1)/(outer1.x + outer1.width - cx1_0)),
  //                    (double)(newImgSize.height - cy1)/(outer1.y + outer1.height - cy1_0));
  //    s1 = std::min(std::min(std::min(std::min((double)cx2/(cx2_0 - outer2.x), (double)cy2/(cy2_0 - outer2.y)),
  //                     (double)(newImgSize.width - cx2)/(outer2.x + outer2.width - cx2_0)),
  //                 (double)(newImgSize.height - cy2)/(outer2.y + outer2.height - cy2_0)),
  //             s1);

  //    s = s0*(1 - alpha) + s1*alpha;
  //}

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

Eigen::Matrix4d loadT(const mynteye::Extrinsics& in) {
  Eigen::Matrix3d R;
  R<<
  in.rotation[0][0], in.rotation[0][1], in.rotation[0][2],
  in.rotation[1][0], in.rotation[1][1], in.rotation[1][2],
  in.rotation[2][0], in.rotation[2][1], in.rotation[2][2];

  double t_x = in.translation[0];
  double t_y = in.translation[1];
  double t_z = in.translation[2];

  Eigen::Quaterniond q(R);
  q.normalize();
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() = q.toRotationMatrix();
  T.topRightCorner<3, 1>() << t_x, t_y, t_z;

  return T;
}

void loadCameraMatrix(cv::Mat& K, cv::Mat& D, cv::Size& image_size,
    struct camera_info& calib_data) {
  K = cv::Mat(3, 3, CV_64F, calib_data.K);
  std::size_t d_length = 4;
  D = cv::Mat(1, d_length, CV_64F, calib_data.D);
  image_size = cv::Size(calib_data.width, calib_data.height);
}

struct camera_info getCalibMatData(const mynteye::IntrinsicsEquidistant& in) {
  struct camera_info calib_mat_data;
  calib_mat_data.distortion_model = "KANNALA_BRANDT";
  calib_mat_data.height = in.height;
  calib_mat_data.width = in.width;

  for (unsigned int i = 0; i < 4; i++) {
    calib_mat_data.D[i] = in.coeffs[i];
  }

  calib_mat_data.K[0] = in.coeffs[4];  // mu
  calib_mat_data.K[4] = in.coeffs[5];  // mv();
  calib_mat_data.K[2] = in.coeffs[6];  // u0();
  calib_mat_data.K[5] = in.coeffs[7];  // v0();
  calib_mat_data.K[8] = 1;
  return calib_mat_data;
}

struct camera_mat_info_pair stereoRectify(
    camodocal::CameraPtr leftOdo,
    camodocal::CameraPtr rightOdo,
    mynteye::IntrinsicsEquidistant in_left,
    mynteye::IntrinsicsEquidistant in_right,
    mynteye::Extrinsics ex_right_to_left) {
  Eigen::Matrix4d T = loadT(ex_right_to_left);
  Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
  Eigen::Vector3d t = T.topRightCorner<3, 1>();
  cv::Mat cv_R, cv_t;
  cv::eigen2cv(R, cv_R);
  cv::eigen2cv(t, cv_t);
  cv::Mat K1, D1, K2, D2;
  cv::Size image_size1, image_size2;

  struct camera_info calib_mat_data_left = getCalibMatData(in_left);
  struct camera_info calib_mat_data_right = getCalibMatData(in_right);

  loadCameraMatrix(K1, D1, image_size1, calib_mat_data_left);
  loadCameraMatrix(K2, D2, image_size2, calib_mat_data_right);

  cv::Mat R1 = cv::Mat(cv::Size(3, 3), CV_64F);
  cv::Mat R2 = cv::Mat(cv::Size(3, 3), CV_64F);
  cv::Mat P1 = cv::Mat(3, 4, CV_64F);
  cv::Mat P2 = cv::Mat(3, 4, CV_64F);

  CvMat c_R = cv_R, c_t = cv_t;
  CvMat c_K1 = K1, c_K2 = K2, c_D1 = D1, c_D2 = D2;
  CvMat c_R1 = R1, c_R2 = R2, c_P1 = P1, c_P2 = P2;
  stereoRectify(leftOdo, rightOdo, &c_K1, &c_K2, &c_D1, &c_D2,
      image_size1, &c_R, &c_t, &c_R1, &c_R2, &c_P1, &c_P2);

  std::cout << "K1: " << K1 << std::endl;
  std::cout << "D1: " << D1 << std::endl;
  std::cout << "K2: " << K2 << std::endl;
  std::cout << "D2: " << D2 << std::endl;
  std::cout << "R: " << cv_R << std::endl;
  std::cout << "t: " << cv_t << std::endl;
  std::cout << "R1: " << R1 << std::endl;
  std::cout << "R2: " << R2 << std::endl;
  std::cout << "P1: " << P1 << std::endl;
  std::cout << "P2: " << P2 << std::endl;

  R1 = rectifyrad(R1);
  R2 = rectifyrad(R2);

  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 4; j++) {
      calib_mat_data_left.P[i*4 + j] = P1.at<double>(i, j);
      calib_mat_data_right.P[i*4 + j] = P2.at<double>(i, j);
    }
  }

  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      calib_mat_data_left.R[i*3 + j] = R1.at<double>(i, j);
      calib_mat_data_right.R[i*3 +j] = R2.at<double>(i, j);
    }
  }
  struct camera_mat_info_pair res = {calib_mat_data_left, calib_mat_data_right};
  return res;
}

#endif

MYNTEYE_BEGIN_NAMESPACE

const char RectifyProcessor::NAME[] = "RectifyProcessor";

RectifyProcessor::RectifyProcessor(
    std::shared_ptr<Device> device, std::int32_t proc_period)
    : Processor(std::move(proc_period)), device_(device) {
  VLOG(2) << __func__ << ": proc_period=" << proc_period;
  calib_model = CalibrationModel::UNKNOW;
  NotifyImageParamsChanged();
}

RectifyProcessor::~RectifyProcessor() {
  VLOG(2) << __func__;
}

std::string RectifyProcessor::Name() {
  return NAME;
}

void RectifyProcessor::NotifyImageParamsChanged() {
  auto in_left = device_->GetIntrinsics(Stream::LEFT);
  auto in_right = device_->GetIntrinsics(Stream::RIGHT);
  if (in_left->calib_model() == CalibrationModel::PINHOLE) {
    InitParams(
      *std::dynamic_pointer_cast<IntrinsicsPinhole>(in_left),
      *std::dynamic_pointer_cast<IntrinsicsPinhole>(in_right),
      device_->GetExtrinsics(Stream::RIGHT, Stream::LEFT));
  } else if (in_left->calib_model() ==
             CalibrationModel::KANNALA_BRANDT) {
#ifdef WITH_CAM_MODELS
    InitParams(
      *std::dynamic_pointer_cast<IntrinsicsEquidistant>(in_left),
      *std::dynamic_pointer_cast<IntrinsicsEquidistant>(in_right),
      device_->GetExtrinsics(Stream::RIGHT, Stream::LEFT));
#else
     VLOG(2) << "calib model type KANNALA_BRANDT"
             << " is not been enabled.";
#endif
  } else {
    VLOG(2) << "calib model type "
            << in_left->calib_model()
            <<" is not been enabled.";
  }
}

Object *RectifyProcessor::OnCreateOutput() {
  return new ObjMat2();
}

bool RectifyProcessor::OnProcess(
    Object *const in, Object *const out, Processor *const parent) {
  MYNTEYE_UNUSED(parent)
  if (calib_model == CalibrationModel::PINHOLE) {
    const ObjMat2 *input = Object::Cast<ObjMat2>(in);
    ObjMat2 *output = Object::Cast<ObjMat2>(out);
    cv::remap(input->first, output->first, map11, map12, cv::INTER_LINEAR);
    cv::remap(input->second, output->second, map21, map22, cv::INTER_LINEAR);
    output->first_id = input->first_id;
    output->first_data = input->first_data;
    output->second_id = input->second_id;
    output->second_data = input->second_data;
    return true;
  } else if (calib_model == CalibrationModel::KANNALA_BRANDT) {
#ifdef WITH_CAM_MODELS
    const ObjMat2 *input = Object::Cast<ObjMat2>(in);
    ObjMat2 *output = Object::Cast<ObjMat2>(out);
    std::cout << in <<std::endl;
    // cv::remap(input->first, output->first, map11, map12, cv::INTER_LINEAR);
    // cv::remap(input->second, output->second, map21, map22, cv::INTER_LINEAR);
    output->first_id = input->first_id;
    output->first_data = input->first_data;
    output->second_id = input->second_id;
    output->second_data = input->second_data;
    return true;
#else
    return false;
#endif
  }
}

void RectifyProcessor::InitParams(
    IntrinsicsPinhole in_left,
    IntrinsicsPinhole in_right,
    Extrinsics ex_right_to_left) {
  calib_model = CalibrationModel::PINHOLE;
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

  VLOG(2) << "InitParams size: " << size;
  VLOG(2) << "M1: " << M1;
  VLOG(2) << "M2: " << M2;
  VLOG(2) << "D1: " << D1;
  VLOG(2) << "D2: " << D2;
  VLOG(2) << "R: " << R;
  VLOG(2) << "T: " << T;

  cv::Rect left_roi, right_roi;
  cv::stereoRectify(
      M1, D1, M2, D2, size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY,
      0, size, &left_roi, &right_roi);

  cv::initUndistortRectifyMap(M1, D1, R1, P1, size, CV_16SC2, map11, map12);
  cv::initUndistortRectifyMap(M2, D2, R2, P2, size, CV_16SC2, map21, map22);
}

#ifdef WITH_CAM_MODELS

camodocal::CameraPtr getCamOdoCameraPtr(const struct camera_info& calib_mat_data) {
  std::string camera_model = calib_mat_data.distortion_model;
  int w = calib_mat_data.width;
  int h = calib_mat_data.height;

  double fx = calib_mat_data.K[0];
  double fy = calib_mat_data.K[4];
  double cx = calib_mat_data.K[2];
  double cy = calib_mat_data.K[5];

  camodocal::EquidistantCameraPtr camera(new camodocal::EquidistantCamera);
  camodocal::EquidistantCamera::Parameters params(camera_model, w, h, calib_mat_data.D[0],
      calib_mat_data.D[1], calib_mat_data.D[2], calib_mat_data.D[3], fx, fy, cx, cy);
  camera->setParameters(params);
  return camera;
}

camodocal::CameraPtr generateCameraFromIntrinsicsEquidistant(
    const mynteye::IntrinsicsEquidistant & in) {
  camodocal::EquidistantCameraPtr camera(
      new camodocal::EquidistantCamera("KANNALA_BRANDT",
                                       in.width,
                                       in.height,
                                       in.coeffs[0],
                                       in.coeffs[1],
                                       in.coeffs[2],
                                       in.coeffs[3],
                                       in.coeffs[4],
                                       in.coeffs[5],
                                       in.coeffs[6],
                                       in.coeffs[7]));
  return camera;
}

void RectifyProcessor::InitParams(
    IntrinsicsEquidistant in_left,
    IntrinsicsEquidistant in_right,
    Extrinsics ex_right_to_left) {

  calib_model = CalibrationModel::KANNALA_BRANDT;
  camodocal::CameraPtr camera_odo_ptr_left_o =
      generateCameraFromIntrinsicsEquidistant(in_left);
  camodocal::CameraPtr camera_odo_ptr_right_o =
      generateCameraFromIntrinsicsEquidistant(in_right);
  struct camera_mat_info_pair calib_info_pair =
      stereoRectify(camera_odo_ptr_left_o,
                    camera_odo_ptr_right_o,
                    in_left,
                    in_right,
                    ex_right_to_left);

  camodocal::CameraPtr camera_odo_ptr_left =
      getCamOdoCameraPtr(calib_info_pair.left);
  camodocal::CameraPtr camera_odo_ptr_right =
      getCamOdoCameraPtr(calib_info_pair.right);

  // for (int i = 0; i < 4 ; i ++) {
  //   std::cout << " D" << i << ": " << calib_info_pair.left.D[i];
  // }
  //   for (int i = 0; i < 9 ; i ++) {
  //   std::cout << " K" << i << ": " << calib_info_pair.left.K[i];
  // }
  // for (int i = 0; i < 9 ; i ++) {
  //   std::cout << " R" << i << ": " << calib_info_pair.left.R[i];
  // }
  // for (int i = 0; i < 12 ; i ++) {
  //   if (i == 0) {
  //     calib_info_pair.left.P[i] = 361.184;
  //   }
  //   if (i == 5) {
  //     calib_info_pair.left.P[i] = 361.184;
  //   }
  //   std::cout << " P" << i << ": " << calib_info_pair.left.P[i];
  // }

  // D0: -0.0167962 D1: -0.0434362 D2: 0.0877926 D3: -0.0569189 K0: 361.172 K1: 0 K2: 381.437 K3: 0 K4: 361.2 K5: 244.849 K6: 0 K7: 0 K8: 1 R0: 0.999997 R1: 0.00230108   R2: 0.00118774   R3: -0.00229864 R4: 0.999995 R5: -0.00205134 R6: -0.00119245 R7: 0.0020486 R8: 0.999997 P0: 361.184 P1: 0 P2: 337.803 P3: 0 P4: 0 P5: 361.184 P6: 247.574 P7: 0 P8: 0 P9: 0 P10: 1 P11: 0
  // D0: -0.0167962 D1: -0.0434362 D2: 0.0877926 D3: -0.0569189 K0: 361.172 K1: 0 K2: 381.437 K3: 0 K4: 361.2 K5: 244.849 K6: 0 K7: 0 K8: 1 R0: 1        R1: -0.000291099 R2: -2.98628e-07 R3: 0.000291099 R4: 1        R5: 1.82261e-07 R6: 2.98575e-07 R7: -1.82348e-07 R8: 1     P0: 0       P1: 0 P2: 376     P3: 0 P4: 0 P5: 0       P6: 240     P7: 0 P8: 0 P9: 0 P10: 1 P11: 0
  cv::Mat rect_R_l =
      cv::Mat::eye(3, 3, CV_32F), rect_R_r = cv::Mat::eye(3, 3, CV_32F);
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      rect_R_l.at<float>(i, j) = calib_info_pair.left.R[i*3+j];
      rect_R_r.at<float>(i, j) = calib_info_pair.right.R[i*3+j];
    }
  }
  double left_f[] =
      {calib_info_pair.left.P[0], calib_info_pair.left.P[5]};
  double left_center[] =
      {calib_info_pair.left.P[2], calib_info_pair.left.P[6]};
  double right_f[] =
      {calib_info_pair.right.P[0], calib_info_pair.right.P[5]};
  double right_center[] =
      {calib_info_pair.right.P[2], calib_info_pair.right.P[6]};
  camera_odo_ptr_left->initUndistortRectifyMap(
      map11, map12, left_f[0], left_f[1],
      cv::Size(0, 0), left_center[0],
      left_center[1], rect_R_l);
  camera_odo_ptr_right->initUndistortRectifyMap(
      map21, map22, right_f[0], right_f[1],
      cv::Size(0, 0), right_center[0],
      right_center[1], rect_R_r);
  std::cout << map11 << std::endl;
}
#endif

MYNTEYE_END_NAMESPACE
