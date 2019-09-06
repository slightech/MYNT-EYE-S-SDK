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
#pragma once
#ifndef MYNTEYE_CAMERA_MODELS_CAMERA_H_
#define MYNTEYE_CAMERA_MODELS_CAMERA_H_

#include <vector>
#include <memory>
#include "util/base.h"
#include <opencv2/core/core.hpp>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

namespace models {

class Camera {
 public:
  enum ModelType { KANNALA_BRANDT, MEI, PINHOLE, SCARAMUZZA };

  class Parameters {
   public:
    explicit Parameters(ModelType modelType);

    Parameters(
        ModelType modelType, const std::string &cameraName, int w, int h);

    ModelType &modelType(void);
    std::string &cameraName(void);
    int &imageWidth(void);
    int &imageHeight(void);

    ModelType modelType(void) const;
    const std::string &cameraName(void) const;
    int imageWidth(void) const;
    int imageHeight(void) const;

    int nIntrinsics(void) const;

   protected:
    ModelType m_modelType;
    int m_nIntrinsics;
    std::string m_cameraName;
    int m_imageWidth;
    int m_imageHeight;
  };

  virtual ModelType modelType(void) const = 0;
  virtual const std::string &cameraName(void) const = 0;
  virtual int imageWidth(void) const = 0;
  virtual int imageHeight(void) const = 0;

  virtual cv::Mat &mask(void);
  virtual const cv::Mat &mask(void) const;

  virtual void estimateIntrinsics(
      const cv::Size &boardSize,
      const std::vector<std::vector<cv::Point3f> > &objectPoints,
      const std::vector<std::vector<cv::Point2f> > &imagePoints) = 0;

  virtual void estimateExtrinsics(
      const std::vector<cv::Point3f> &objectPoints,
      const std::vector<cv::Point2f> &imagePoints,
      cv::Mat &rvec, cv::Mat &tvec) const;  // NOLINT      

  // Lift points from the image plane to the projective space
  virtual void liftProjective(
      const models::Vector2d &p, models::Vector3d &P) const = 0;  // NOLINT
  // %output P

  // Projects 3D points to the image plane (Pi function)
  virtual void spaceToPlane(
      const models::Vector3d &P, models::Vector2d &p) const = 0;  // NOLINT
  // %output p

  // Projects 3D points to the image plane (Pi function)
  // and calculates jacobian
  // virtual void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
  //                          Eigen::Matrix<double,2,3>& J) const = 0;
  // %output p
  // %output J

  // virtual void initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale =
  // 1.0) const = 0;
  virtual cv::Mat initUndistortRectifyMap(
      cv::Mat &map1, cv::Mat &map2, float fx = -1.0f, float fy = -1.0f,  // NOLINT
      cv::Size imageSize = cv::Size(0, 0), float cx = -1.0f, float cy = -1.0f,
      cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const = 0;

  virtual int parameterCount(void) const = 0;

  virtual void readParameters(const std::vector<double> &parameters) = 0;
  virtual void writeParameters(std::vector<double> &parameters) const = 0;  // NOLINT

  virtual std::string parametersToString(void) const = 0;  // NOLINT

  /**
   * \brief Calculates the reprojection distance between points
   *
   * \param P1 first 3D point coordinates
   * \param P2 second 3D point coordinates
   * \return euclidean distance in the plane
   */
  double reprojectionDist(
      const models::Vector3d &P1, const models::Vector3d &P2) const;

  double reprojectionError(
      const std::vector<std::vector<cv::Point3f> > &objectPoints,
      const std::vector<std::vector<cv::Point2f> > &imagePoints,
      const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
      cv::OutputArray perViewErrors = cv::noArray()) const;

  double reprojectionError(
      const models::Vector3d &P, const models::Quaterniond &camera_q,
      const models::Vector3d &camera_t,
          const models::Vector2d &observed_p) const;

  void projectPoints(
      const std::vector<cv::Point3f> &objectPoints, const cv::Mat &rvec,
      const cv::Mat &tvec, std::vector<cv::Point2f> &imagePoints) const;  // NOLINT

 protected:
  cv::Mat m_mask;
};

typedef std::shared_ptr<Camera> CameraPtr;
typedef std::shared_ptr<const Camera> CameraConstPtr;
}  // namespace models

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_CAMERA_MODELS_CAMERA_H_

