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
#include "util_pcl.h"

// #include <pcl/common/common_headers.h>

#include <cmath>

#include "mynteye/logger.h"

std::shared_ptr<pcl::visualization::PCLVisualizer> CustomColorVis(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc) {
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(
      pc, 255, 255, 255);
  viewer->addPointCloud<pcl::PointXYZ>(pc, single_color, "point cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point cloud");
  // viewer->addCoordinateSystem(1.0);
  viewer->addCoordinateSystem(1000.0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, -150, 0, -1, 0);
  return (viewer);
}

PCViewer::PCViewer() : viewer_(nullptr) {
  VLOG(2) << __func__;
}

PCViewer::~PCViewer() {
  VLOG(2) << __func__;
  if (viewer_) {
    // viewer_->saveCameraParameters("pcl_camera_params.txt");
    viewer_->close();
    viewer_ == nullptr;
  }
}

void PCViewer::Update(const cv::Mat &xyz) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
  ConvertMatToPointCloud(xyz, pc);
  Update(pc);
}

void PCViewer::Update(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc) {
  if (viewer_ == nullptr) {
    viewer_ = CustomColorVis(pc);
  }
  viewer_->updatePointCloud(pc, "point cloud");
  viewer_->spinOnce();
}

bool PCViewer::WasVisual() const {
  return viewer_ != nullptr;
}

bool PCViewer::WasStopped() const {
  return viewer_ != nullptr && viewer_->wasStopped();
}

void PCViewer::ConvertMatToPointCloud(
    const cv::Mat &xyz, pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
  // cv::Mat channels[3];
  // cv::split(xyz, channels);
  // double min, max;
  // cv::minMaxLoc(channels[2], &min, &max);

  for (int i = 0; i < xyz.rows; i++) {
    for (int j = 0; j < xyz.cols; j++) {
      auto &&p = xyz.at<cv::Point3f>(i, j);
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
        // LOG(INFO) << "[" << i << "," << j << "] x: " << p.x << ", y: " << p.y
        // << ", z: " << p.z;
        pcl::PointXYZ point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        // point.z = p.z - min;
        pc->points.push_back(point);
      }
    }
  }

  pc->width = static_cast<int>(pc->points.size());
  pc->height = 1;
}
