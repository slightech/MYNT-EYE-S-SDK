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
#ifndef MYNTEYE_TUTORIALS_PC_VIEWER_H_  // NOLINT
#define MYNTEYE_TUTORIALS_PC_VIEWER_H_
#pragma once

#include <memory>

#include <opencv2/core/core.hpp>

#include <pcl/visualization/pcl_visualizer.h>

class PCViewer {
 public:
  PCViewer();
  ~PCViewer();

  void Update(const cv::Mat &xyz);

  void Update(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc);

  bool WasVisual() const;
  bool WasStopped() const;

 private:
  void ConvertMatToPointCloud(
      const cv::Mat &xyz, pcl::PointCloud<pcl::PointXYZ>::Ptr pc);

  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
};

#endif  // MYNTEYE_TUTORIALS_PC_VIEWER_H_ NOLINT
