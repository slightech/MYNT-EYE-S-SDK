#ifndef MYNTEYE_TUTORIALS_PCVIEWER_H_  // NOLINT
#define MYNTEYE_TUTORIALS_PCVIEWER_H_
#pragma once

#include <opencv2/core/core.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include <memory>

class PCViewer {
 public:
  PCViewer();
  ~PCViewer();

  void Draw(const cv::Mat &xyz);

  void Draw(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc);

  bool WasDrew() const;
  bool WasStopped() const;

 private:
  void ConvertMatToPointCloud(
      const cv::Mat &xyz, pcl::PointCloud<pcl::PointXYZ>::Ptr pc);

  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
};

#endif  // MYNTEYE_TUTORIALS_PCVIEWER_H_ NOLINT
