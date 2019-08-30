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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// #include "mynteye/logger.h"
#include "mynteye/api/api.h"

#include "util_cv.h"
#include "util_pcl.h"

namespace {

class DepthRegion {
 public:
  explicit DepthRegion(std::uint32_t n)
      : n_(std::move(n)), show_(false), selected_(false), point_(0, 0) {}

  ~DepthRegion() = default;

  /**
   * 鼠标事件：默认不选中区域，随鼠标移动而显示。单击后，则会选中区域来显示。你可以再单击已选中区域或双击未选中区域，取消选中。
   */
  void OnMouse(const int &event, const int &x, const int &y, const int &flags) {
    MYNTEYE_UNUSED(flags)
    if (event != cv::EVENT_MOUSEMOVE && event != cv::EVENT_LBUTTONDOWN) {
      return;
    }
    show_ = true;

    if (event == cv::EVENT_MOUSEMOVE) {
      if (!selected_) {
        point_.x = x;
        point_.y = y;
      }
    } else if (event == cv::EVENT_LBUTTONDOWN) {
      if (selected_) {
        if (x >= static_cast<int>(point_.x - n_) &&
            x <= static_cast<int>(point_.x + n_) &&
            y >= static_cast<int>(point_.y - n_) &&
            y <= static_cast<int>(point_.y + n_)) {
          selected_ = false;
        }
      } else {
        selected_ = true;
      }
      point_.x = x;
      point_.y = y;
    }
  }

  template <typename T>
  void ShowElems(
      const cv::Mat &depth,
      std::function<std::string(const T &elem)> elem2string,
      int elem_space = 40,
      std::function<std::string(
          const cv::Mat &depth, const cv::Point &point, const std::uint32_t &n)>
          getinfo = nullptr) {
    if (!show_)
      return;

    int space = std::move(elem_space);
    int n = 2 * n_ + 1;
    cv::Mat im(space * n, space * n, CV_8UC3, cv::Scalar(255, 255, 255));

    int x, y;
    std::string str;
    int baseline = 0;
    for (int i = -n_; i <= n; ++i) {
      x = point_.x + i;
      if (x < 0 || x >= depth.cols)
        continue;
      for (int j = -n_; j <= n; ++j) {
        y = point_.y + j;
        if (y < 0 || y >= depth.rows)
          continue;

        str = elem2string(depth.at<T>(y, x));

        cv::Scalar color(0, 0, 0);
        if (i == 0 && j == 0)
          color = cv::Scalar(0, 0, 255);

        cv::Size sz =
            cv::getTextSize(str, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

        cv::putText(
            im, str, cv::Point(
                         (i + n_) * space + (space - sz.width) / 2,
                         (j + n_) * space + (space + sz.height) / 2),
            cv::FONT_HERSHEY_PLAIN, 1, color, 1);
      }
    }

    if (getinfo) {
      std::string info = getinfo(depth, point_, n_);
      if (!info.empty()) {
        cv::Size sz =
            cv::getTextSize(info, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

        cv::putText(
            im, info, cv::Point(5, 5 + sz.height), cv::FONT_HERSHEY_PLAIN, 1,
            cv::Scalar(255, 0, 255), 1);
      }
    }

    cv::imshow("region", im);
  }

  void DrawRect(cv::Mat &image) {  // NOLINT
    if (!show_)
      return;
    std::uint32_t n = (n_ > 1) ? n_ : 1;
    n += 1;  // outside the region
    cv::rectangle(
        image, cv::Point(point_.x - n, point_.y - n),
        cv::Point(point_.x + n, point_.y + n),
        selected_ ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 1);
  }

 private:
  std::uint32_t n_;
  bool show_;
  bool selected_;
  cv::Point point_;
};

void OnDepthMouseCallback(int event, int x, int y, int flags, void *userdata) {
  DepthRegion *region = reinterpret_cast<DepthRegion *>(userdata);
  region->OnMouse(event, x, y, flags);
}

}  // namespace

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);
  if (!api) return 1;

  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);

  api->SetOptionValue(Option::IR_CONTROL, 80);
  api->SetDisparityComputingMethodType(DisparityComputingMethod::BM);
  api->EnableStreamData(Stream::DISPARITY_NORMALIZED);
  api->EnableStreamData(Stream::POINTS);
  api->EnableStreamData(Stream::DEPTH);

  api->Start(Source::VIDEO_STREAMING);

  cv::namedWindow("frame");
  cv::namedWindow("depth");
  cv::namedWindow("region");

  DepthRegion depth_region(3);
  auto depth_info = [](
      const cv::Mat &depth, const cv::Point &point, const std::uint32_t &n) {
    MYNTEYE_UNUSED(depth)
    std::ostringstream os;
    os << "depth pos: [" << point.y << ", " << point.x << "]"
       << "±" << n << ", unit: mm";
    return os.str();
  };

  CVPainter painter;
  PCViewer pcviewer;

  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);

    cv::Mat img;
    cv::hconcat(left_data.frame, right_data.frame, img);

    painter.DrawImgData(img, *left_data.img);

    cv::imshow("frame", img);
    // LOG(INFO) << "left id: " << left_data.frame_id
    //     << ", right id: " << right_data.frame_id;

    auto &&disp_data = api->GetStreamData(Stream::DISPARITY_NORMALIZED);
    auto &&depth_data = api->GetStreamData(Stream::DEPTH);
    if (!disp_data.frame.empty() && !depth_data.frame.empty()) {
      // Show disparity instead of depth, but show depth values in region.
      auto &&depth_frame = disp_data.frame;

#ifdef WITH_OPENCV3
      // ColormapTypes
      //   http://docs.opencv.org/master/d3/d50/group__imgproc__colormap.html#ga9a805d8262bcbe273f16be9ea2055a65
      cv::applyColorMap(depth_frame, depth_frame, cv::COLORMAP_JET);
#endif

      cv::setMouseCallback("depth", OnDepthMouseCallback, &depth_region);
      // Note: DrawRect will change some depth values to show the rect.
      depth_region.DrawRect(depth_frame);

      cv::imshow("depth", depth_frame);
      // LOG(INFO) << "depth id: " << disp_data.frame_id;

      depth_region.ShowElems<ushort>(
          depth_data.frame,
          [](const ushort &elem) {
            if (elem >= 10000) {
              // Filter errors, or limit to valid range.
              //
              // reprojectImageTo3D(), missing values will set to 10000
              //   https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga1bc1152bd57d63bc524204f21fde6e02
              return std::string("invalid");
            }
            return std::to_string(elem);
          },
          80, depth_info);
    }

    auto &&points_data = api->GetStreamData(Stream::POINTS);
    if (!points_data.frame.empty()) {
      pcviewer.Update(points_data.frame);
      // LOG(INFO) << "points id: " << points_data.frame_id;
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
    if (pcviewer.WasStopped()) {
      break;
    }
  }

  api->Stop(Source::VIDEO_STREAMING);
  return 0;
}
