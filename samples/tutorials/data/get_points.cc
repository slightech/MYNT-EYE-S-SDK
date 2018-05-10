#include <opencv2/highgui/highgui.hpp>

#include <glog/logging.h>

#include "mynteye/api.h"

#include "data/pcviewer.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);

  api->EnableStreamData(Stream::POINTS);

  api->Start(Source::VIDEO_STREAMING);

  cv::namedWindow("frame");
  PCViewer pcviewer;

  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);

    cv::Mat img;
    cv::hconcat(left_data.frame, right_data.frame, img);
    cv::imshow("frame", img);

    auto &&points_data = api->GetStreamData(Stream::POINTS);
    if (!points_data.frame.empty()) {
      pcviewer.Draw(points_data.frame);
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
    if (pcviewer.WasDrew() && pcviewer.WasStopped()) {
      break;
    }
  }

  api->Stop(Source::VIDEO_STREAMING);
  return 0;
}
