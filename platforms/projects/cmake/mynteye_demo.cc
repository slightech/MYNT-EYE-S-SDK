#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>

#include "mynteye/api/api.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char const *argv[]) {
  auto &&api = API::Create(0, nullptr);
  if (!api) return 1;

  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);
  api->Start(Source::VIDEO_STREAMING);

  double fps;
  double t = 0.01;
  std::cout << "fps:" << std::endl;

  cv::namedWindow("frame");

  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);

    cv::Mat img;
    if (!left_data.frame.empty() && !right_data.frame.empty()) {
      double t_c = cv::getTickCount() / cv::getTickFrequency();
      fps = 1.0/(t_c - t);
      printf("\b\b\b\b\b\b\b\b\b%.2f", fps);
      t = t_c;
      cv::hconcat(left_data.frame, right_data.frame, img);
      cv::imshow("frame", img);
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  api->Stop(Source::VIDEO_STREAMING);
  return 0;
}
