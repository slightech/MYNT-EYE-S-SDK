#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>

#include "mynteye/api/api.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);
  if (!api) return 1;

  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);

  api->EnableStreamData(Stream::DEPTH);

  api->Start(Source::VIDEO_STREAMING);

  cv::namedWindow("frame");
  cv::namedWindow("depth_real");

  std::int32_t count = 0;
  std::cout << "Press 'Space' 's' 'S' to save image." << std::endl;
  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);
    if (!left_data.frame.empty() && !right_data.frame.empty()) {
      cv::Mat img;
      cv::hconcat(left_data.frame, right_data.frame, img);
      cv::imshow("frame", img);
    }

    auto &&depth_data = api->GetStreamData(Stream::DEPTH);
    if (!depth_data.frame.empty()) {
      cv::imshow("depth_real", depth_data.frame);  // CV_16UC1
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    } else if (key == 32 || key == 's' || key == 'S') {
      if (!left_data.frame.empty()
          && !right_data.frame.empty()
          && !depth_data.frame.empty()) {
        char l_name[20];
        char r_name[20];
        char d_name[20];
        ++count;
        snprintf(l_name, sizeof(l_name), "left_%d.jpg", count);
        snprintf(r_name, sizeof(r_name), "right_%d.jpg", count);
        snprintf(d_name, sizeof(d_name), "depth_%d.jpg", count);

        cv::imwrite(l_name, left_data.frame);
        cv::imwrite(r_name, right_data.frame);
        cv::imwrite(d_name, depth_data.frame);

        std::cout << "Saved " << l_name << " " << r_name << " " << d_name << " to current directory" << std::endl;
      }
    }
  }

  api->Stop(Source::VIDEO_STREAMING);
  return 0;
}
