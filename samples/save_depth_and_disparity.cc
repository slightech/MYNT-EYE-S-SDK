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

  cv::namedWindow("disparity");
  cv::namedWindow("depth_real");

  std::int32_t count = 0;
  std::cout << "Press 'Space' 's' 'S' to save image." << std::endl;
  while (true) {
    api->WaitForStreams();

    auto &&depth_data = api->GetStreamData(Stream::DEPTH);
    if (!depth_data.frame.empty()) {
      cv::imshow("depth_real", depth_data.frame);  // CV_16UC1
    }

    auto &&dis_data = api->GetStreamData(Stream::DISPARITY);
    if (!dis_data.frame.empty()) {
      cv::imshow("disparity", dis_data.frame);  // CV_16UC1
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    } else {
      if (!depth_data.frame.empty()) {
        char d_name[20];
        ++count;
        snprintf(d_name, sizeof(d_name), "depth_%d.jpg", count);

        cv::imwrite(d_name, depth_data.frame);

        std::cout << "Saved " << d_name << " to current directory" << std::endl;
      }

      if (!dis_data.frame.empty()) {
        char dis_name[20];
        ++count;
        snprintf(dis_name, sizeof(dis_name), "disparity_%d.jpg", count);

        cv::imwrite(dis_name, dis_data.frame);

        std::cout << "Saved " << dis_name << " to current directory" << std::endl;
      }
    }
  }

  api->Stop(Source::VIDEO_STREAMING);
  return 0;
}
