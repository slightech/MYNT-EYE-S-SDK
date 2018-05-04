#include <opencv2/highgui/highgui.hpp>

#include <glog/logging.h>

#include "mynteye/api.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);

  // Detect infrared add-ons
  LOG(INFO) << "Support infrared: " << std::boolalpha
            << api->Supports(AddOns::INFRARED);
  LOG(INFO) << "Support infrared2: " << std::boolalpha
            << api->Supports(AddOns::INFRARED2);

  // Get infrared intensity range
  auto &&info = api->GetOptionInfo(Option::IR_CONTROL);
  LOG(INFO) << Option::IR_CONTROL << ": {" << info << "}";

  // Set infrared intensity value
  api->SetOptionValue(Option::IR_CONTROL, 80);

  api->Start(Source::VIDEO_STREAMING);

  cv::namedWindow("frame");

  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);

    cv::Mat img;
    cv::hconcat(left_data.frame, right_data.frame, img);
    cv::imshow("frame", img);

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  api->Stop(Source::ALL);
  return 0;
}
