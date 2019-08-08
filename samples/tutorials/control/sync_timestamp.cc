#include <opencv2/highgui/highgui.hpp>

#include "mynteye/logger.h"
#include "mynteye/api/api.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);
  if (!api) return 1;

  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);

  Model model = api->GetModel();

  if (model != Model::STANDARD200B) {
    LOG(INFO) << "Sorry, This device don't support sync timestamp";
    return 0;
  }

  if (model == Model::STANDARD200B) {
    std::uint64_t value = 10;
    auto ok = api->SetOptionValue(Option::SYNC_TIMESTAMP, value);
    if (ok) {
      LOG(INFO) << "Set timestamp is successful.";
    } else {
      LOG(INFO) << "Set timestamp is failed.";
    }
  }

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

  api->Stop(Source::VIDEO_STREAMING);
  return 0;
}
