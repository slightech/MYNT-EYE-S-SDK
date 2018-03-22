#include <glog/logging.h>

#include "mynteye/mynteye.h"
#include "uvc/uvc.h"

struct glog_init {
  glog_init(int /*argc*/, char *argv[]) {
    // FLAGS_logtostderr = true;
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    FLAGS_log_dir = ".";
    FLAGS_max_log_size = 1024;
    FLAGS_stop_logging_if_full_disk = true;

    FLAGS_v = 2;

    google::InitGoogleLogging(argv[0]);

    VLOG(2) << __func__;
  }

  ~glog_init() {
    VLOG(2) << __func__;
    google::ShutdownGoogleLogging();
  }
};

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  glog_init _(argc, argv);

  auto context = uvc::create_context();
  auto devices = uvc::query_devices(context);
  LOG_IF(FATAL, devices.size() <= 0) << "No devices :(";
  for (auto &&device : devices) {
    auto vid = uvc::get_vendor_id(*device);
    auto pid = uvc::get_product_id(*device);
    LOG(INFO) << "vid: " << vid << ", pid: " << pid;
  }

  return 0;
}
