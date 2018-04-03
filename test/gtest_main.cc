#include "gtest/gtest.h"
#include <stdio.h>
#include "glog/logging.h"

struct glog_init {
  glog_init(int /*argc*/, char *argv[]) {
    // FLAGS_logtostderr = true;
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    FLAGS_log_dir = ".";
    FLAGS_max_log_size = 1024;
    FLAGS_stop_logging_if_full_disk = true;

    // FLAGS_v = 2;

    google::InitGoogleLogging(argv[0]);

    VLOG(2) << __func__;
  }

  ~glog_init() {
    VLOG(2) << __func__;
    google::ShutdownGoogleLogging();
  }
};

GTEST_API_ int main(int argc, char **argv) {
  glog_init _(argc, argv);

  printf("Running main() from gtest_main.cc\n");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
