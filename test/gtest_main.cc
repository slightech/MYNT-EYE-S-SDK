#include <stdio.h>

#include "gtest/gtest.h"

#include "mynteye/glog_init.h"

int main(int argc, char **argv) {
  glog_init _(argc, argv);

  printf("Running main() from gtest_main.cc\n");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
