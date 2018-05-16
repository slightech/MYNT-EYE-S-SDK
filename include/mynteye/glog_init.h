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
#ifndef MYNTEYE_GLOG_INIT_H_  // NOLINT
#define MYNTEYE_GLOG_INIT_H_
#pragma once

#include <glog/logging.h>

/** Helper to init glog with args. */
struct glog_init {
  /**
   * Init glog with args in constructor, and shutdown it in destructor.
   */
  glog_init(int argc, char *argv[]) {
    (void)argc;

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

#endif  // MYNTEYE_GLOG_INIT_H_ NOLINT
