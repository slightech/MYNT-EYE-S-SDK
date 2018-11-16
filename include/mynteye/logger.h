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
#ifndef MYNTEYE_LOGGER_H_
#define MYNTEYE_LOGGER_H_
#pragma once

#ifdef WITH_GLOG

#include <glog/logging.h>

/** Helper to init glog with args. */
struct glog_init {
  /**
   * Init glog with args in constructor, and shutdown it in destructor.
   */
  glog_init(int argc, char *argv[]) {
    (void)argc;

    // Set whether log messages go to stderr instead of logfiles
    FLAGS_logtostderr = true;

    // Set whether log messages go to stderr in addition to logfiles.
    // FLAGS_alsologtostderr = true;

    // Set color messages logged to stderr (if supported by terminal).
    FLAGS_colorlogtostderr = true;

    // Log suppression level: messages logged at a lower level than this
    // are suppressed.
    FLAGS_minloglevel = google::GLOG_INFO;

    // If specified, logfiles are written into this directory instead of the
    // default logging directory.
    FLAGS_log_dir = ".";

    // Sets the maximum log file size (in MB).
    FLAGS_max_log_size = 8;

    // Sets whether to avoid logging to the disk if the disk is full.
    FLAGS_stop_logging_if_full_disk = true;

    // Show all VLOG(m) messages for m <= this.
    // FLAGS_v = 2;

    google::InitGoogleLogging(argv[0]);

    VLOG(2) << __func__;
  }

  ~glog_init() {
    VLOG(2) << __func__;
    google::ShutdownGoogleLogging();
  }
};

#else

struct glog_init {
  glog_init(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    // Do nothing.
  }
};

#define MYNTEYE_MAX_LOG_LEVEL google::INFO

#include "mynteye/miniglog.h"

#endif

#endif  // MYNTEYE_LOGGER_H_
