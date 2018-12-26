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

// #define GOOGLE_STRIP_LOG 1
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
#define GLOG_NO_ABBREVIATED_SEVERITIES
#endif

#ifdef HAVE_LIB_GFLAGS
#include <gflags/gflags.h>
#endif

#include "mynteye/logger.h"
#include "mynteye/mynteye.h"

int main(int /*argc*/, char *argv[]) {
  // Set whether log messages go to stderr instead of logfiles
  // FLAGS_logtostderr = true;
  // Set whether log messages go to stderr in addition to logfiles.
  FLAGS_alsologtostderr = true;
  // Set color messages logged to stderr (if supported by terminal).
  FLAGS_colorlogtostderr = true;

  // If specified, logfiles are written into this directory instead of the
  // default logging directory.
  FLAGS_log_dir = ".";
  // Sets the maximum log file size (in MB).
  FLAGS_max_log_size = 1024;
  // Sets whether to avoid logging to the disk if the disk is full.
  FLAGS_stop_logging_if_full_disk = true;

  // Show all VLOG(m) messages for m <= this.
  FLAGS_v = 2;

  // Initialize google's logging library.
  google::InitGoogleLogging(argv[0]);
#ifdef HAVE_LIB_GFLAGS
  // Optional: parse command line flags
  gflags::ParseCommandLineFlags(&argc, &argv, true);
#endif

  if (VLOG_IS_ON(2)) {
    VLOG(2) << "do some logging preparation and logging";
  }

  VLOG(2) << "verbose msg";
  VLOG(1) << "debug msg";
  LOG(INFO) << "info msg";
  LOG(WARNING) << "warning msg";
  LOG(ERROR) << "error msg";
  // LOG(FATAL) << "fatal msg";

  LOG(INFO) << "MYNTEYE API version is " << MYNTEYE_API_VERSION_STR;
  if (MYNTEYE_API_VERSION >= MYNTEYE_API_VERSION_CHECK(2, 0, 0)) {
    LOG(INFO) << "MYNTEYE API version is greater than or equal to 2.0.0";
  } else {
    LOG(INFO) << "MYNTEYE API version is less than 2.0.0";
  }

  // Shutdown google's logging library.
  google::ShutdownGoogleLogging();
  return 0;
}

// miniglog: https://github.com/tzutalin/miniglog
/*
ANDROID_LOG_FATAL,    // LOG(FATAL)
ANDROID_LOG_ERROR,    // LOG(ERROR)
ANDROID_LOG_WARN,     // LOG(WARNING)
ANDROID_LOG_INFO,     // LOG(INFO), LG, VLOG(0)
ANDROID_LOG_DEBUG,    // VLOG(1)
ANDROID_LOG_VERBOSE,  // VLOG(2) .. VLOG(N)
*/
