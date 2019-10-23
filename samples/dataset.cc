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
#include "dataset.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iomanip>
#include <limits>
#include <stdexcept>
#include <utility>

#include "mynteye/logger.h"
#include "mynteye/util/files.h"

#define FULL_PRECISION \
  std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)

#define IMAGE_FILENAME_WIDTH 6

MYNTEYE_BEGIN_NAMESPACE

namespace tools {

Dataset::Dataset(std::string outdir) : outdir_(std::move(outdir)) {
  VLOG(2) << __func__;
  if (!files::mkdir(outdir_)) {
    LOG(FATAL) << "Create directory failed: " << outdir_;
  }
}

Dataset::~Dataset() {
  VLOG(2) << __func__;
  for (auto &&it = stream_writers_.begin(); it != stream_writers_.end(); it++) {
    if (it->second) {
      it->second->ofs.flush();
      it->second->ofs.close();
    }
  }
  if (motion_writer_) {
    motion_writer_->ofs.flush();
    motion_writer_->ofs.close();
  }
}

void Dataset::SaveStreamData(
    const Stream &stream, const device::StreamData &data) {
  auto &&writer = GetStreamWriter(stream);
  auto seq = stream_counts_[stream];
  writer->ofs << seq << ", " << data.img->frame_id << ", "
              << data.img->timestamp << ", " << data.img->exposure_time
              << std::endl;
  if (data.frame) {
    std::stringstream ss;
    ss << writer->outdir << MYNTEYE_OS_SEP << std::dec
       << std::setw(IMAGE_FILENAME_WIDTH) << std::setfill('0') << seq << ".png";
    if (data.frame->format() == Format::GREY) {
      cv::Mat img(
          data.frame->height(), data.frame->width(), CV_8UC1,
          data.frame->data());
      cv::imwrite(ss.str(), img);
    } else if (data.frame->format() == Format::YUYV) {
      cv::Mat img(
          data.frame->height(), data.frame->width(), CV_8UC2,
          data.frame->data());
      cv::cvtColor(img, img, cv::COLOR_YUV2BGR_YUY2);
      cv::imwrite(ss.str(), img);
    } else if (data.frame->format() == Format::BGR888) {
      cv::Mat img(
          data.frame->height(), data.frame->width(), CV_8UC3,
          data.frame->data());
      cv::imwrite(ss.str(), img);
    } else {
      cv::Mat img(
          data.frame->height(), data.frame->width(), CV_8UC1,
          data.frame->data());
      cv::imwrite(ss.str(), img);
    }
  }
  ++stream_counts_[stream];
}

void Dataset::SaveMotionData(const device::MotionData &data) {
  auto &&writer = GetMotionWriter();
  // auto seq = data.imu->serial_number;
  auto seq = motion_count_;
  writer->ofs << seq << ", " << static_cast<int>(data.imu->flag) << ", "
              << data.imu->timestamp << ", " << data.imu->accel[0] << ", "
              << data.imu->accel[1] << ", " << data.imu->accel[2] << ", "
              << data.imu->gyro[0] << ", " << data.imu->gyro[1] << ", "
              << data.imu->gyro[2] << ", " << data.imu->temperature
              << std::endl;
  ++motion_count_;
  /*
  if(motion_count_ != seq) {
    LOG(INFO) << "motion_count_ != seq !" << " motion_count_: " << motion_count_
  << " seq: " << seq;
    motion_count_ = seq;
  }
  */
}

void Dataset::SaveStreamData(
    const Stream &stream, const api::StreamData &data) {
  auto &&writer = GetStreamWriter(stream);
  auto seq = stream_counts_[stream];
  writer->ofs << seq << ", " << data.img->frame_id << ", "
              << data.img->timestamp << ", " << data.img->exposure_time
              << std::endl;
  if (!data.frame.empty()) {
    std::stringstream ss;
    ss << writer->outdir << MYNTEYE_OS_SEP << std::dec
       << std::setw(IMAGE_FILENAME_WIDTH) << std::setfill('0') << seq << ".png";
       cv::imwrite(ss.str(), data.frame);
  }
  ++stream_counts_[stream];
}

void Dataset::SaveMotionData(const api::MotionData &data) {
  auto &&writer = GetMotionWriter();
  // auto seq = data.imu->serial_number;
  auto seq = motion_count_;
  writer->ofs << seq << ", " << static_cast<int>(data.imu->flag) << ", "
                << data.imu->timestamp << ", " << data.imu->accel[0] << ", "
                << data.imu->accel[1] << ", " << data.imu->accel[2] << ", "
                << data.imu->gyro[0] << ", " << data.imu->gyro[1] << ", "
                << data.imu->gyro[2] << ", " << data.imu->temperature
                << std::endl;

  motion_count_++;
  /*
  if(motion_count_ != seq) {
    LOG(INFO) << "motion_count_ != seq !" << " motion_count_: " << motion_count_
  << " seq: " << seq;
    motion_count_ = seq;
  }
  */
}

Dataset::writer_t Dataset::GetStreamWriter(const Stream &stream) {
  try {
    return stream_writers_.at(stream);
  } catch (const std::out_of_range &e) {
    writer_t writer = std::make_shared<Writer>();
    switch (stream) {
      case Stream::LEFT: {
        writer->outdir = outdir_ + MYNTEYE_OS_SEP "left";
      } break;
      case Stream::RIGHT: {
        writer->outdir = outdir_ + MYNTEYE_OS_SEP "right";
      } break;
      case Stream::DEPTH: {
        writer->outdir = outdir_ + MYNTEYE_OS_SEP "depth";
      } break;
      case Stream::DISPARITY: {
        writer->outdir = outdir_ + MYNTEYE_OS_SEP "disparity";
      } break;
      case Stream::RIGHT_RECTIFIED: {
        writer->outdir = outdir_ + MYNTEYE_OS_SEP "right_rect";
      } break;
      case Stream::LEFT_RECTIFIED: {
        writer->outdir = outdir_ + MYNTEYE_OS_SEP "left_rect";
      } break;
      case Stream::DISPARITY_NORMALIZED: {
         writer->outdir = outdir_ + MYNTEYE_OS_SEP "disparity_norm";
      } break;
      default:
        LOG(FATAL) << "Unsupported stream: " << stream;
    }
    writer->outfile = writer->outdir + MYNTEYE_OS_SEP "stream.txt";

    files::mkdir(writer->outdir);
    writer->ofs.open(writer->outfile, std::ofstream::out);
    writer->ofs << "seq, frame_id, timestamp, exposure_time" << std::endl;
    writer->ofs << FULL_PRECISION;

    stream_writers_[stream] = writer;
    stream_counts_[stream] = 0;
    return writer;
  }
}

Dataset::writer_t Dataset::GetMotionWriter() {
  if (motion_writer_ == nullptr) {
    writer_t writer = std::make_shared<Writer>();
    writer->outdir = outdir_;
    writer->outfile = writer->outdir + MYNTEYE_OS_SEP "motion.txt";

    files::mkdir(writer->outdir);
    writer->ofs.open(writer->outfile, std::ofstream::out);
    writer->ofs << "seq, flag, timestamp, accel_x, accel_y, accel_z, "
                   "gyro_x, gyro_y, gyro_z, temperature"
                << std::endl;
    writer->ofs << FULL_PRECISION;

    motion_writer_ = writer;
    motion_count_ = 0;
    accel_count_ = 0;
    gyro_count_ = 0;
  }
  return motion_writer_;
}

}  // namespace tools

MYNTEYE_END_NAMESPACE
