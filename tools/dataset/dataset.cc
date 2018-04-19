#include "dataset/dataset.h"

#ifdef USE_OPENCV2
#include <opencv2/highgui/highgui.hpp>
#else
#include <opencv2/imgcodecs/imgcodecs.hpp>
#endif

#include <glog/logging.h>

#include <iomanip>
#include <limits>
#include <stdexcept>
#include <utility>

#include "mynteye/files.h"

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
    ss << writer->outdir << OS_SEP << std::dec
       << std::setw(IMAGE_FILENAME_WIDTH) << std::setfill('0') << seq << ".png";
    cv::Mat img(
        data.frame->height(), data.frame->width(), CV_8UC1, data.frame->data());
    cv::imwrite(ss.str(), img);
  }
  ++stream_counts_[stream];
}

void Dataset::SaveMotionData(const device::MotionData &data) {
  auto &&writer = GetMotionWriter();
  auto seq = motion_count_;
  writer->ofs << seq << ", " << data.imu->frame_id << ", "
              << data.imu->timestamp << ", " << data.imu->accel[0] << ", "
              << data.imu->accel[1] << ", " << data.imu->accel[2] << ", "
              << data.imu->gyro[0] << ", " << data.imu->gyro[1] << ", "
              << data.imu->gyro[2] << ", " << data.imu->temperature
              << std::endl;
  ++motion_count_;
}

Dataset::writer_t Dataset::GetStreamWriter(const Stream &stream) {
  try {
    return stream_writers_.at(stream);
  } catch (const std::out_of_range &e) {
    writer_t writer = std::make_shared<Writer>();
    switch (stream) {
      case Stream::LEFT: {
        writer->outdir = outdir_ + OS_SEP "left";
      } break;
      case Stream::RIGHT: {
        writer->outdir = outdir_ + OS_SEP "right";
      } break;
      default:
        LOG(FATAL) << "Unsupported stream: " << stream;
    }
    writer->outfile = writer->outdir + OS_SEP "stream.txt";

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
    writer->outfile = writer->outdir + OS_SEP "motion.txt";

    files::mkdir(writer->outdir);
    writer->ofs.open(writer->outfile, std::ofstream::out);
    writer->ofs << "seq, frame_id, timestamp, accel_x, accel_y, accel_z, "
                   "gyro_x, gyro_y, gyro_z, temperature"
                << std::endl;
    writer->ofs << FULL_PRECISION;

    motion_writer_ = writer;
    motion_count_ = 0;
  }
  return motion_writer_;
}

}  // namespace tools

MYNTEYE_END_NAMESPACE
