#ifndef MYNTEYE_TOOLS_DATASET_H_  // NOLINT
#define MYNTEYE_TOOLS_DATASET_H_
#pragma once

#include <fstream>
#include <map>
#include <memory>
#include <string>

#include "mynteye/callbacks.h"
#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

namespace tools {

class Dataset {
 public:
  struct Writer {
    std::ofstream ofs;
    std::string outdir;
    std::string outfile;
  };

  using writer_t = std::shared_ptr<Writer>;

  explicit Dataset(std::string outdir);
  ~Dataset();

  void SaveStreamData(const Stream &stream, const device::StreamData &data);
  void SaveMotionData(const device::MotionData &data);

 private:
  writer_t GetStreamWriter(const Stream &stream);
  writer_t GetMotionWriter();

  std::string outdir_;

  std::map<Stream, writer_t> stream_writers_;
  writer_t motion_writer_;

  std::map<Stream, std::size_t> stream_counts_;
  std::size_t motion_count_;
};

}  // namespace tools

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_TOOLS_DATASET_H_ NOLINT
