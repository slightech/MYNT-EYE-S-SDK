#ifndef MYNTEYE_INTERNAL_STREAMS_H_  // NOLINT
#define MYNTEYE_INTERNAL_STREAMS_H_
#pragma once

#include <map>
#include <vector>

#include "mynteye/mynteye.h"
#include "mynteye/types.h"

#include "internal/callbacks.h"

MYNTEYE_BEGIN_NAMESPACE

namespace streams {

class Stream {
 public:
};

}  // namesapce streams

class Streams {
 public:
  using frame_t = device::Frame;
  using stream_data_t = device::StreamData;
  using stream_datas_t = std::vector<stream_data_t>;

  Streams();
  ~Streams();

  void ConfigStream(
      const Capabilities &capability, const StreamRequest &request);

  void PushStream(const Capabilities &capability, const void *data);

  // void WaitForStreams() const;

  // std::vector<StreamData> GetStreamData(const Stream &stream) const;
  // StreamData GetLatestStreamData(const Stream &stream) const;

 private:
  bool IsStreamCapability(const Capabilities &capability) const;
  bool HasStreamConfigRequest(const Capabilities &capability) const;

  const StreamRequest &GetStreamConfigRequest(
      const Capabilities &capability) const;

  bool HasStreamDatas(const Stream &stream) const;
  stream_datas_t &GetStreamDatas(const Stream &stream);

  std::vector<Capabilities> stream_capabilities_;
  std::map<Capabilities, StreamRequest> stream_config_requests_;

  std::map<Stream, stream_datas_t> stream_datas_map_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_STREAMS_H_ NOLINT
