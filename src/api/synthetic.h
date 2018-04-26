#ifndef MYNTEYE_SYNTHETIC_H_  // NOLINT
#define MYNTEYE_SYNTHETIC_H_
#pragma once

#include <memory>
#include <vector>

#include "api/api.h"

MYNTEYE_BEGIN_NAMESPACE

class API;
class Processor;

class Synthetic {
 public:
  using stream_callback_t = API::stream_callback_t;

  explicit Synthetic(API *api);
  ~Synthetic();

  bool Supports(const Stream &stream) const;

  void SetStreamCallback(const Stream &stream, stream_callback_t callback);
  bool HasStreamCallback(const Stream &stream) const;

  void StartVideoStreaming();
  void StopVideoStreaming();

  void WaitForStreams();

  api::StreamData GetStreamData(const Stream &stream);
  std::vector<api::StreamData> GetStreamDatas(const Stream &stream);

 private:
  API *api_;

  std::vector<std::shared_ptr<Processor>> processors_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_SYNTHETIC_H_ NOLINT
