#ifndef MYNTEYE_SYNTHETIC_H_  // NOLINT
#define MYNTEYE_SYNTHETIC_H_
#pragma once

#include <map>
#include <memory>
#include <vector>

#include "api/api.h"

MYNTEYE_BEGIN_NAMESPACE

class API;
class Processor;

class Synthetic {
 public:
  using stream_callback_t = API::stream_callback_t;

  typedef enum Mode {
    MODE_NATIVE,     // Native stream
    MODE_SYNTHETIC,  // Synthetic stream
    MODE_LAST        // Unsupported
  } mode_t;

  explicit Synthetic(API *api);
  ~Synthetic();

  mode_t GetMode(const Stream &stream) const;
  bool Supports(const Stream &stream) const;

  void EnableStreamData(const Stream &stream);
  void DisableStreamData(const Stream &stream);

  void SetStreamCallback(const Stream &stream, stream_callback_t callback);
  bool HasStreamCallback(const Stream &stream) const;

  void StartVideoStreaming();
  void StopVideoStreaming();

  void WaitForStreams();

  api::StreamData GetStreamData(const Stream &stream);
  std::vector<api::StreamData> GetStreamDatas(const Stream &stream);

 private:
  void InitStreamSupports();

  API *api_;

  std::map<Stream, mode_t> stream_supports_mode_;

  std::vector<std::shared_ptr<Processor>> processors_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_SYNTHETIC_H_ NOLINT
