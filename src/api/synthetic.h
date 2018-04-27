#ifndef MYNTEYE_SYNTHETIC_H_  // NOLINT
#define MYNTEYE_SYNTHETIC_H_
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "api/api.h"

MYNTEYE_BEGIN_NAMESPACE

class API;
class Object;
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
  bool IsStreamDataEnabled(const Stream &stream) const;

  void SetStreamCallback(const Stream &stream, stream_callback_t callback);
  bool HasStreamCallback(const Stream &stream) const;

  void StartVideoStreaming();
  void StopVideoStreaming();

  void WaitForStreams();

  api::StreamData GetStreamData(const Stream &stream);
  std::vector<api::StreamData> GetStreamDatas(const Stream &stream);

 private:
  void InitStreamSupports();

  void EnableStreamData(const Stream &stream, std::uint32_t depth);
  void DisableStreamData(const Stream &stream, std::uint32_t depth);

  void InitProcessors();

  template <class T>
  bool ActivateProcessor(bool tree = false);
  template <class T>
  bool DeactivateProcessor(bool tree = false);

  bool OnRectifyProcess(
      Object *const in, Object *const out, Processor *const parent);
  bool OnDisparityProcess(
      Object *const in, Object *const out, Processor *const parent);
  bool OnDisparityNormalizedProcess(
      Object *const in, Object *const out, Processor *const parent);
  bool OnPointsProcess(
      Object *const in, Object *const out, Processor *const parent);
  bool OnDepthProcess(
      Object *const in, Object *const out, Processor *const parent);

  API *api_;

  std::map<Stream, mode_t> stream_supports_mode_;

  std::map<Stream, stream_callback_t> stream_callbacks_;

  std::shared_ptr<Processor> processor_;
};

template <class T, class P>
std::shared_ptr<T> find_processor(const P &processor) {
  if (processor->Name() == T::NAME) {
    return std::dynamic_pointer_cast<T>(processor);
  }
  auto &&childs = processor->GetChilds();
  return find_processor<T>(std::begin(childs), std::end(childs), T::NAME);
}

template <class T, class InputIt>
std::shared_ptr<T> find_processor(
    InputIt first, InputIt last, const std::string &name) {
  if (first == last)
    return nullptr;
  for (auto it = first; it != last; ++it) {
    if ((*it)->Name() == name) {
      return std::dynamic_pointer_cast<T>(*it);
    }
  }
  for (auto it = first; it != last; ++it) {
    auto &&childs = (*it)->GetChilds();
    if (childs.empty())
      continue;
    auto &&result =
        find_processor<T>(std::begin(childs), std::end(childs), name);
    if (result == nullptr)
      continue;
    return result;
  }
  return nullptr;
}

template <class T>
bool Synthetic::ActivateProcessor(bool parents) {
  auto &&processor = find_processor<T>(processor_);
  if (processor == nullptr)
    return false;
  processor->Activate(parents);
  return true;
}

template <class T>
bool Synthetic::DeactivateProcessor(bool childs) {
  auto &&processor = find_processor<T>(processor_);
  if (processor == nullptr)
    return false;
  processor->Deactivate(childs);
  return true;
}

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_SYNTHETIC_H_ NOLINT
