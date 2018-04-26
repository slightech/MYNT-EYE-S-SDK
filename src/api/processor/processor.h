#ifndef MYNTEYE_PROCESSOR_H_  // NOLINT
#define MYNTEYE_PROCESSOR_H_
#pragma once

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "mynteye/mynteye.h"

#include "api/processor/object.h"

MYNTEYE_BEGIN_NAMESPACE

class Processor /*: public std::enable_shared_from_this<Processor>*/ {
 public:
  using PreProcessCallback = std::function<void(Object *const)>;
  using PostProcessCallback = std::function<void(Object *const)>;
  using ProcessCallback = std::function<bool(
      Object *const in, Object *const out, Processor *const parent)>;

  Processor();
  virtual ~Processor();

  virtual std::string Name();

  void AddChild(const std::shared_ptr<Processor> &child);

  void RemoveChild(const std::shared_ptr<Processor> &child);

  std::list<std::shared_ptr<Processor>> GetChilds();

  void SetPreProcessCallback(PreProcessCallback callback);
  void SetPostProcessCallback(PostProcessCallback callback);
  void SetProcessCallback(ProcessCallback callback);

  void Activate(bool tree = false);
  void Deactivate(bool tree = false);
  bool IsActivated();

  bool IsIdle();

  /** Returns dropped or not. */
  bool Process(const Object *const in);

  /**
   * Returns the last output.
   * @note Returns null if not output now.
   */
  Object *GetOutput();

  std::uint64_t GetDroppedCount();

 protected:
  virtual Object *OnCreateOutput() = 0;
  virtual void OnProcess(
      Object *const in, Object *const out, Processor *const parent) = 0;

 private:
  /** Run in standalone thread. */
  void Run();

  void SetIdle(bool idle);

  bool activated_;

  bool input_ready_;
  std::mutex mtx_input_ready_;
  std::condition_variable cond_input_ready_;

  bool idle_;
  std::uint64_t dropped_count_;
  std::mutex mtx_state_;

  std::unique_ptr<Object> input_;
  std::unique_ptr<Object> output_;

  std::unique_ptr<Object> output_result_;
  std::mutex mtx_result_;

  PreProcessCallback pre_callback_;
  PostProcessCallback post_callback_;
  ProcessCallback callback_;

  Processor *parent_;
  std::list<std::shared_ptr<Processor>> childs_;

  std::thread thread_;
};

template <typename T>
void iterate_processors(
    const T &processors, std::function<void(std::shared_ptr<Processor>)> fn) {
  for (auto &&proc : processors) {
    fn(proc);
    iterate_processors(proc->GetChilds(), fn);
  }
}

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_PROCESSOR_H_  NOLINT
