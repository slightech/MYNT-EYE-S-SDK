#ifndef MYNTEYE_INTERNAL_MOTIONS_H_  // NOLINT
#define MYNTEYE_INTERNAL_MOTIONS_H_
#pragma once

#include <memory>
#include <vector>

#include "mynteye/mynteye.h"

#include "internal/callbacks.h"

MYNTEYE_BEGIN_NAMESPACE

class Channels;

class Motions {
 public:
  using motion_data_t = device::MotionData;
  using motion_datas_t = std::vector<motion_data_t>;

  using motion_callback_t = device::MotionCallback;

  explicit Motions(std::shared_ptr<Channels> channels);
  ~Motions();

  void SetMotionCallback(motion_callback_t callback);

  void StartMotionTracking();
  void StopMotionTracking();

  void EnableMotionDatas(std::size_t max_size);
  motion_datas_t GetMotionDatas();

 private:
  std::shared_ptr<Channels> channels_;

  motion_callback_t motion_callback_;

  motion_datas_t motion_datas_;
  bool motion_datas_enabled_;
  std::size_t motion_datas_max_size;

  bool is_imu_tracking;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_MOTIONS_H_ NOLINT
