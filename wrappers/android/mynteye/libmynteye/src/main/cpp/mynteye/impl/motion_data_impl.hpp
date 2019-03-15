#pragma once

#include "mynteye/device/callbacks.h"

#include "imu_data.hpp"
#include "motion_data.hpp"

namespace mynteye_jni {

class MotionDataImpl : public MotionData {
 public:
  using motion_data_t = MYNTEYE_NAMESPACE::device::MotionData;

  explicit MotionDataImpl(const motion_data_t& data) : data_(data) {}
  ~MotionDataImpl() {}

  ImuData Imu() override {
    auto&& imu = data_.imu;
    return {
      imu->frame_id,
      imu->flag,
      static_cast<int64_t>(imu->timestamp),
      std::vector<double>(imu->accel, imu->accel + 3),
      std::vector<double>(imu->gyro, imu->gyro + 3),
      imu->temperature,
    };
  }

 private:
  motion_data_t data_;
};

}  // namespace mynteye_jni
