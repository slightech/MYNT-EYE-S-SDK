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
#include "mynteye/device/motions.h"

#include "mynteye/logger.h"
#include "mynteye/device/channel/channels.h"

MYNTEYE_BEGIN_NAMESPACE

namespace {

void matrix_3x1(const double (*src1)[3], const double (*src2)[1],
    double (*dst)[1]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 1; j++) {
      for (int k = 0; k < 3; k++) {
        dst[i][j] += src1[i][k] * src2[k][j];
      }
    }
  }
}

void matrix_3x3(const double (*src1)[3], const double (*src2)[3],
    double (*dst)[3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        dst[i][j] += src1[i][k] * src2[k][j];
      }
    }
  }
}

} // namespace

Motions::Motions(std::shared_ptr<Channels> channels)
    : channels_(channels),
      motion_callback_(nullptr),
      motion_datas_enabled_(false),
      is_imu_tracking(false),
      proc_mode_(static_cast<const std::int32_t>(ProcessMode::PROC_NONE)),
      motion_intrinsics_(nullptr) {
  CHECK_NOTNULL(channels_);
  VLOG(2) << __func__;
}

Motions::~Motions() {
  VLOG(2) << __func__;
}

void Motions::SetMotionCallback(motion_callback_t callback) {
  motion_callback_ = callback;
  if (motion_callback_) {
    accel_range = channels_->GetControlValue(Option::ACCELEROMETER_RANGE);
    if (accel_range == -1)
      accel_range = channels_->GetAccelRangeDefault();

    gyro_range = channels_->GetControlValue(Option::GYROSCOPE_RANGE);
    if (gyro_range == -1)
      gyro_range = channels_->GetGyroRangeDefault();
    channels_->SetAcceRange(accel_range);
    channels_->SetGyroRange(gyro_range);

    channels_->SetImuCallback([this](const ImuPacket2 &packet) {
      if (!motion_callback_ && !motion_datas_enabled_) {
        return;
      }
      for (auto &&seg : packet.segments) {
        auto &&imu = std::make_shared<ImuData>();
        // imu->frame_id = seg.frame_id;
        // if (seg.offset < 0 &&
        //     static_cast<uint32_t>(-seg.offset) > packet.timestamp) {
        //   LOG(WARNING) << "Imu timestamp offset is incorrect";
        // }
        imu->frame_id = seg.frame_id;
        imu->timestamp = seg.timestamp;
        imu->flag = seg.flag;
        imu->is_ets = seg.is_ets;
        // imu->temperature = seg.temperature / 326.8f + 25;
        imu->temperature = seg.temperature;
        // LOG(INFO) << "beforea" << seg.accel[0];
        // LOG(INFO) << "before" << seg.accel[1];
        // LOG(INFO) << "before" << seg.accel[2];
        // LOG(INFO) << "beforeg" << seg.gyro[0];
        // LOG(INFO) << "before" << seg.gyro[1];
        // LOG(INFO) << "before" << seg.gyro[2];
        // imu->accel[0] = seg.accel[0] * 1.f * accel_range / 0x10000;
        // imu->accel[1] = seg.accel[1] * 1.f * accel_range / 0x10000;
        // imu->accel[2] = seg.accel[2] * 1.f * accel_range / 0x10000;
        // imu->gyro[0] = seg.gyro[0] * 1.f * gyro_range / 0x10000;
        // imu->gyro[1] = seg.gyro[1] * 1.f * gyro_range / 0x10000;
        // imu->gyro[2] = seg.gyro[2] * 1.f * gyro_range / 0x10000;

        imu->accel[0] = seg.accel[0];
        imu->accel[1] = seg.accel[1];
        imu->accel[2] = seg.accel[2];
        imu->gyro[0] = seg.gyro[0];
        imu->gyro[1] = seg.gyro[1];
        imu->gyro[2] = seg.gyro[2];

        // LOG(INFO)<< "aftera" << imu->accel[0];
        // LOG(INFO)<< "after" << imu->accel[1];
        // LOG(INFO)<< "after" << imu->accel[2];
        // LOG(INFO)<< "afterg" << imu->gyro[0];
        // LOG(INFO)<< "afterg" << imu->gyro[1];
        // LOG(INFO)<< "afterg" << imu->gyro[2];

        bool proc_assembly =((proc_mode_ & ProcessMode::PROC_IMU_ASSEMBLY) > 0);
        bool proc_temp_drift =
            ((proc_mode_ & ProcessMode::PROC_IMU_TEMP_DRIFT) > 0);
        if (channels_ && !channels_->IsImuProtocol2()) {
          if (proc_assembly && proc_temp_drift) {
            ProcImuTempDrift(imu);
            ProcImuAssembly(imu);
          } else if (proc_assembly) {
            ProcImuAssembly(imu);
          } else if (proc_temp_drift) {
            ProcImuTempDrift(imu);
          }
        }

        std::lock_guard<std::mutex> _(mtx_datas_);
        motion_data_t data = {imu};
        if (motion_datas_enabled_ && motion_datas_max_size_ > 0) {
          if (motion_datas_.size() >= motion_datas_max_size_) {
            motion_datas_.erase(motion_datas_.begin());
          }
          motion_datas_.push_back(data);
        }

        motion_callback_(data);
      }
    });
  } else {
    channels_->SetImuCallback(nullptr);
  }
}

void Motions::DoMotionTrack() {
  channels_->DoImuTrack();
}

void Motions::StartMotionTracking() {
  if (!is_imu_tracking) {
    channels_->StartImuTracking();
    is_imu_tracking = true;
  } else {
    LOG(WARNING) << "Imu is tracking already";
  }
}

void Motions::StopMotionTracking() {
  if (is_imu_tracking) {
    channels_->StopImuTracking();
    is_imu_tracking = false;
  }
}

void Motions::DisableMotionDatas() {
  std::lock_guard<std::mutex> _(mtx_datas_);
  motion_datas_enabled_ = false;
  motion_datas_max_size_ = 0;
  motion_datas_.clear();
}

void Motions::EnableMotionDatas(std::size_t max_size) {
  if (max_size <= 0) {
    LOG(WARNING) << "Could not enable motion datas with max_size <= 0";
    return;
  }
  std::lock_guard<std::mutex> _(mtx_datas_);
  motion_datas_enabled_ = true;
  motion_datas_max_size_ = max_size;
}

Motions::motion_datas_t Motions::GetMotionDatas() {
  if (!motion_datas_enabled_) {
    LOG(FATAL) << "Must enable motion datas before getting them, or you set "
                  "motion callback instead";
  }
  std::lock_guard<std::mutex> _(mtx_datas_);
  motion_datas_t datas = motion_datas_;
  motion_datas_.clear();
  return datas;
}

void Motions::ProcImuAssembly(std::shared_ptr<ImuData> data) const {
  if (nullptr == motion_intrinsics_ ||
      IsNullAssemblyOrTempDrift())
    return;

  double dst[3][3] = {0};
  if (data->flag == 1) {
    matrix_3x3(motion_intrinsics_->accel.scale,
        motion_intrinsics_->accel.assembly, dst);
    double s[3][1] = {0};
    double d[3][1] = {0};
    for (int i = 0; i < 3; i++) {
      s[i][0] = data->accel[i];
    }
    matrix_3x1(dst, s, d);
    for (int i = 0; i < 3; i++) {
      data->accel[i] = d[i][0];
    }
  } else if (data->flag == 2) {
    matrix_3x3(motion_intrinsics_->gyro.scale,
        motion_intrinsics_->gyro.assembly, dst);
    double s[3][1] = {0};
    double d[3][1] = {0};
    for (int i = 0; i < 3; i++) {
      s[i][0] = data->gyro[i];
    }
    matrix_3x1(dst, s, d);
    for (int i = 0; i < 3; i++) {
      data->gyro[i] = d[i][0];
    }
  }
}

void Motions::ProcImuTempDrift(std::shared_ptr<ImuData> data) const {
  if (nullptr == motion_intrinsics_ ||
      IsNullAssemblyOrTempDrift())
    return;

  double temp = data->temperature;
  if (data->flag == 1) {
    data->accel[0] -= motion_intrinsics_->accel.x[1] * temp
      + motion_intrinsics_->accel.x[0];
    data->accel[1] -= motion_intrinsics_->accel.y[1] * temp
      + motion_intrinsics_->accel.y[0];
    data->accel[2] -= motion_intrinsics_->accel.z[1] * temp
      + motion_intrinsics_->accel.z[0];
  } else if (data->flag == 2) {
    data->gyro[0] -= motion_intrinsics_->gyro.x[1] * temp
      + motion_intrinsics_->gyro.x[0];
    data->gyro[1] -= motion_intrinsics_->gyro.y[1] * temp
      + motion_intrinsics_->gyro.y[0];
    data->gyro[2] -= motion_intrinsics_->gyro.z[1] * temp
      + motion_intrinsics_->gyro.z[0];
  }
}

void Motions::SetMotionIntrinsics(const std::shared_ptr<MotionIntrinsics>& in) {
  motion_intrinsics_ = in;
}

void Motions::EnableProcessMode(const std::int32_t& mode) {
  proc_mode_ = mode;
}

bool Motions::IsNullAssemblyOrTempDrift() const {
  if (!device_info_)
    return true;

  if (device_info_->spec_version >= Version(1, 2))
    return false;

  return true;
}

void Motions::SetDeviceInfo(const std::shared_ptr<DeviceInfo>& in) {
  device_info_ = in;
}

MYNTEYE_END_NAMESPACE
