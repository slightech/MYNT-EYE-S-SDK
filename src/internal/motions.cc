#include "internal/motions.h"

#include <glog/logging.h>

#include "internal/channels.h"

MYNTEYE_BEGIN_NAMESPACE

Motions::Motions(std::shared_ptr<Channels> channels)
    : channels_(channels),
      motion_callback_(nullptr),
      motion_datas_enabled_(false),
      is_imu_tracking(false) {
  CHECK_NOTNULL(channels_);
  VLOG(2) << __func__;
}

Motions::~Motions() {
  VLOG(2) << __func__;
}

void Motions::SetMotionCallback(motion_callback_t callback) {
  motion_callback_ = callback;
}

void Motions::StartMotionTracking() {
  if (!is_imu_tracking) {
    channels_->StartImuTracking([this](const ImuPacket &packet) {
      if (!motion_callback_ && !motion_datas_enabled_) {
        LOG(WARNING) << "";
        return;
      }
      for (auto &&seg : packet.segments) {
        auto &&imu = std::make_shared<ImuData>();
        imu->frame_id = seg.frame_id;
        if (seg.offset < 0 &&
            static_cast<uint32_t>(-seg.offset) > packet.timestamp) {
          LOG(WARNING) << "Imu timestamp offset is incorrect";
        }
        imu->timestamp = packet.timestamp + seg.offset;
        imu->accel[0] = seg.accel[0] * 8.f / 0x10000;
        imu->accel[1] = seg.accel[1] * 8.f / 0x10000;
        imu->accel[2] = seg.accel[2] * 8.f / 0x10000;
        imu->gyro[0] = seg.gyro[0] * 1000.f / 0x10000;
        imu->gyro[1] = seg.gyro[1] * 1000.f / 0x10000;
        imu->gyro[2] = seg.gyro[2] * 1000.f / 0x10000;
        imu->temperature = seg.temperature / 326.8f + 25;
        if (motion_callback_) {
          motion_callback_({imu});
        }
      }
    });
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

void Motions::EnableMotionDatas(std::size_t max_size) {
  if (max_size <= 0) {
    LOG(WARNING) << "Could not enable motion datas with max_size <= 0";
    return;
  }
  motion_datas_enabled_ = true;
  motion_datas_max_size = max_size;
}

Motions::motion_datas_t Motions::GetMotionDatas() {
  if (!motion_datas_enabled_) {
    LOG(FATAL) << "Must enable motion datas before getting them, or you set "
                  "motion callback instead";
  }
  return motion_datas_;
}

MYNTEYE_END_NAMESPACE
