#ifndef MYNTEYE_INTERNAL_CHANNELS_H_  // NOLINT
#define MYNTEYE_INTERNAL_CHANNELS_H_
#pragma once

#include <map>
#include <memory>
#include <thread>

#include "mynteye/mynteye.h"
#include "mynteye/types.h"

#include "internal/types.h"
#include "uvc/uvc.h"

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

struct device;
struct xu;

}  // namespace uvc

class MYNTEYE_API Channels {
 public:
  typedef enum Channel {
    CHANNEL_CAM_CTRL = 1,
    CHANNEL_HALF_DUPLEX = 2,
    CHANNEL_IMU_WRITE = 3,
    CHANNEL_IMU_READ = 4,
    CHANNEL_FILE = 5,
    CHANNEL_LAST
  } channel_t;

  typedef struct ControlInfo {
    std::int32_t min;
    std::int32_t max;
    std::int32_t def;
  } control_info_t;

  typedef enum XuCmd {
    XU_CMD_ZDC = 0xE6,    // zero drift calibration
    XU_CMD_ERASE = 0xDE,  // erase chip
    XU_CMD_LAST
  } xu_cmd_t;

  typedef enum FileId {
    FID_DEVICE_INFO = 1,  // device info
    FID_IMG_PARAMS = 2,   // image intrinsics & extrinsics
    FID_IMU_PARAMS = 4,   // imu intrinsics & extrinsics
    FID_LAST,
  } file_id_t;

  using imu_callback_t = std::function<void(const ImuPacket &packet)>;

  using device_info_t = DeviceInfo;

  typedef struct ImgParams {
    bool ok;
    Intrinsics in_left;
    Intrinsics in_right;
    Extrinsics ex_left_to_right;
  } img_params_t;

  typedef struct ImuParams {
    bool ok;
    ImuIntrinsics in_accel;
    ImuIntrinsics in_gyro;
    Extrinsics ex_left_to_imu;
  } imu_params_t;

  explicit Channels(std::shared_ptr<uvc::device> device);
  ~Channels();

  void LogControlInfos() const;
  void UpdateControlInfos();
  control_info_t GetControlInfo(const Option &option) const;

  std::int32_t GetControlValue(const Option &option) const;
  void SetControlValue(const Option &option, std::int32_t value);

  bool RunControlAction(const Option &option) const;

  void SetImuCallback(imu_callback_t callback);
  void StartImuTracking(imu_callback_t callback = nullptr);
  void StopImuTracking();

  bool GetFiles(
      device_info_t *info, img_params_t *img_params, imu_params_t *imu_params,
      Version *spec_version = nullptr) const;
  bool SetFiles(
      device_info_t *info, img_params_t *img_params, imu_params_t *imu_params,
      Version *spec_version = nullptr);

 private:
  bool PuControlRange(
      Option option, int32_t *min, int32_t *max, int32_t *def) const;
  bool PuControlQuery(Option option, uvc::pu_query query, int32_t *value) const;

  bool XuControlRange(
      channel_t channel, uint8_t id, int32_t *min, int32_t *max, int32_t *def) const;
  bool XuControlRange(
      const uvc::xu &xu, uint8_t selector, uint8_t id, int32_t *min, int32_t *max,
      int32_t *def) const;

  bool XuControlQuery(
      channel_t channel, uvc::xu_query query, uint16_t size,
      uint8_t *data) const;
  bool XuControlQuery(
      const uvc::xu &xu, uint8_t selector, uvc::xu_query query, uint16_t size,
      uint8_t *data) const;

  bool XuCamCtrlQuery(uvc::xu_query query, uint16_t size, uint8_t *data) const;
  std::int32_t XuCamCtrlGet(Option option) const;
  void XuCamCtrlSet(Option option, std::int32_t value) const;

  bool XuHalfDuplexSet(Option option, xu_cmd_t cmd) const;

  bool XuImuWrite(const ImuReqPacket &req) const;
  bool XuImuRead(ImuResPacket *res) const;

  bool XuFileQuery(uvc::xu_query query, uint16_t size, uint8_t *data) const;

  control_info_t PuControlInfo(Option option) const;
  control_info_t XuControlInfo(Option option) const;

  std::shared_ptr<uvc::device> device_;

  std::map<Option, control_info_t> control_infos_;

  bool is_imu_tracking_;
  std::thread imu_track_thread_;
  volatile bool imu_track_stop_;

  std::uint32_t imu_sn_;
  imu_callback_t imu_callback_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_CHANNELS_H_ NOLINT
