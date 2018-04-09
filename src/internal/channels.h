#ifndef MYNTEYE_INTERNAL_CHANNELS_H_  // NOLINT
#define MYNTEYE_INTERNAL_CHANNELS_H_
#pragma once

#include <map>
#include <memory>

#include "mynteye/mynteye.h"
#include "uvc/uvc.h"

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

struct device;
struct xu;

}  // namespace uvc

class Channels {
 public:
  typedef enum Channel {
    CHANNEL_CAM_CTRL = 0x0100,
    CHANNEL_HALF_DUPLEX = 0x0200,
    CHANNEL_IMU_WRITE = 0x0300,
    CHANNEL_IMU_READ = 0x0400,
    CHANNEL_FILE = 0x0500,
    CHANNEL_LAST
  } channel_t;

  typedef struct ControlInfo {
    std::int32_t min;
    std::int32_t max;
    std::int32_t def;
  } control_info_t;

  explicit Channels(std::shared_ptr<uvc::device> device);
  ~Channels();

 private:
  bool XuControlQuery(
      uint8_t selector, uvc::xu_query query, uint16_t size, uint8_t *data);
  bool XuControlQuery(
      const uvc::xu &xu, uint8_t selector, uvc::xu_query query, uint16_t size,
      uint8_t *data);

  std::shared_ptr<uvc::device> device_;

  std::map<Option, ControlInfo> control_infos_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_CHANNELS_H_ NOLINT
