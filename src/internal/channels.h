#ifndef MYNTEYE_INTERNAL_CHANNELS_H_  // NOLINT
#define MYNTEYE_INTERNAL_CHANNELS_H_
#pragma once

#include <memory>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

struct device;
struct xu;

}  // namespace uvc

class Channels {
 public:
  typedef enum Query {
    SET_CUR,
    GET_CUR,
    GET_MIN,
    GET_MAX,
    GET_DEF,
    LAST
  } query_t;

  explicit Channels(std::shared_ptr<uvc::device> device);
  ~Channels();

 private:
  bool ControlQuery(
      const uvc::xu &xu, uint8_t selector, const query_t &query, uint16_t size,
      uint8_t *data);

  std::shared_ptr<uvc::device> device_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_CHANNELS_H_ NOLINT
