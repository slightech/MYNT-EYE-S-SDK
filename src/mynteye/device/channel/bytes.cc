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
#include "mynteye/device/channel/bytes.h"

#include "mynteye/logger.h"
#include "mynteye/util/strings.h"

MYNTEYE_BEGIN_NAMESPACE

namespace bytes {

// from

std::string _from_data(const std::uint8_t *data, std::size_t count) {
  std::string s(reinterpret_cast<const char *>(data), count);
  strings::trim(s);
  return s;
}

// from types

std::size_t from_data(IntrinsicsBase *in, const std::uint8_t *data,
    bool get_size) {
  switch (in->calib_model()) {
    case CalibrationModel::PINHOLE:
      return from_data(dynamic_cast<IntrinsicsPinhole *>(in), data,
          get_size);
    case CalibrationModel::KANNALA_BRANDT:
      return from_data(dynamic_cast<IntrinsicsEquidistant *>(in), data,
          get_size);
    default:
      LOG(FATAL) << "Unknown calib model: " << in->calib_model();
  }
}

std::size_t from_data(IntrinsicsPinhole *in, const std::uint8_t *data,
    bool get_size) {
  std::size_t i = 0;

  if (get_size) {
    // width, 2
    in->width = _from_data<std::uint16_t>(data + i);
    i += 2;
    // height, 2
    in->height = _from_data<std::uint16_t>(data + i);
    i += 2;
  }
  // fx, 8
  in->fx = _from_data<double>(data + i);
  i += 8;
  // fy, 8
  in->fy = _from_data<double>(data + i);
  i += 8;
  // cx, 8
  in->cx = _from_data<double>(data + i);
  i += 8;
  // cy, 8
  in->cy = _from_data<double>(data + i);
  i += 8;
  if (get_size) {
    // model, 1
    in->model = data[i];
    i += 1;
  }
  // coeffs, 40
  for (std::size_t j = 0; j < 5; j++) {
    in->coeffs[j] = _from_data<double>(data + i + j * 8);
  }
  i += 40;

  return i;
}

std::size_t from_data(IntrinsicsEquidistant *in, const std::uint8_t *data,
    bool get_size) {
  std::size_t i = 0;

  if (get_size) {
    // width, 2
    in->width = _from_data<std::uint16_t>(data + i);
    i += 2;
    // height, 2
    in->height = _from_data<std::uint16_t>(data + i);
    i += 2;
  }
  // coeffs, 64
  for (std::size_t j = 0; j < 8; j++) {
    in->coeffs[j] = _from_data<double>(data + i + j * 8);
  }
  i += 64;

  return i;
}

std::size_t from_data(ImuIntrinsics *in, const std::uint8_t *data,
    bool get_size) {
  std::size_t i = 0;

  // scale
  for (std::size_t j = 0; j < 3; j++) {
    for (std::size_t k = 0; k < 3; k++) {
      in->scale[j][k] = _from_data<double>(data + i + (j * 3 + k) * 8);
    }
  }
  i += 72;
  if (get_size) {
    // assembly
    for (std::size_t j = 0; j < 3; j++) {
      for (std::size_t k = 0; k < 3; k++) {
        in->assembly[j][k] = _from_data<double>(data + i + (j * 3 + k) * 8);
      }
    }
    i += 72;
  } else {
    // assembly
    for (std::size_t j = 0; j < 3; j++) {
      for (std::size_t k = 0; k < 3; k++) {
        in->assembly[j][k] = 0.0;
      }
    }
  }
  // drift
  for (std::size_t j = 0; j < 3; j++) {
    in->drift[j] = _from_data<double>(data + i + j * 8);
  }
  i += 24;
  // noise
  for (std::size_t j = 0; j < 3; j++) {
    in->noise[j] = _from_data<double>(data + i + j * 8);
  }
  i += 24;
  // bias
  for (std::size_t j = 0; j < 3; j++) {
    in->bias[j] = _from_data<double>(data + i + j * 8);
  }
  i += 24;
  if (get_size) {
    // temperature drift
    // x
    for (std::size_t j = 0; j < 2; j++) {
      in->x[j] = _from_data<double>(data + i + j * 8);
    }
    i += 16;
    // y
    for (std::size_t j = 0; j < 2; j++) {
      in->y[j] = _from_data<double>(data + i + j * 8);
    }
    i += 16;
    // z
    for (std::size_t j = 0; j < 2; j++) {
      in->z[j] = _from_data<double>(data + i + j * 8);
    }
    i += 16;
  } else {
    // temperature drift
    // x
    for (std::size_t j = 0; j < 2; j++) {
      in->x[j] = 0.0;
    }
    // y
    for (std::size_t j = 0; j < 2; j++) {
      in->y[j] = 0.0;
    }
    // z
    for (std::size_t j = 0; j < 2; j++) {
      in->z[j] = 0.0;
    }
  }

  return i;
}

std::size_t from_data(Extrinsics *ex, const std::uint8_t *data) {
  std::size_t i = 0;

  // rotation
  for (std::size_t j = 0; j < 3; j++) {
    for (std::size_t k = 0; k < 3; k++) {
      ex->rotation[j][k] = _from_data<double>(data + i + (j * 3 + k) * 8);
    }
  }
  i += 72;
  // translation
  for (std::size_t j = 0; j < 3; j++) {
    ex->translation[j] = _from_data<double>(data + i + j * 8);
  }
  i += 24;

  return i;
}

// to

std::size_t _to_data(std::string value, std::uint8_t *data, std::size_t count) {
  std::copy(value.begin(), value.end(), data);
  for (std::size_t i = value.size(); i < count; i++) {
    data[i] = ' ';
  }
  return count;
}

// to types

std::size_t to_data(const IntrinsicsBase *in, std::uint8_t *data,
    bool set_size) {
  switch (in->calib_model()) {
    case CalibrationModel::PINHOLE:
      return to_data(dynamic_cast<const IntrinsicsPinhole *>(in), data,
          set_size);
    case CalibrationModel::KANNALA_BRANDT:
      return to_data(dynamic_cast<const IntrinsicsEquidistant *>(in), data,
          set_size);
    default:
      LOG(FATAL) << "Unknown calib model: " << in->calib_model();
  }
}

std::size_t to_data(const IntrinsicsPinhole *in, std::uint8_t *data,
    bool set_size) {
  std::size_t i = 0;

  if (set_size) {
    // width, 2
    _to_data(in->width, data + i);
    i += 2;
    // height, 2
    _to_data(in->height, data + i);
    i += 2;
  }
  // fx, 8
  _to_data(in->fx, data + i);
  i += 8;
  // fy, 8
  _to_data(in->fy, data + i);
  i += 8;
  // cx, 8
  _to_data(in->cx, data + i);
  i += 8;
  // cy, 8
  _to_data(in->cy, data + i);
  i += 8;
  if (set_size) {
    // model, 1
    data[i] = in->model;
    i += 1;
  }
  // coeffs, 40
  for (std::size_t j = 0; j < 5; j++) {
    _to_data(in->coeffs[j], data + i + j * 8);
  }
  i += 40;

  return i;
}

std::size_t to_data(const IntrinsicsEquidistant *in, std::uint8_t *data,
    bool set_size) {
  std::size_t i = 0;

  if (set_size) {
    // width, 2
    _to_data(in->width, data + i);
    i += 2;
    // height, 2
    _to_data(in->height, data + i);
    i += 2;
  }
  // coeffs, 64
  for (std::size_t j = 0; j < 8; j++) {
    _to_data(in->coeffs[j], data + i + j * 8);
  }
  i += 64;

  return i;
}

std::size_t to_data(const ImuIntrinsics *in, std::uint8_t *data,
    bool set_size) {
  std::size_t i = 0;

  // scale
  for (std::size_t j = 0; j < 3; j++) {
    for (std::size_t k = 0; k < 3; k++) {
      _to_data(in->scale[j][k], data + i + (j * 3 + k) * 8);
    }
  }
  i += 72;
  if (set_size) {
    // assembly
    for (std::size_t j = 0; j < 3; j++) {
      for (std::size_t k = 0; k < 3; k++) {
        _to_data(in->assembly[j][k], data + i + (j * 3 + k) * 8);
      }
    }
    i += 72;
  }
  // drift
  for (std::size_t j = 0; j < 3; j++) {
    _to_data(in->drift[j], data + i + j * 8);
  }
  i += 24;
  // noise
  for (std::size_t j = 0; j < 3; j++) {
    _to_data(in->noise[j], data + i + j * 8);
  }
  i += 24;
  // bias
  for (std::size_t j = 0; j < 3; j++) {
    _to_data(in->bias[j], data + i + j * 8);
  }
  i += 24;
  if (set_size) {
    // temperature drift
    // x
    for (std::size_t j = 0; j < 2; j++) {
      _to_data<double>(in->x[j], data + i + j * 8);
    }
    i += 16;
    // y
    for (std::size_t j = 0; j < 2; j++) {
      _to_data<double>(in->y[j], data + i + j * 8);
    }
    i += 16;
    // z
    for (std::size_t j = 0; j < 2; j++) {
      _to_data<double>(in->z[j], data + i + j * 8);
    }
    i += 16;
  }

  return i;
}

std::size_t to_data(const Extrinsics *ex, std::uint8_t *data) {
  std::size_t i = 0;

  // rotation
  for (std::size_t j = 0; j < 3; j++) {
    for (std::size_t k = 0; k < 3; k++) {
      _to_data(ex->rotation[j][k], data + i + (j * 3 + k) * 8);
    }
  }
  i += 72;
  // translation
  for (std::size_t j = 0; j < 3; j++) {
    _to_data(ex->translation[j], data + i + j * 8);
  }
  i += 24;

  return i;
}

}  // namespace bytes

MYNTEYE_END_NAMESPACE
