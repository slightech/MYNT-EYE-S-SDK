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
#include "mynteye/device/config.h"

MYNTEYE_BEGIN_NAMESPACE

const std::map<Model, StreamSupports> stream_supports_map = {
  {Model::STANDARD, {Stream::LEFT, Stream::RIGHT}},
  {Model::STANDARD2, {Stream::LEFT, Stream::RIGHT}},
  {Model::STANDARD210A, {Stream::LEFT, Stream::RIGHT}},
  {Model::STANDARD200B, {Stream::LEFT, Stream::RIGHT}},
};

const std::map<Model, CapabilitiesSupports> capabilities_supports_map = {
  {Model::STANDARD, {Capabilities::STEREO, Capabilities::IMU}},
  {Model::STANDARD2, {Capabilities::STEREO_COLOR, Capabilities::IMU}},
  {Model::STANDARD210A, {Capabilities::STEREO_COLOR, Capabilities::IMU}},
  {Model::STANDARD200B, {Capabilities::STEREO_COLOR, Capabilities::IMU}}
};

const std::map<Model, OptionSupports> option_supports_map = {
  {Model::STANDARD, {
    Option::GAIN, Option::BRIGHTNESS, Option::CONTRAST,
    Option::FRAME_RATE, Option::IMU_FREQUENCY,
    Option::EXPOSURE_MODE, Option::MAX_GAIN, Option::MAX_EXPOSURE_TIME,
      Option::DESIRED_BRIGHTNESS,
    Option::IR_CONTROL,
    Option::HDR_MODE,
    Option::ACCELEROMETER_RANGE, Option::GYROSCOPE_RANGE,
    Option::ZERO_DRIFT_CALIBRATION,
    Option::ERASE_CHIP}
  },
  {Model::STANDARD2, {
    Option::BRIGHTNESS,
    Option::EXPOSURE_MODE, Option::MAX_GAIN, Option::MAX_EXPOSURE_TIME,
      Option::MIN_EXPOSURE_TIME, Option::DESIRED_BRIGHTNESS, Option::ACCELEROMETER_RANGE,
    Option::GYROSCOPE_RANGE, Option::ACCELEROMETER_LOW_PASS_FILTER,
    Option::GYROSCOPE_LOW_PASS_FILTER, Option::IIC_ADDRESS_SETTING,
    Option::ERASE_CHIP}
  },
  {Model::STANDARD210A, {
    Option::BRIGHTNESS,
    Option::EXPOSURE_MODE, Option::MAX_GAIN, Option::MAX_EXPOSURE_TIME,
      Option::MIN_EXPOSURE_TIME, Option::DESIRED_BRIGHTNESS,
    Option::ACCELEROMETER_RANGE, Option::GYROSCOPE_RANGE,
    Option::ACCELEROMETER_LOW_PASS_FILTER, Option::GYROSCOPE_LOW_PASS_FILTER,
    Option::IIC_ADDRESS_SETTING, Option::ERASE_CHIP}
  },
  {Model::STANDARD200B, {
    Option::BRIGHTNESS,
    Option::EXPOSURE_MODE, Option::MAX_GAIN, Option::MAX_EXPOSURE_TIME,
      Option::MIN_EXPOSURE_TIME, Option::DESIRED_BRIGHTNESS, Option::ACCELEROMETER_RANGE,
    Option::GYROSCOPE_RANGE, Option::ACCELEROMETER_LOW_PASS_FILTER,
    Option::GYROSCOPE_LOW_PASS_FILTER, Option::ERASE_CHIP, Option::SYNC_TIMESTAMP}
  },
};

const std::map<Model, std::map<Capabilities, StreamRequests>>
stream_requests_map = {
  {Model::STANDARD,
    {{Capabilities::STEREO, {
      {752, 480, Format::YUYV, 60},
      {376, 240, Format::YUYV, 60}}
    }}
  },
  {Model::STANDARD2,
    {{Capabilities::STEREO_COLOR, {
      {1280, 400, Format::YUYV, 10},
      {1280, 400, Format::YUYV, 20},
      {1280, 400, Format::YUYV, 30},
      {1280, 400, Format::YUYV, 60},
      {2560, 800, Format::YUYV, 10},
      {2560, 800, Format::YUYV, 20},
      {2560, 800, Format::YUYV, 30}}
    }}
  },
  {Model::STANDARD210A,
    {{Capabilities::STEREO_COLOR, {
      {1280, 400, Format::BGR888, 10},
      {1280, 400, Format::BGR888, 20},
      {1280, 400, Format::BGR888, 30},
      {1280, 400, Format::BGR888, 60},
      {2560, 800, Format::BGR888, 10},
      {2560, 800, Format::BGR888, 20},
      {2560, 800, Format::BGR888, 30}}
    }}
  },
  {Model::STANDARD200B,
    {{Capabilities::STEREO_COLOR, {
      {1280, 400, Format::YUYV, 10},
      {1280, 400, Format::YUYV, 20},
      {1280, 400, Format::YUYV, 30},
      {1280, 400, Format::YUYV, 60},
      {2560, 800, Format::YUYV, 10},
      {2560, 800, Format::YUYV, 20},
      {2560, 800, Format::YUYV, 30}}
    }}
  },
};

/**
 * default intrinsics
 */

std::shared_ptr<IntrinsicsBase> getDefaultIntrinsics() {
  auto res = std::make_shared<IntrinsicsPinhole>();
  res->width = 640;
  res->height = 400;
  res->model = 0;
  res->fx = 3.6220059643202876e+02;
  res->fy = 3.6350065250745848e+02;
  res->cx = 4.0658699068023441e+02;
  res->cy = 2.3435161110061483e+02;
  double codffs[5] = {
    -2.5034765682756088e-01,
    5.0579399202897619e-02,
    -7.0536676161976066e-04,
    -8.5255451307033846e-03,
    0.
  };
  for (unsigned int i = 0; i < 5; i++) {
    res->coeffs[i] = codffs[i];
  }
  return res;
}

std::shared_ptr<IntrinsicsBase> getDefaultIntrinsics(const Resolution &resolution) {
  auto res = getDefaultIntrinsics();
  res->resize_scale = static_cast<double>(resolution.width / res->width);
  res->ResizeIntrinsics();
  return res;
}

std::shared_ptr<Extrinsics> getDefaultExtrinsics() {
  auto res = std::make_shared<Extrinsics>();
  double rotation[9] = {
    9.9867908939669447e-01,  -6.3445566137485428e-03, 5.0988459509619687e-02,
    5.9890316389333252e-03,  9.9995670037792639e-01,  7.1224201868366971e-03,
    -5.1031440326695092e-02, -6.8076406092671274e-03, 9.9867384471984544e-01
  };
  double translation[3] = {-1.2002489764113250e+02, -1.1782637409050747e+00,
      -5.2058205159996538e+00};
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      res->rotation[i][j] = rotation[i*3 + j];
    }
  }
  for (unsigned int i = 0; i < 3; i++) {
    res->translation[i] = translation[i];
  }
  return res;
}

MYNTEYE_END_NAMESPACE
