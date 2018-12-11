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
    {Model::STANDARD, {Stream::LEFT, Stream::RIGHT}}};

const std::map<Model, CapabilitiesSupports> capabilities_supports_map = {
    {Model::STANDARD, {Capabilities::STEREO, Capabilities::IMU}}};

const std::map<Model, OptionSupports> option_supports_map = {
    {Model::STANDARD,
     {Option::GAIN, Option::BRIGHTNESS, Option::CONTRAST, Option::FRAME_RATE,
      Option::IMU_FREQUENCY, Option::EXPOSURE_MODE, Option::MAX_GAIN,
      Option::MAX_EXPOSURE_TIME, Option::DESIRED_BRIGHTNESS, Option::IR_CONTROL,
      Option::HDR_MODE, Option::ZERO_DRIFT_CALIBRATION, Option::ERASE_CHIP,
      Option::ACCELEROMETER_RANGE, Option::GYROSCOPE_RANGE}}};

const std::map<Model, std::map<Capabilities, StreamRequests>>
    stream_requests_map = {
        {Model::STANDARD,
         {{Capabilities::STEREO, {{752, 480, Format::YUYV, 25}}}}}};

MYNTEYE_END_NAMESPACE
