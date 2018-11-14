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
#ifndef MYNTEYE_DEVICE_UTILS_H_
#define MYNTEYE_DEVICE_UTILS_H_
#pragma once

#include <memory>
#include <string>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

/**
 * @defgroup utils Utiliities
 */

class Device;

namespace device {

/**
 * @ingroup utils
 *
 * Detecting MYNT EYE devices and prompt user to select one.
 *
 * @return the selected device, or `nullptr` if none.
 */
MYNTEYE_API std::shared_ptr<Device> select();

}  // namespace device

namespace utils {

/**
 * @ingroup utils
 *
 * Get real exposure time in ms from virtual value, according to its frame rate.
 *
 * @param frame_rate the frame rate of the device.
 * @param exposure_time the virtual exposure time.
 * @return the real exposure time in ms, or the virtual value if frame rate is
 * invalid.
 */
MYNTEYE_API float get_real_exposure_time(
    std::int32_t frame_rate, std::uint16_t exposure_time);

/**
 * @ingroup utils
 * 
 * Get sdk root dir.
 */
MYNTEYE_API std::string get_sdk_root_dir();

/**
 * @ingroup utils
 * 
 * Get sdk install dir.
 */
MYNTEYE_API std::string get_sdk_install_dir();

}  // namespace utils

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_UTILS_H_
