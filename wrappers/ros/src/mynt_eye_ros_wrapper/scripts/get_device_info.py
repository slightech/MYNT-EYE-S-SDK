#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright 2018 Slightech Co., Ltd. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# pylint: disable=missing-docstring

from __future__ import print_function

import sys

import rospy

# pylint: disable=wildcard-import, unused-wildcard-import
from mynt_eye_ros_wrapper.srv import *


def get_device_info(**keys):
  if not keys:
    sys.exit('get_device_info without keys :(')

  rospy.wait_for_service('mynteye/get_info', 1)

  try:
    get_info = rospy.ServiceProxy('mynteye/get_info', GetInfo)

    result = {}
    for key_name, key_value in keys.items():
      result[key_name] = get_info(key_value).value

    return result
  except rospy.ServiceException as error:
    sys.exit('Failed to call service: {}'.format(error))


def main():
  keys = {
      'DEVICE_NAME': GetInfoRequest.DEVICE_NAME,
      'SERIAL_NUMBER': GetInfoRequest.SERIAL_NUMBER,
      'FIRMWARE_VERSION': GetInfoRequest.FIRMWARE_VERSION,
      'HARDWARE_VERSION': GetInfoRequest.HARDWARE_VERSION,
      'SPEC_VERSION': GetInfoRequest.SPEC_VERSION,
      'LENS_TYPE': GetInfoRequest.LENS_TYPE,
      'IMU_TYPE': GetInfoRequest.IMU_TYPE,
      'NOMINAL_BASELINE': GetInfoRequest.NOMINAL_BASELINE,
      'AUXILIARY_CHIP_VERSION': GetInfoRequest.AUXILIARY_CHIP_VERSION,
      'ISP_VERSION': GetInfoRequest.ISP_VERSION,
  }
  for k, v in get_device_info(**keys).items():
    print('{}: {}'.format(k, v))


if __name__ == '__main__':
  main()
