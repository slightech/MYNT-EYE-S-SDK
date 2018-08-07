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

import os
import sys

PY_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIB_DIR = os.path.join(PY_DIR, '_install/lib')
for root, dirs, files in os.walk(LIB_DIR):
  if files:
    sys.path.append(root)

# pylint: disable=import-error,wrong-import-position
import mynteye_py as mynteye
# glog_init = mynteye.glog_init.create(sys.argv)


def main():
  # api = mynteye.API.create()  # should glog_init
  api = mynteye.API.create(sys.argv)
  if not api:
    sys.exit(1)

  # model
  print('model: {}'.format(api.model))

  # supports
  supports_types = (
      mynteye.Stream,
      mynteye.Capabilities,
      mynteye.Option,
      mynteye.AddOns)
  for x in supports_types:
    print('\nsupports({})'.format(x.__name__))
    for k, v in x.names.iteritems():
      print('  supports({}): {}'.format(k, api.supports(v)))

  # get_stream_requests
  for k, v in mynteye.Capabilities.names.iteritems():
    if v != mynteye.STEREO:
      continue
    if not api.supports(v):
      continue
    print('\nget_stream_requests({})'.format(k))
    for req in api.get_stream_requests(v):
      print('  {}'.format(req))

  # config_stream_request
  # print('\nconfig_stream_request({},?)'.format(mynteye.STEREO))
  # api.config_stream_request(mynteye.STEREO, req)
  # print('  {}'.format(req))

  # get_info
  print()
  for k, v in mynteye.Info.names.iteritems():
    print('get_info({}): {}'.format(k, api.get_info(v)))

  # get_intrinsics
  print('\nget_intrinsics(LEFT)\n{}'.format(api.get_intrinsics(mynteye.LEFT)))
  print('\nget_intrinsics(RIGHT)\n{}'.format(api.get_intrinsics(mynteye.RIGHT)))
  # get_extrinsics
  print('\nget_extrinsics(LEFT, RIGHT)\n{}'.format(
      api.get_extrinsics(mynteye.LEFT, mynteye.RIGHT)))
  # get_motion_intrinsics
  print('\nget_motion_intrinsics()\n{}'.format(api.get_motion_intrinsics()))
  # get_motion_extrinsics
  print('\nget_motion_extrinsics(LEFT)\n{}'.format(
      api.get_motion_extrinsics(mynteye.LEFT)))

  # api.log_option_infos()

  # get_option_info, get_option_value
  print()
  for k, v in mynteye.Option.names.iteritems():
    if v == mynteye.ZERO_DRIFT_CALIBRATION or v == mynteye.ERASE_CHIP:
      continue
    if not api.supports(v):
      continue
    print('get_option_info({}): {}, cur: {}'.format(
        k, api.get_option_info(v), api.get_option_value(v)))

  # set_option_value

  def set_rate(frame_rate=25, imu_frequency=500):  # pylint: disable=unused-variable
    # FRAME_RATE values: 10, 15, 20, 25, 30, 35, 40, 45, 50, 55
    api.set_option_value(mynteye.FRAME_RATE, frame_rate)
    # IMU_FREQUENCY values: 100, 200, 250, 333, 500
    api.set_option_value(mynteye.IMU_FREQUENCY, imu_frequency)

  def set_ir(intensity=80):  # pylint: disable=unused-variable
    # set infrared intensity value: range [0,48], default 160
    api.set_option_value(mynteye.IR_CONTROL, intensity)

  def set_auto_exposure(  # pylint: disable=unused-variable
          max_gain=48,  # pylint: disable=bad-continuation
          max_exposure_time=240,  # pylint: disable=bad-continuation
          desired_brightness=192):  # pylint: disable=bad-continuation
    # auto-exposure: 0
    api.set_option_value(mynteye.EXPOSURE_MODE, 0)
    # max_gain: range [0,48], default 48
    api.set_option_value(mynteye.MAX_GAIN, max_gain)
    # max_exposure_time: range [0,240], default 240
    api.set_option_value(mynteye.MAX_EXPOSURE_TIME, max_exposure_time)
    # desired_brightness: range [0,255], default 192
    api.set_option_value(mynteye.DESIRED_BRIGHTNESS, desired_brightness)

  def set_manual_exposure(gain=24, brightness=120, contrast=127):  # pylint: disable=unused-variable
    # manual-exposure: 1
    api.set_option_value(mynteye.EXPOSURE_MODE, 1)
    # gain: range [0,48], default 24
    api.set_option_value(mynteye.GAIN, gain)
    # brightness/exposure_time: range [0,240], default 120
    api.set_option_value(mynteye.BRIGHTNESS, brightness)
    # contrast/black_level_calibration: range [0,255], default 127
    api.set_option_value(mynteye.CONTRAST, contrast)

  # set_rate(25, 500);
  # set_ir(80);
  # set_auto_exposure(48, 240, 192);
  # set_manual_exposure(24, 120, 127);

  # run_option_action
  # api.run_option_action(mynteye.ZERO_DRIFT_CALIBRATION)

  ##############################################################################
  # get stream and motion datas

  import cv2
  import numpy as np
  from util.cv_painter import Gravity, draw_text

  # api.enable_stream_data(mynteye.DISPARITY_NORMALIZED);

  api.enable_motion_datas()

  api.start(mynteye.ALL)

  # cv2.namedWindow('frame')
  # cv2.namedWindow('disparity')

  fps = 0.
  while True:
    t = cv2.getTickCount()

    api.wait_for_streams()

    left_data = api.get_stream_data(mynteye.LEFT)
    right_data = api.get_stream_data(mynteye.RIGHT)

    motion_datas = api.get_motion_datas()

    img = np.hstack((left_data.frame, right_data.frame))
    draw_text(img, '{1}x{0}'.format(*img.shape), Gravity.TOP_LEFT)
    draw_text(img, '{:.1f}'.format(fps), Gravity.TOP_RIGHT)
    if motion_datas:
      imu = motion_datas[0].imu
      draw_text(img,
                'accel: {:+8f}, {:+8f}, {:+8f}'.format(*imu.accel),
                Gravity.BOTTOM_LEFT)
      draw_text(img,
                'gyro: {:+8f}, {:+8f}, {:+8f}'.format(*imu.gyro),
                Gravity.BOTTOM_RIGHT)
    cv2.imshow('frame', img)

    # disp_data = api.get_stream_data(mynteye.DISPARITY_NORMALIZED);
    # if disp_data.frame is not None:
    #   cv2.imshow('disparity', disp_data.frame);

    t = cv2.getTickCount() - t
    fps = cv2.getTickFrequency() / t

    key = cv2.waitKey(10) & 0xFF
    if key == 27 or key == ord('q'):  # esc, q
      break

  api.stop(mynteye.ALL)

  cv2.destroyAllWindows()


if __name__ == '__main__':
  main()
