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

import cv2


FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 1
FONT_COLOR = (255, 255, 255)
THICKNESS = 1


class Gravity(object):  # pylint: disable=no-init
  TOP_LEFT = 1
  TOP_RIGHT = 2
  BOTTOM_LEFT = 3
  BOTTOM_RIGHT = 4


def draw_text(img, text, gravity, margin=10, offset_x=0, offset_y=0):
  h, w = img.shape[:2]  # pylint: disable=invalid-name

  # getTextSize, result: ((width, height), baseline)
  x, y = cv2.getTextSize(text, FONT_FACE, FONT_SCALE, THICKNESS)[0]

  org = {
      Gravity.TOP_LEFT:     (margin, margin + y),
      Gravity.TOP_RIGHT:    (w - margin - x, margin + y),
      Gravity.BOTTOM_LEFT:  (margin, h - margin),
      Gravity.BOTTOM_RIGHT: (w - margin - x, h - margin),
  }.get(gravity, (margin, margin + y))

  org = (org[0] + offset_x, org[1] + offset_y)

  # putText(img, text, org, fontFace, fontScale, color, thickness, lineType)
  cv2.putText(img, text, org, FONT_FACE, FONT_SCALE,
              FONT_COLOR, THICKNESS, cv2.LINE_AA)
