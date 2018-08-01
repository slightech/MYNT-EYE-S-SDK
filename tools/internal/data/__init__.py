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


def isiter_not_str(obj):
  return hasattr(obj, '__iter__') and not isinstance(obj, basestring)


class What(object):
  img_left = "img_left"
  img_right = "img_right"
  imu = "imu"
  temp = "temp"


class DataError(Exception):

  def __init__(self, message):
    super(DataError, self).__init__()
    self.message = message


class Data(object):

  def __init__(self):
    self._timestamp = 0

  @property
  def timestamp(self):
    return self._timestamp

  @timestamp.setter
  def timestamp(self, value):
    self._timestamp = value

  def __str__(self):
    return "timestamp: {:f}".format(self.timestamp)


class Image(Data):

  def __init__(self):
    super(Image, self).__init__()
    self._data = None

  @property
  def data(self):
    return self._data

  @data.setter
  def data(self, data):
    self._data = data

  @property
  def width(self):
    return 0

  @property
  def height(self):
    return 0


class IMU(Data):

  def __init__(self):
    super(IMU, self).__init__()
    self._accel_x = 0
    self._accel_y = 0
    self._accel_z = 0
    self._gyro_x = 0
    self._gyro_y = 0
    self._gyro_z = 0

  @property
  def accel(self):
    return self._accel_x, self._accel_y, self._accel_z

  @property
  def accel_x(self):
    return self._accel_x

  @accel_x.setter
  def accel_x(self, accel_x):
    self._accel_x = accel_x

  @property
  def accel_y(self):
    return self._accel_y

  @accel_y.setter
  def accel_y(self, accel_y):
    self._accel_y = accel_y

  @property
  def accel_z(self):
    return self._accel_z

  @accel_z.setter
  def accel_z(self, accel_z):
    self._accel_z = accel_z

  @property
  def gyro(self):
    return self._gyro_x, self._gyro_y, self._gyro_z

  @property
  def gyro_x(self):
    return self._gyro_x

  @gyro_x.setter
  def gyro_x(self, gyro_x):
    self._gyro_x = gyro_x

  @property
  def gyro_y(self):
    return self._gyro_y

  @gyro_y.setter
  def gyro_y(self, gyro_y):
    self._gyro_y = gyro_y

  @property
  def gyro_z(self):
    return self._gyro_z

  @gyro_z.setter
  def gyro_z(self, gyro_z):
    self._gyro_z = gyro_z

  def __str__(self):
    return super(IMU, self).__str__() + \
        "\naccel: {:f}, {:f}, {:f}".format(*self.accel) + \
        "\ngyro: {:f}, {:f}, {:f}".format(*self.gyro)


class Temp(Data):

  def __init__(self):
    super(Temp, self).__init__()
    self._value = 0

  @property
  def value(self):
    return self._value

  @value.setter
  def value(self, value):
    self._value = value

  def __str__(self):
    return super(Temp, self).__str__() + \
        "\ntemp: {:f}".format(self.value)


class Dataset(object):

  def __init__(self, path):
    self.path = path

  def generate(self, *what):  # pylint: disable=unused-argument
    raise DataError('DataError: method not implemented')

  def iterate(self, action, *what):
    for result in self.generate(*what):
      if isinstance(result, dict):  # dict > **kwds
        action(**result)
      elif isiter_not_str(result):  # iterable > *args
        action(*result)
      else:
        action(result)

  def collect(self, *what):
    results = {}
    for result in self.generate(*what):
      for key in result.keys():
        if key not in what:
          continue
        if key not in results:
          results[key] = []
        results[key].append(result[key])
    return results

  @property
  def timebeg(self):
    raise DataError('DataError: method not implemented')

  @property
  def timeend(self):
    raise DataError('DataError: method not implemented')

  @property
  def duration(self):
    raise DataError('DataError: method not implemented')


class ROSBag(Dataset):

  def __init__(self, path, **config):
    super(ROSBag, self).__init__(path)
    self.topic_img_left = config['topic_img_left'] \
        if 'topic_img_left' in config else None
    self.topic_img_right = config['topic_img_right'] \
        if 'topic_img_right' in config else None
    self.topic_imu = config['topic_imu'] \
        if 'topic_imu' in config else None
    self.topic_temp = config['topic_temp'] \
        if 'topic_temp' in config else None

    import yaml
    from rosbag.bag import Bag
    # pylint: disable=protected-access
    self._info = yaml.load(Bag(self.path, 'r')._get_yaml_info())

  def generate(self, *what):
    import rosbag
    hit_img_left = What.img_left in what
    hit_img_right = What.img_right in what
    hit_imu = What.imu in what
    hit_temp = What.temp in what
    try:
      # pylint: disable=unused-variable
      for topic, msg, t in rosbag.Bag(self.path).read_messages():
        result = {}
        stamp = msg.header.stamp.to_sec()
        if hit_img_left and topic == self.topic_img_left:
          img = Image()
          img.timestamp = stamp
          # pylint: disable=fixme
          # TODO: data with cv_bridge
          result[What.img_left] = img
        elif hit_img_right and topic == self.topic_img_right:
          img = Image()
          img.timestamp = stamp
          # TODO: data with cv_bridge
          result[What.img_right] = img
        elif hit_imu and topic == self.topic_imu:
          imu = IMU()
          imu.timestamp = stamp
          imu.accel_x = msg.linear_acceleration.x
          imu.accel_y = msg.linear_acceleration.y
          imu.accel_z = msg.linear_acceleration.z
          imu.gyro_x = msg.angular_velocity.x
          imu.gyro_y = msg.angular_velocity.y
          imu.gyro_z = msg.angular_velocity.z
          result[What.imu] = imu
        elif hit_temp and topic == self.topic_temp:
          temp = Temp()
          temp.timestamp = stamp
          temp.value = msg.data
          result[What.temp] = temp
        else:
          # raise DataError('DataError: not proper topics in the rosbag')
          continue
        yield result
    finally:
      pass

  @property
  def info(self):
    return self._info

  @property
  def timebeg(self):
    return self._info['start']

  @property
  def timeend(self):
    return self._info['end']

  @property
  def duration(self):
    return self._info['duration']


class MYNTEYE(Dataset):
  # pylint: disable=no-member

  def __init__(self, path):
    super(MYNTEYE, self).__init__(path)
    self._info = self._get_info()

  def _get_info(self):
    import os
    import sys
    from os import path

    info = type('', (), {})()

    info.img_left_dir = path.join(self.path, 'left')
    info.img_left_txt = path.join(info.img_left_dir, 'stream.txt')
    info.has_img_left = path.isfile(info.img_left_txt)

    info.img_right_dir = path.join(self.path, 'right')
    info.img_right_txt = path.join(info.img_right_dir, 'stream.txt')
    info.has_img_right = path.isfile(info.img_right_txt)

    info.imu_txt = path.join(self.path, 'motion.txt')
    info.has_imu = path.isfile(info.imu_txt)

    if info.has_img_left:
      info_txt = info.img_left_txt
    elif info.has_img_right:
      info_txt = info.img_right_txt
    elif info.has_imu:
      info_txt = info.img_left_txt
    else:
      sys.exit('Error: Dataset is empty or unexpected format')
    with open(info_txt, 'rb') as f:
      fields = [_.strip() for _ in f.readline().split(',')]

      first = f.readline()
      f.seek(-2, os.SEEK_END)
      while f.read(1) != b'\n':
        f.seek(-2, os.SEEK_CUR)
      last = f.readline()

      index = -1
      for i, field in enumerate(fields):
        if field == 'timestamp':
          index = i
          break
      if index == -1:
        sys.exit('Error: Dataset is unexpected format, timestamp not found')

      # unit from 1us to 1s
      info.timebeg = float(first.split(',')[index].strip()) * 0.000001
      info.timeend = float(last.split(',')[index].strip()) * 0.000001
      # print('time: [{}, {}]'.format(info.timebeg, info.timeend))

    return info

  def generate(self, *what):
    hit_img_left = What.img_left in what
    hit_img_right = What.img_right in what
    hit_imu = What.imu in what
    hit_temp = What.temp in what

    def get_fields(f):
      fields = {}
      for i, field in enumerate(_.strip() for _ in f.readline().split(',')):
        fields[field] = i
      return fields

    if hit_img_left and self._info.has_img_left:
      with open(self._info.img_left_txt) as f:
        fields = get_fields(f)
        for line in f:
          values = [_.strip() for _ in line.split(',')]
          img = Image()
          img.timestamp = float(values[fields['timestamp']]) * 0.000001
          yield {What.img_left: img}
    if hit_img_right and self._info.has_img_right:
      with open(self._info.img_right_txt) as f:
        fields = get_fields(f)
        for line in f:
          values = [_.strip() for _ in line.split(',')]
          img = Image()
          img.timestamp = float(values[fields['timestamp']]) * 0.000001
          yield {What.img_right: img}
    if (hit_imu or hit_temp) and self._info.has_imu:
      with open(self._info.imu_txt) as f:
        fields = get_fields(f)
        for line in f:
          values = [_.strip() for _ in line.split(',')]
          imu = IMU()
          imu.timestamp = float(values[fields['timestamp']]) * 0.000001
          imu.accel_x = float(values[fields['accel_x']])
          imu.accel_y = float(values[fields['accel_y']])
          imu.accel_z = float(values[fields['accel_z']])
          imu.gyro_x = float(values[fields['gyro_x']])
          imu.gyro_y = float(values[fields['gyro_y']])
          imu.gyro_z = float(values[fields['gyro_z']])
          temp = Temp()
          temp.timestamp = imu.timestamp
          temp.value = float(values[fields['temperature']])
          yield {What.imu: imu, What.temp: temp}

  @property
  def timebeg(self):
    return self._info.timebeg

  @property
  def timeend(self):
    return self._info.timeend

  @property
  def duration(self):
    return self.timeend - self.timebeg


if __name__ == '__main__':

  class DataA(Dataset):

    def generate(self, *what):
      yield 'a'
      yield 'b'

  class DataB(Dataset):

    def generate(self, *what):
      yield 'a1', 'a2', 'a3'
      yield 'b1', 'b2', 'b3'

  print('DataA, generate')
  for x in DataA('path').generate("what"):
    print(x)
  print('\nDataA, iterate')
  DataA('path').iterate(print, "what")

  print('\nDataB, generate')
  for x in DataB('path').generate("what"):
    print(', '.join(x))
  print('\nDataB, iterate')
  DataB('path').iterate(lambda *x: print(', '.join(x)), "what")
