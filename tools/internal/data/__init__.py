#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
