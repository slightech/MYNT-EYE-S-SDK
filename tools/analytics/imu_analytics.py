#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=missing-docstring

from __future__ import print_function

import os
import sys

TOOLBOX_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(TOOLBOX_DIR, 'internal'))

# pylint: disable=import-error,wrong-import-position
from data import DataError, Dataset, ROSBag, MYNTEYE, What


TIME_SCALE_FACTORS = {
    's': 1.,
    'm': 1. / 60,
    'h': 1. / 3600
}


ANGLE_DEGREES = 'd'
ANGLE_RADIANS = 'r'
ANGLE_UNITS = (ANGLE_DEGREES, ANGLE_RADIANS)


BIN_CONFIG_NAME = 'imu_analytics_bin.cfg'
BIN_IMU_NAME = 'imu_analytics_imu.bin'
BIN_TEMP_NAME = 'imu_analytics_temp.bin'


class RawDataset(Dataset):

  def __init__(self, path, dataset_creator):
    super(RawDataset, self).__init__(path)
    self.dataset_creator = dataset_creator
    self._digest()

  def _digest(self):
    dataset = self.dataset_creator(self.path)
    results = dataset.collect(What.imu, What.temp)
    self._dataset = dataset
    self._results = results
    self._has_imu = What.imu in results.keys()
    self._has_temp = What.temp in results.keys()
    print('  ' + ', '.join('{}: {}'.format(k, len(v))
                           for k, v in results.items()))

  @staticmethod
  def _hypot(*args):
    from math import sqrt
    return sqrt(sum(x ** 2 for x in args))

  def plot(self, t_scale_factor, gryo_converter,
           ax_accel_x, ax_accel_y, ax_accel_z, ax_accel,
           ax_gyro_x, ax_gyro_y, ax_gyro_z, ax_temp):
    results = self._results

    if self._has_imu:
      imu_t_beg = results[What.imu][0].timestamp
      imu_ts = [(imu.timestamp - imu_t_beg) * t_scale_factor
                for imu in results[What.imu]]

      ax_accel_x.plot(imu_ts, [imu.accel_x for imu in results[What.imu]])
      ax_accel_y.plot(imu_ts, [imu.accel_y for imu in results[What.imu]])
      ax_accel_z.plot(imu_ts, [imu.accel_z for imu in results[What.imu]])
      import math
      my_gryo_converter = \
          lambda x: gryo_converter(x, math.degrees, math.radians)
      ax_gyro_x.plot(imu_ts, [my_gryo_converter(imu.gyro_x)
                              for imu in results[What.imu]])
      ax_gyro_y.plot(imu_ts, [my_gryo_converter(imu.gyro_y)
                              for imu in results[What.imu]])
      ax_gyro_z.plot(imu_ts, [my_gryo_converter(imu.gyro_z)
                              for imu in results[What.imu]])

      ax_accel.plot(imu_ts, [self._hypot(imu.accel_x, imu.accel_y, imu.accel_z)
                             for imu in results[What.imu]])

    if self._has_temp:
      temp_t_beg = results[What.temp][0].timestamp
      temp_ts = [(temp.timestamp - temp_t_beg) * t_scale_factor
                 for temp in results[What.temp]]
      ax_temp.plot(temp_ts, [temp.value for temp in results[What.temp]])

  def generate(self, *what):  # pylint: disable=unused-argument
    raise DataError('DataError: method not implemented')

  def iterate(self, action, *what):  # pylint: disable=unused-argument
    raise DataError('DataError: method not implemented')

  def collect(self, *what):  # pylint: disable=unused-argument
    raise DataError('DataError: method not implemented')

  @property
  def timebeg(self):
    return self._dataset.timebeg

  @property
  def timeend(self):
    return self._dataset.timeend

  @property
  def duration(self):
    return self._dataset.duration

  @property
  def has_imu(self):
    return self._has_imu

  @property
  def has_temp(self):
    return self._has_temp


class BinDataset(RawDataset):
  """
  Binary memory-mapped files of large dataset.

  References:
    https://stackoverflow.com/questions/5854515/large-plot-20-million-samples-gigabytes-of-data
    https://stackoverflow.com/questions/1053928/very-large-matrices-using-python-and-numpy
  """

  # def __init__(self, path, dataset_creator):
  #   super(BinDataset, self).__init__(path, dataset_creator)

  def _digest(self):
    bindir = os.path.splitext(self.path)[0]
    bincfg = os.path.join(bindir, BIN_CONFIG_NAME)
    if os.path.isfile(bincfg):
      with open(bincfg, 'r') as f_cfg:
        import yaml
        cfg = yaml.load(f_cfg)
        self._info = cfg['info']
        self._binimu = os.path.join(bindir, cfg['bins']['imu'])
        self._bintemp = os.path.join(bindir, cfg['bins']['temp'])
        print('find binary files ...')
        print('  binimu: {}'.format(self._binimu))
        print('  bintemp: {}'.format(self._bintemp))
        print('  bincfg: {}'.format(bincfg))
        if self._exists():
          while True:
            sys.stdout.write('Do you want to use it directly? [Y/n] ')
            choice = raw_input().lower()
            if choice == '' or choice == 'y':
              return
            elif choice == 'n':
              break
            else:
              print('Please respond with \'y\' or \'n\'.')
    self._convert()

  def _exists(self):
    return os.path.isfile(self._binimu) or os.path.isfile(self._bintemp)

  def _convert(self):
    import numpy as np

    dataset = self.dataset_creator(self.path)
    bindir = os.path.splitext(self.path)[0]
    if not os.path.exists(bindir):
      os.makedirs(bindir)
    binimu = os.path.join(bindir, BIN_IMU_NAME)
    bintemp = os.path.join(bindir, BIN_TEMP_NAME)
    bincfg = os.path.join(bindir, BIN_CONFIG_NAME)
    print('save to binary files ...')
    print('  binimu: {}'.format(binimu))
    print('  bintemp: {}'.format(bintemp))
    print('  bincfg: {}'.format(bincfg))

    has_imu = False
    has_temp = False
    with open(binimu, 'wb') as f_imu, open(bintemp, 'wb') as f_temp:
      imu_t_beg = -1
      imu_count = 0
      temp_t_beg = -1
      temp_count = 0
      for result in dataset.generate(What.imu, What.temp):
        if What.imu in result:
          imu = result[What.imu]
          if imu_t_beg == -1:
            imu_t_beg = imu.timestamp
          np.array([(
              (imu.timestamp - imu_t_beg),
              imu.accel_x, imu.accel_y, imu.accel_z,
              self._hypot(imu.accel_x, imu.accel_y, imu.accel_z),
              imu.gyro_x, imu.gyro_y, imu.gyro_z
          )], dtype="f8, f8, f8, f8, f8, f8, f8, f8").tofile(f_imu)
          imu_count = imu_count + 1
          has_imu = True
        if What.temp in result:
          temp = result[What.temp]
          if temp_t_beg == -1:
            temp_t_beg = temp.timestamp
          np.array([(
              (temp.timestamp - temp_t_beg),
              temp.value
          )], dtype="f8, f8").tofile(f_temp)
          temp_count = temp_count + 1
          has_temp = True
        sys.stdout.write('\r  imu: {}, temp: {}'.format(imu_count, temp_count))
      sys.stdout.write('\n')

    # pylint: disable=attribute-defined-outside-init
    self._info = {
        'timebeg': dataset.timebeg,
        'timeend': dataset.timeend,
        'duration': dataset.duration,
        'has_imu': has_imu,
        'has_temp': has_temp
    }
    self._binimu = binimu
    self._bintemp = bintemp

    with open(bincfg, 'w') as f_cfg:
      import yaml
      yaml.dump({'info': self._info, 'bins': {
          'imu': BIN_IMU_NAME,
          'temp': BIN_TEMP_NAME
      }}, f_cfg, default_flow_style=False)

  def plot(self, t_scale_factor, gryo_converter,
           ax_accel_x, ax_accel_y, ax_accel_z, ax_accel,
           ax_gyro_x, ax_gyro_y, ax_gyro_z, ax_temp):
    import numpy as np
    if self.has_imu:
      imus = np.memmap(self._binimu, dtype=[
          ('t', 'f8'),
          ('accel_x', 'f8'), ('accel_y', 'f8'), ('accel_z', 'f8'),
          ('accel', 'f8'),
          ('gyro_x', 'f8'), ('gyro_y', 'f8'), ('gyro_z', 'f8'),
      ], mode='r')
      imus_t = imus['t'] * t_scale_factor
      ax_accel_x.plot(imus_t, imus['accel_x'])
      ax_accel_y.plot(imus_t, imus['accel_y'])
      ax_accel_z.plot(imus_t, imus['accel_z'])
      ax_accel.plot(imus_t, imus['accel'])

      my_gryo_converter = \
          lambda x: gryo_converter(x, np.degrees, np.radians)
      ax_gyro_x.plot(imus_t, my_gryo_converter(imus['gyro_x']))
      ax_gyro_y.plot(imus_t, my_gryo_converter(imus['gyro_y']))
      ax_gyro_z.plot(imus_t, my_gryo_converter(imus['gyro_z']))
    if self.has_temp:
      temps = np.memmap(self._bintemp, dtype=[
          ('t', 'f8'), ('value', 'f8')
      ], mode='r')
      temps_t = temps['t'] * t_scale_factor
      ax_temp.plot(temps_t, temps['value'])

  @property
  def timebeg(self):
    return self._info['timebeg']

  @property
  def timeend(self):
    return self._info['timeend']

  @property
  def duration(self):
    return self._info['duration']

  @property
  def has_imu(self):
    return self._info['has_imu']

  @property
  def has_temp(self):
    return self._info['has_temp']


def analyze(dataset, profile):
  if not profile.time_unit:
    if dataset.duration > 3600:
      time_unit = 'h'
    elif dataset.duration > 60:
      time_unit = 'm'
    else:
      time_unit = 's'
  else:
    time_unit = profile.time_unit

  t_name = 'time ({})'.format(time_unit)
  t_scale_factor = TIME_SCALE_FACTORS[time_unit]

  time_limits = profile.time_limits
  if not time_limits:
    time_limits = [0, dataset.duration * t_scale_factor]
  accel_limits = profile.accel_limits
  gyro_limits = profile.gyro_limits
  temp_limits = profile.temp_limits
  auto = profile.auto

  import matplotlib.pyplot as plt

  fig_1 = plt.figure(1, [16, 12])
  fig_1.suptitle('IMU Analytics')
  fig_1.subplots_adjust(wspace=0.4, hspace=0.2)

  ax_accel_x = fig_1.add_subplot(241)
  ax_accel_x.set_title('accel_x')
  ax_accel_x.set_xlabel(t_name)
  ax_accel_x.set_ylabel('accel_x (m/s^2)')
  ax_accel_x.axis('auto')
  ax_accel_x.set_xlim(time_limits)
  if not auto and accel_limits and accel_limits[0]:
    ax_accel_x.set_ylim(accel_limits[0])

  ax_accel_y = fig_1.add_subplot(242)
  ax_accel_y.set_title('accel_y')
  ax_accel_y.set_xlabel(t_name)
  ax_accel_y.set_ylabel('accel_y (m/s^2)')
  ax_accel_y.axis('auto')
  ax_accel_y.set_xlim(time_limits)
  if not auto and accel_limits and accel_limits[1]:
    ax_accel_y.set_ylim(accel_limits[1])

  ax_accel_z = fig_1.add_subplot(243)
  ax_accel_z.set_title('accel_z')
  ax_accel_z.set_xlabel(t_name)
  ax_accel_z.set_ylabel('accel_z (m/s^2)')
  ax_accel_z.axis('auto')
  ax_accel_z.set_xlim(time_limits)
  if not auto and accel_limits and accel_limits[2]:
    ax_accel_z.set_ylim(accel_limits[2])

  ax_accel = fig_1.add_subplot(244)
  ax_accel.set_title('accel hypot(x,y,z)')
  ax_accel.set_xlabel(t_name)
  ax_accel.set_ylabel('accel (m/s^2)')
  ax_accel.axis('auto')
  ax_accel.set_xlim(time_limits)
  if not auto and accel_limits and accel_limits[3]:
    ax_accel.set_ylim(accel_limits[3])

  ax_gyro_ylabels = {
      ANGLE_DEGREES: 'deg/sec',
      ANGLE_RADIANS: 'rad/sec'
  }
  ax_gyro_ylabel = ax_gyro_ylabels[profile.gyro_show_unit]

  ax_gyro_x = fig_1.add_subplot(245)
  ax_gyro_x.set_title('gyro_x')
  ax_gyro_x.set_xlabel(t_name)
  ax_gyro_x.set_ylabel('gyro_x ({})'.format(ax_gyro_ylabel))
  ax_gyro_x.axis('auto')
  ax_gyro_x.set_xlim(time_limits)
  if not auto and gyro_limits and gyro_limits[0]:
    ax_gyro_x.set_ylim(gyro_limits[0])

  ax_gyro_y = fig_1.add_subplot(246)
  ax_gyro_y.set_title('gyro_y')
  ax_gyro_y.set_xlabel(t_name)
  ax_gyro_y.set_ylabel('gyro_y ({})'.format(ax_gyro_ylabel))
  ax_gyro_y.axis('auto')
  ax_gyro_y.set_xlim(time_limits)
  if not auto and gyro_limits and gyro_limits[1]:
    ax_gyro_y.set_ylim(gyro_limits[1])

  ax_gyro_z = fig_1.add_subplot(247)
  ax_gyro_z.set_title('gyro_z')
  ax_gyro_z.set_xlabel(t_name)
  ax_gyro_z.set_ylabel('gyro_z ({})'.format(ax_gyro_ylabel))
  ax_gyro_z.axis('auto')
  ax_gyro_z.set_xlim(time_limits)
  if not auto and gyro_limits and gyro_limits[2]:
    ax_gyro_z.set_ylim(gyro_limits[2])

  ax_temp = None
  if dataset.has_temp:
    ax_temp = fig_1.add_subplot(248)
    ax_temp.set_title('temperature')
    ax_temp.set_xlabel(t_name)
    ax_temp.set_ylabel('temperature (degree Celsius)')
    ax_temp.axis('auto')
    ax_temp.set_xlim(time_limits)
    if not auto and temp_limits:
      ax_temp.set_ylim(temp_limits)

  def gryo_converter(x, degrees, radians):
    if profile.gyro_show_unit == profile.gyro_data_unit:
      return x
    if profile.gyro_show_unit == ANGLE_DEGREES and \
            profile.gyro_data_unit == ANGLE_RADIANS:
      return degrees(x)
    if profile.gyro_show_unit == ANGLE_RADIANS and \
            profile.gyro_data_unit == ANGLE_DEGREES:
      return radians(x)
    sys.exit('Error: gryo_converter wrong logic')

  dataset.plot(t_scale_factor, gryo_converter,
               ax_accel_x, ax_accel_y, ax_accel_z, ax_accel,
               ax_gyro_x, ax_gyro_y, ax_gyro_z, ax_temp)

  outdir = profile.outdir
  if outdir:
    figpath = os.path.join(outdir, 'imu_analytics.png')
    print('save figure to:\n  {}'.format(figpath))
    if not os.path.exists(outdir):
      os.makedirs(outdir)
    fig_1.savefig(figpath, dpi=100)

  plt.show()


def _parse_args():
  def limits_type(string, num=1):
    if not string:
      return None
    if num < 1:
      sys.exit('Error: limits_type must be greater than one pair')
    pairs = string.split(':')
    pairs_len = len(pairs)
    if pairs_len == 1:
      values = pairs[0].split(',')
      if len(values) != 2:
        sys.exit('Error: limits_type must be two values'
                 ' as \'min,max\' for each pair')
      results = (float(values[0]), float(values[1]))
      if num > 1:
        return [results for i in xrange(num)]
      else:
        return results
    elif pairs_len == num:
      results = []
      for i in xrange(num):
        if pairs[i]:
          values = pairs[i].split(',')
          if len(values) != 2:
            sys.exit('Error: limits_type must be two values'
                     ' as \'min,max\' for each pair')
          results.append((float(values[0]), float(values[1])))
        else:
          results.append(None)
      return results
    else:
      sys.exit('Error: limits_type must one or {:d} pairs'.format(num))

  from functools import partial

  import argparse
  parser = argparse.ArgumentParser(
      prog=os.path.basename(__file__),
      formatter_class=argparse.RawTextHelpFormatter,
      description='usage examples:'
      '\n  python %(prog)s -i DATASET'
      '\n  python %(prog)s -i DATASET -al=-10,10'
      '\n  python %(prog)s -i DATASET -al=-5,5::5,15: -gl=-0.1,0.1:: -kl=')
  parser.add_argument(
      '-i',
      '--input',
      dest='input',
      metavar='DATASET',
      required=True,
      help='the input dataset path')
  parser.add_argument(
      '-o',
      '--outdir',
      dest='outdir',
      metavar='OUTDIR',
      help='the output directory')
  parser.add_argument(
      '-c',
      '--config',
      dest='config',
      metavar='CONFIG',
      help='yaml config file about input dataset')
  parser.add_argument(
      '-tu',
      '--time-unit',
      dest='time_unit',
      metavar='s|m|h',
      help='the time unit (seconds, minutes or hours)')
  parser.add_argument(
      '-gdu',
      '--gyro-data-unit',
      dest='gyro_data_unit',
      metavar='r|d',
      default='r',
      help='the gyro data unit (radians or degrees, default: %(default)s)')
  parser.add_argument(
      '-gsu',
      '--gyro-show-unit',
      dest='gyro_show_unit',
      metavar='r|d',
      help='the gyro show unit (radians or degrees, '
      'default: same as gyro data unit)')
  parser.add_argument(
      '-tl',
      '--time-limits',
      dest='time_limits',
      metavar='min,max',
      type=limits_type,
      help='the time limits, in time unit')
  parser.add_argument(
      '-al',
      '--accel-limits',
      dest='accel_limits',
      metavar='min,max [min,max:...]',
      default='-10,10',
      type=partial(limits_type, num=4),
      help='the accel limits (default: %(default)s)'
      '\n  or 4 limits of accel_x,y,z,accel like \'min,max:...\'')
  parser.add_argument(
      '-gl',
      '--gyro-limits',
      dest='gyro_limits',
      metavar='min,max [min,max:...]',
      default='-0.02,0.02',
      type=partial(limits_type, num=3),
      help='the gyro limits (default: %(default)s)'
      '\n  or 3 limits of gyro_x,y,z like \'min,max:...\'')
  parser.add_argument(
      '-kl',
      '--temp-limits',
      dest='temp_limits',
      metavar='min,max',
      default='-20,80',
      type=limits_type,
      help='the temperature limits (default: %(default)s)')
  parser.add_argument(
      '-l',
      '--limits',
      dest='all_limits',
      metavar='min,max [min,max:...]',
      # nargs='+',
      type=partial(limits_type, num=8),
      help='the all limits, absent one will auto scale'
      '\n  accel_x,y,z,accel,gyro_x,y,z,temp like \'min,max:...\'')
  parser.add_argument(
      '-a',
      '--auto',
      dest='auto',
      action='store_true',
      help='make all limits auto scale to data limits, except the time')
  parser.add_argument(
      '-b',
      '--binary',
      dest='binary',
      action='store_true',
      help='save large dataset to binary files'
      ', and plot them with numpy.memmap()')
  return parser.parse_args()


def _dict2obj(d):
  from collections import namedtuple
  return namedtuple('X', d.keys())(*d.values())


def _main():
  args = _parse_args()
  # print(args)

  dataset_path = args.input
  if not dataset_path or not os.path.exists(dataset_path):
    sys.exit('Error: the dataset path not exists, %s' % dataset_path)
  dataset_path = os.path.normpath(dataset_path)

  outdir = args.outdir
  if not outdir:
    outdir = os.path.splitext(dataset_path)[0]
  else:
    outdir = os.path.abspath(outdir)

  print('imu analytics ...')
  print('  input: %s' % dataset_path)
  print('  outdir: %s' % outdir)

  profile = {
      'auto': False,
      'time_unit': None,
      'gyro_data_unit': None,
      'gyro_show_unit': None,
      'time_limits': None,
      'accel_limits': None,
      'gyro_limits': None,
      'temp_limits': None
  }
  profile['auto'] = args.auto

  if args.time_unit:
    if args.time_unit not in TIME_SCALE_FACTORS.keys():
      sys.exit('Error: the time unit must be \'s|m|h\'')
    else:
      profile['time_unit'] = args.time_unit

  if args.gyro_data_unit:
    if args.gyro_data_unit not in ANGLE_UNITS:
      sys.exit('Error: the gyro unit must be \'r|d\'')
    else:
      profile['gyro_data_unit'] = args.gyro_data_unit
  else:
    profile['gyro_data_unit'] = ANGLE_RADIANS

  if args.gyro_show_unit:
    if args.gyro_show_unit not in ANGLE_UNITS:
      sys.exit('Error: the gyro unit must be \'r|d\'')
    else:
      profile['gyro_show_unit'] = args.gyro_show_unit
  else:
    profile['gyro_show_unit'] = profile['gyro_data_unit']

  if args.time_limits:
    if not args.time_unit:
      sys.exit('Error: the time unit must be set')
    profile['time_limits'] = args.time_limits

  if args.all_limits:
    profile['accel_limits'] = args.all_limits[:4]
    profile['gyro_limits'] = args.all_limits[4:7]
    profile['temp_limits'] = args.all_limits[7]
  else:
    profile['accel_limits'] = args.accel_limits
    profile['gyro_limits'] = args.gyro_limits
    profile['temp_limits'] = args.temp_limits

  for k, v in profile.items():
    print('  {}: {}'.format(k, v))

  def dataset_creator(path):
    print('open dataset ...')
    if args.config:
      import yaml
      config = yaml.load(file(args.config, 'r'))
      model = config['dataset']
      if model == 'rosbag':
        dataset = ROSBag(path, **config['rosbag'])
      elif model == 'mynteye':
        dataset = MYNTEYE(path)
      else:
        sys.exit('Error: dataset model not supported {}'.format(model))
    else:
      dataset = ROSBag(
          path,
          topic_imu='/mynteye/imu',
          topic_temp='/mynteye/temp')
    return dataset

  if args.binary:
    dataset = BinDataset(dataset_path, dataset_creator)
  else:
    dataset = RawDataset(dataset_path, dataset_creator)
  print('  timebeg: {:f}, timeend: {:f}, duration: {:f}'.format(
      dataset.timebeg, dataset.timeend, dataset.duration))

  profile['outdir'] = outdir
  analyze(dataset, _dict2obj(profile))

  print('imu analytics done')


if __name__ == '__main__':
  _main()
