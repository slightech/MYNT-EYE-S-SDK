#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=missing-docstring

from __future__ import print_function

import os
import sys

TOOLBOX_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(TOOLBOX_DIR, 'internal'))

# pylint: disable=import-error,wrong-import-position
from data import ROSBag, What


ANGLE_DEGREES = 'd'
ANGLE_RADIANS = 'r'
ANGLE_UNITS = (ANGLE_DEGREES, ANGLE_RADIANS)

BIN_IMG_NAME = 'stamp_analytics_img.bin'
BIN_IMU_NAME = 'stamp_analytics_imu.bin'

RESULT_FIGURE = 'stamp_analytics.png'


class BinDataset(object):

  def __init__(self, path, dataset_creator):
    self.path = path
    self.dataset_creator = dataset_creator
    self._digest()

  def _digest(self):
    bindir = os.path.splitext(self.path)[0]
    binimg = os.path.join(bindir, BIN_IMG_NAME)
    binimu = os.path.join(bindir, BIN_IMU_NAME)
    if os.path.isfile(binimg) and os.path.isfile(binimu):
      print('find binary files ...')
      print('  binimg: {}'.format(binimg))
      print('  binimu: {}'.format(binimu))
      while True:
        sys.stdout.write('Do you want to use it directly? [Y/n] ')
        choice = raw_input().lower()
        if choice == '' or choice == 'y':
          self._binimg = binimg
          self._binimu = binimu
          self._has_img = True
          self._has_imu = True
          return
        elif choice == 'n':
          break
        else:
          print('Please respond with \'y\' or \'n\'.')
    self._convert()

  def _convert(self):
    import numpy as np

    dataset = self.dataset_creator(self.path)
    bindir = os.path.splitext(self.path)[0]
    if not os.path.exists(bindir):
      os.makedirs(bindir)
    binimg = os.path.join(bindir, BIN_IMG_NAME)
    binimu = os.path.join(bindir, BIN_IMU_NAME)
    print('save to binary files ...')
    print('  binimg: {}'.format(binimg))
    print('  binimu: {}'.format(binimu))

    has_img = False
    has_imu = False
    with open(binimg, 'wb') as f_img, open(binimu, 'wb') as f_imu:
      img_count = 0
      imu_count = 0
      for result in dataset.generate(What.img_left, What.imu):
        if What.img_left in result:
          img = result[What.img_left]
          np.array([(
              img.timestamp
          )], dtype="f8").tofile(f_img)
          img_count = img_count + 1
          has_img = True
        if What.imu in result:
          imu = result[What.imu]
          np.array([(
              imu.timestamp,
              imu.accel_x, imu.accel_y, imu.accel_z,
              imu.gyro_x, imu.gyro_y, imu.gyro_z
          )], dtype="f8, f8, f8, f8, f8, f8, f8").tofile(f_imu)
          imu_count = imu_count + 1
          has_imu = True
        sys.stdout.write('\r  img: {}, imu: {}'.format(img_count, imu_count))
      sys.stdout.write('\n')

    # pylint: disable=attribute-defined-outside-init
    self._binimg = binimg
    self._binimu = binimu
    self._has_img = has_img
    self._has_imu = has_imu

  def stamp_analytics(self, args):
    outdir = args.outdir

    import numpy as np
    if self.has_img:
      # pd.cut fails on readonly arrays
      #   https://github.com/pandas-dev/pandas/issues/18773
      # imgs = np.memmap(self._binimg, dtype=[
      #     ('t', 'f8')
      # ], mode='r')
      imgs = np.fromfile(self._binimg, dtype=[
          ('t', 'f8')
      ])
    else:
      sys.exit("Error: there are no imgs.")

    if self.has_imu:
      imus = np.memmap(self._binimu, dtype=[
          ('t', 'f8'),
          ('accel_x', 'f8'), ('accel_y', 'f8'), ('accel_z', 'f8'),
          ('gyro_x', 'f8'), ('gyro_y', 'f8'), ('gyro_z', 'f8'),
      ], mode='r')
    else:
      sys.exit("Error: there are no imus.")

    period_img = 1. / args.rate_img
    period_imu = 1. / args.rate_imu
    print('\nrate (Hz)')
    print('  img: {}, imu: {}'.format(args.rate_img, args.rate_imu))
    print('sample period (s)')
    print('  img: {}, imu: {}'.format(period_img, period_imu))

    imgs_t_diff = np.diff(imgs['t'])
    imus_t_diff = np.diff(imus['t'])

    print('\ndiff count')
    print('  imgs: {}, imus: {}'.format(imgs['t'].size, imus['t'].size))
    print('  imgs_t_diff: {}, imus_t_diff: {}'
          .format(imgs_t_diff.size, imus_t_diff.size))

    print('\ndiff where (factor={})'.format(args.factor))

    where = np.argwhere(imgs_t_diff > period_img * (1 + args.factor))
    print('  imgs where diff > {}*{} ({})'.format(period_img,
                                                  1 + args.factor, where.size))
    for x in where:
      print('  {:8d}: {:.16f}'.format(x[0], imgs_t_diff[x][0]))

    where = np.argwhere(imgs_t_diff < period_img * (1 - args.factor))
    print('  imgs where diff < {}*{} ({})'.format(period_img,
                                                  1 - args.factor, where.size))
    for x in where:
      print('  {:8d}: {:.16f}'.format(x[0], imgs_t_diff[x][0]))

    where = np.argwhere(imus_t_diff > period_imu * (1 + args.factor))
    print('  imus where diff > {}*{} ({})'.format(period_imu,
                                                  1 + args.factor, where.size))
    for x in where:
      print('  {:8d}: {:.16f}'.format(x[0], imus_t_diff[x][0]))

    where = np.argwhere(imus_t_diff < period_imu * (1 - args.factor))
    print('  imus where diff < {}*{} ({})'.format(period_imu,
                                                  1 - args.factor, where.size))
    for x in where:
      print('  {:8d}: {:.16f}'.format(x[0], imus_t_diff[x][0]))

    import pandas as pd
    bins = imgs['t']
    bins_n = imgs['t'].size
    bins = pd.Series(data=bins).drop_duplicates(keep='first')
    cats = pd.cut(imus['t'], bins)

    print('\nimage timestamp duplicates: {}'.format(bins_n - bins.size))

    self._plot(outdir, imgs_t_diff, imus_t_diff, cats.value_counts())

  def _plot(self, outdir, imgs_t_diff, imus_t_diff, imgs_t_imus):
    import matplotlib.pyplot as plt
    import numpy as np

    fig_1 = plt.figure(1, [16, 6])
    fig_1.suptitle('Stamp Analytics')
    fig_1.subplots_adjust(
        left=0.1,
        right=0.95,
        top=0.85,
        bottom=0.15,
        wspace=0.4)

    ax_imgs_t_diff = fig_1.add_subplot(131)
    ax_imgs_t_diff.set_title('Image Timestamp Diff')
    ax_imgs_t_diff.set_xlabel('diff index')
    ax_imgs_t_diff.set_ylabel('diff (s)')
    ax_imgs_t_diff.axis('auto')

    ax_imus_t_diff = fig_1.add_subplot(132)
    ax_imus_t_diff.set_title('Imu Timestamp Diff')
    ax_imus_t_diff.set_xlabel('diff index')
    ax_imus_t_diff.set_ylabel('diff (s)')
    ax_imus_t_diff.axis('auto')

    ax_imgs_t_imus = fig_1.add_subplot(133)
    ax_imgs_t_imus.set_title('Imu Count Per Image Intervel')
    ax_imgs_t_imus.set_xlabel('intervel index')
    ax_imgs_t_imus.set_ylabel('imu count')
    ax_imgs_t_imus.axis('auto')

    ax_imgs_t_diff.set_xlim([0, imgs_t_diff.size])
    ax_imgs_t_diff.plot(imgs_t_diff)

    ax_imus_t_diff.set_xlim([0, imus_t_diff.size])
    ax_imus_t_diff.plot(imus_t_diff)

    # print(imgs_t_imus.values)
    # imgs_t_imus.plot(kind='line', ax=ax_imgs_t_imus)
    data = imgs_t_imus.values
    ax_imgs_t_imus.set_xlim([0, data.size])
    ax_imgs_t_imus.set_ylim([np.min(data) - 1, np.max(data) + 1])
    ax_imgs_t_imus.plot(data)

    if outdir:
      figpath = os.path.join(outdir, RESULT_FIGURE)
      print('\nsave figure to:\n  {}'.format(figpath))
      if not os.path.exists(outdir):
        os.makedirs(outdir)
      fig_1.savefig(figpath, dpi=100)

    plt.show()

  @property
  def has_img(self):
    return self._has_img

  @property
  def has_imu(self):
    return self._has_imu


def _parse_args():
  import argparse
  parser = argparse.ArgumentParser(
      prog=os.path.basename(__file__),
      formatter_class=argparse.RawTextHelpFormatter,
      description='usage examples:'
      '\n  python %(prog)s -i DATASET')
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
      '-f',
      '--factor',
      dest='factor',
      metavar='FACTOR',
      default=0.1,
      type=float,
      help='the wave factor (default: %(default)s)')
  parser.add_argument(
      '--rate-img',
      dest='rate_img',
      metavar='RATE',
      default=25,
      type=int,
      help='the img rate (default: %(default)s)')
  parser.add_argument(
      '--rate-imu',
      dest='rate_imu',
      metavar='RATE',
      default=500,
      type=int,
      help='the imu rate (default: %(default)s)')
  return parser.parse_args()


def _main():
  args = _parse_args()

  dataset_path = args.input
  if not dataset_path or not os.path.exists(dataset_path):
    sys.exit('Error: the dataset path not exists, %s' % dataset_path)
  dataset_path = os.path.normpath(dataset_path)

  outdir = args.outdir
  if not args.outdir:
    outdir = os.path.splitext(dataset_path)[0]
  else:
    outdir = os.path.abspath(outdir)
  args.outdir = outdir

  print('stamp analytics ...')
  print('  input: %s' % dataset_path)
  print('  outdir: %s' % outdir)

  def dataset_creator(path):
    print('open dataset ...')
    if args.config:
      import yaml
      config = yaml.load(file(args.config, 'r'))
      if config['dataset'] != 'rosbag':
        sys.exit('Error: dataset model only support rosbag now')
      dataset = ROSBag(path, **config['rosbag'])
    else:
      dataset = ROSBag(path,
                       topic_img_left='/mynteye/left',
                       topic_imu='/mynteye/imu')
    return dataset

  dataset = BinDataset(dataset_path, dataset_creator)
  dataset.stamp_analytics(args)

  print('stamp analytics done')


if __name__ == '__main__':
  _main()
