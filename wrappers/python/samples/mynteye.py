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
sys.path.append(os.path.join(PY_DIR, '_output/lib'))

import mynteye_py  # pylint: disable=import-error,wrong-import-position
# glog_init = mynteye_py.glog_init.create(sys.argv)


def main():
  # api = mynteye_py.api.create()  # should glog_init
  api = mynteye_py.api.create(sys.argv)  # pylint: disable=unused-variable


if __name__ == '__main__':
  main()
