# MYNT® EYE SDK

## Overview

MYNT® EYE SDK 2.0 is a cross-platform library for MYNT® EYE cameras.

The following platforms have been tested:

* Windows 10
* Ubuntu 16.04 / 14.04
* Jetson TX2

Please follow the guide doc to install the SDK on different platforms.

## Documentations

* [API Doc](https://github.com/slightech/MYNT-EYE-SDK-2/releases): API reference, some guides and data spec.
* [Guide Doc](https://github.com/slightech/MYNT-EYE-SDK-2-Guide/releases): How to install and start using the SDK.

> Supported languages: `zh-Hans`.

## Firmwares

[Google Drive]: https://drive.google.com/drive/folders/1tdFCcTBMNcImEGZ39tdOZmlX2SHKCr2f
[百度网盘]: https://pan.baidu.com/s/1yPQDp2r0x4jvNwn2UjlMUQ

Get firmwares from our online disks: [Google Drive][], [百度网盘][]. The latest version is `2.0.0-rc`.

## Usage

In short,

```bash
$ make
Usage:
  make help            show help message
  make apidoc          make api doc
  make opendoc         open api doc (html)
  make init            init project
  make build           build project
  make test            build test and run
  make install         install project
  make samples         build samples
  make tools           build tools
  make ros             build ros wrapper
  make clean|cleanall  clean generated or useless things
```

Init project, build samples and run someone.

```bash
make init
make samples
./samples/_output/bin/device/camera_d
```

## Mirrors

国内镜像：[码云](https://gitee.com/mynt/MYNT-EYE-SDK-2)。

## License

This project is licensed under the Apache License, Version 2.0. Copyright 2018 Slightech Co., Ltd.
