# MYNT® EYE SDK

[![](https://img.shields.io/badge/MYNT%20EYE%20SDK-2.0.1--rc2-brightgreen.svg?style=flat)](https://github.com/slightech/MYNT-EYE-SDK-2)

## Overview

MYNT® EYE SDK 2.0 is a cross-platform library for MYNT® EYE cameras.

The following platforms have been tested:

* Windows 10
* Ubuntu 16.04 / 14.04
* Jetson TX2

Please follow the guide doc to install the SDK on different platforms.

## Documentations

* [API Doc](https://github.com/slightech/MYNT-EYE-SDK-2/releases): API reference, some guides and data spec.
  * en: [![](https://img.shields.io/badge/Download-PDF-blue.svg?style=flat)](https://github.com/slightech/MYNT-EYE-SDK-2/files/2422149/mynt-eye-sdk-apidoc-2.0.1-rc2-en.pdf) [![](https://img.shields.io/badge/Download-HTML-blue.svg?style=flat)](https://github.com/slightech/MYNT-EYE-SDK-2/files/2422151/mynt-eye-sdk-apidoc-2.0.1-rc2-html-en.zip) [![](https://img.shields.io/badge/Online-HTML-blue.svg?style=flat)](https://slightech.github.io/MYNT-EYE-SDK-2/)
  * zh-Hans: [![](https://img.shields.io/badge/Download-PDF-blue.svg?style=flat)](https://github.com/slightech/MYNT-EYE-SDK-2/files/2422152/mynt-eye-sdk-apidoc-2.0.1-rc2-zh-Hans.pdf) [![](https://img.shields.io/badge/Download-HTML-blue.svg?style=flat)](https://github.com/slightech/MYNT-EYE-SDK-2/files/2422153/mynt-eye-sdk-apidoc-2.0.1-rc2-html-zh-Hans.zip) [![](https://img.shields.io/badge/Online-HTML-blue.svg?style=flat)](http://doc.myntai.com/resource/api/mynt-eye-sdk-apidoc-2.0.1-rc2-html-zh-Hans/mynt-eye-sdk-apidoc-2.0.1-rc2-html-zh-Hans/index.html)
* [Guide Doc](https://github.com/slightech/MYNT-EYE-SDK-2-Guide/releases): How to install and start using the SDK.
  * en: [![](https://img.shields.io/badge/Download-PDF-blue.svg?style=flat)](https://github.com/slightech/MYNT-EYE-SDK-2-Guide/files/2422160/mynt-eye-sdk-guide-2.0.1-rc2-en.pdf) [![](https://img.shields.io/badge/Download-HTML-blue.svg?style=flat)](https://github.com/slightech/MYNT-EYE-SDK-2-Guide/files/2422161/mynt-eye-sdk-guide-2.0.1-rc2-html-en.zip) [![](https://img.shields.io/badge/Online-HTML-blue.svg?style=flat)](https://slightech.github.io/MYNT-EYE-SDK-2-Guide/)
  * zh-Hans: [![](https://img.shields.io/badge/Download-PDF-blue.svg?style=flat)](https://github.com/slightech/MYNT-EYE-SDK-2-Guide/files/2422162/mynt-eye-sdk-guide-2.0.1-rc2-zh-Hans.pdf) [![](https://img.shields.io/badge/Download-HTML-blue.svg?style=flat)](https://github.com/slightech/MYNT-EYE-SDK-2-Guide/files/2422164/mynt-eye-sdk-guide-2.0.1-rc2-html-zh-Hans.zip) [![](https://img.shields.io/badge/Online-HTML-blue.svg?style=flat)](http://doc.myntai.com/resource/sdk/mynt-eye-sdk-guide-2.0.1-rc2-html-zh-Hans/mynt-eye-sdk-guide-2.0.1-rc2-html-zh-Hans/index.html)

> Supported languages: `en`, `zh-Hans`.

## Firmwares

[MYNTEYE_BOX]: http://doc.myntai.com/mynteye/s/download

Get firmwares from our online disks: [MYNTEYE_BOX][]. The latest version is `2.0.1`.

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
