#pragma once

#include "mynteye/mynteye.h"

#include <string>
#include <vector>

MYNTEYE_BEGIN_NAMESPACE

struct MYNTEYE_API UsbInfo {
  int vid;
  int pid;
  int fd;
  int busnum;
  int devaddr;
  std::string usbfs;
  std::string name;
  std::string serial;
};

MYNTEYE_API void set_usb_infos(const std::vector<UsbInfo> &infos);
MYNTEYE_API std::vector<UsbInfo> get_usb_infos();

MYNTEYE_END_NAMESPACE
