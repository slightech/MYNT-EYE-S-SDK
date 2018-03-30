#ifndef MYNTEYE_UVC_H_  // NOLINT
#define MYNTEYE_UVC_H_
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "mynteye/mynteye.h"

#define MYNTEYE_VID 0x04B4
#define MYNTEYE_PID 0x00F9

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

struct extension_unit {
  int unit;
};

struct context;  // Opaque type representing access to the underlying UVC
                 // implementation
struct device;   // Opaque type representing access to a specific UVC device

// Enumerate devices
std::shared_ptr<context> create_context();
std::vector<std::shared_ptr<device>> query_devices(
    std::shared_ptr<context> context);

// Static device properties
int get_vendor_id(const device &device);
int get_product_id(const device &device);

std::string get_video_name(const device &device);

// Access XU controls
void get_control(
    const device &device, const extension_unit &xu, uint8_t ctrl, void *data,
    int len);
void set_control(
    const device &device, const extension_unit &xu, uint8_t ctrl, void *data,
    int len);

// Control streaming
typedef std::function<void(const void *frame)> video_channel_callback;

void set_device_mode(
    device &device, int width, int height, uint32_t fourcc, int fps,  // NOLINT
    video_channel_callback callback);
void start_streaming(device &device, int num_transfer_bufs);  // NOLINT
void stop_streaming(device &device);                          // NOLINT

}  // namespace uvc

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_UVC_H_ NOLINT
