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

/*
struct uvc_xu_control_query

__u8    unit            Extension unit ID
__u8    selector        Control selector
__u8    query           Request code to send to the device
__u16   size            Control data size (in bytes)
__u8    *data           Control value
*/

// extension unit
struct xu {
  uint8_t unit = 3;
};

typedef enum xu_query {
  XU_SET_CUR,  // Set current value of the control
  XU_GET_CUR,  // Get current value of the control
  XU_GET_MIN,  // Get min value of the control
  XU_GET_MAX,  // Get max value of the control
  XU_GET_DEF,  // Get default value of the control
  XU_LAST
} xu_query;

struct context;  // Opaque type representing access to the underlying UVC
                 // implementation
struct device;   // Opaque type representing access to a specific UVC device

// Enumerate devices
std::shared_ptr<context> create_context();
std::vector<std::shared_ptr<device>> query_devices(
    std::shared_ptr<context> context);

// Static device properties
std::string get_name(const device &device);
int get_vendor_id(const device &device);
int get_product_id(const device &device);

std::string get_video_name(const device &device);

// Access XU controls
bool xu_control_query(
    const device &device, const xu &xu, uint8_t selector, xu_query query,
    uint16_t size, uint8_t *data);

// Control streaming
typedef std::function<void(const void *frame)> video_channel_callback;

void set_device_mode(
    device &device, int width, int height, int fourcc, int fps,  // NOLINT
    video_channel_callback callback);
void start_streaming(device &device, int num_transfer_bufs);  // NOLINT
void stop_streaming(device &device);                          // NOLINT

}  // namespace uvc

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_UVC_H_ NOLINT
