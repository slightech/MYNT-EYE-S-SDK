#ifndef MYNTEYE_UVC_H_  // NOLINT
#define MYNTEYE_UVC_H_
#pragma once

#include <memory>
#include <vector>

#include "mynteye/mynteye.h"

#define MYNTEYE_VID 0x04B4
#define MYNTEYE_PID 0x00F9

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

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

}  // namespace uvc

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_UVC_H_ NOLINT
