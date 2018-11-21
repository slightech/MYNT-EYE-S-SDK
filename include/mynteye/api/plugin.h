// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef MYNTEYE_API_PLUGIN_H_
#define MYNTEYE_API_PLUGIN_H_
#pragma once

#include <cstdint>

#include <opencv2/core/core.hpp>

#include "mynteye/mynteye.h"

#ifndef MYNTEYE_PLUGIN_VERSION_CODE
#define MYNTEYE_PLUGIN_VERSION_CODE 0
#endif

MYNTEYE_BEGIN_NAMESPACE

class API;
struct Object;

/**
 * The plugin which could implement processing by yourself.
 */
class MYNTEYE_API Plugin {
 public:
  Plugin() = default;
  virtual ~Plugin() = 0;

  /**
   * Called when plugin created.
   * @param api the API instacne.
   */
  virtual void OnCreate(API *api) {
    api_ = api;
  }

  /**
   * Called when process rectify.
   * @param in input object.
   * @param out output object.
   * @return `true` if you process rectify.
   */
  virtual bool OnRectifyProcess(Object *const in, Object *const out) {
    MYNTEYE_UNUSED(in)
    MYNTEYE_UNUSED(out)
    return false;
  }

  /**
   * Called when process disparity.
   * @param in input object.
   * @param out output object.
   * @return `true` if you process disparity.
   */
  virtual bool OnDisparityProcess(Object *const in, Object *const out) {
    MYNTEYE_UNUSED(in)
    MYNTEYE_UNUSED(out)
    return false;
  }

  /**
   * Called when process normalized disparity.
   * @param in input object.
   * @param out output object.
   * @return `true` if you process normalized disparity.
   */
  virtual bool OnDisparityNormalizedProcess(
      Object *const in, Object *const out) {
    MYNTEYE_UNUSED(in)
    MYNTEYE_UNUSED(out)
    return false;
  }

  /**
   * Called when process points.
   * @param in input object.
   * @param out output object.
   * @return `true` if you process points.
   */
  virtual bool OnPointsProcess(Object *const in, Object *const out) {
    MYNTEYE_UNUSED(in)
    MYNTEYE_UNUSED(out)
    return false;
  }

  /**
   * Called when process depth.
   * @param in input object.
   * @param out output object.
   * @return `true` if you process depth.
   */
  virtual bool OnDepthProcess(Object *const in, Object *const out) {
    MYNTEYE_UNUSED(in)
    MYNTEYE_UNUSED(out)
    return false;
  }

 protected:
  API *api_;
};

inline Plugin::~Plugin() = default;

using plugin_version_code_t = std::uint32_t();
using plugin_create_t = Plugin *();
using plugin_destroy_t = void(Plugin *);

MYNTEYE_END_NAMESPACE

extern "C" {

/**
 * Get the plugin version code.
 */
MYNTEYE_API std::uint32_t plugin_version_code();

/**
 * Create the plugin.
 */
MYNTEYE_API mynteye::Plugin *plugin_create();

/**
 * Destroy the plugin.
 */
MYNTEYE_API void plugin_destroy(mynteye::Plugin *plugin);

}

#endif  // MYNTEYE_API_PLUGIN_H_
