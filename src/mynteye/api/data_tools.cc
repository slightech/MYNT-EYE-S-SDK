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

#include <algorithm>
#include <functional>
#include <stdexcept>
#include "mynteye/api/data_tools.h"
#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

cv::Mat frame2mat(const std::shared_ptr<device::Frame> &frame) {
  if (frame->format() == Format::YUYV) {
    cv::Mat img(frame->height(), frame->width(), CV_8UC2, frame->data());
    cv::cvtColor(img, img, cv::COLOR_YUV2BGR_YUY2);
    return img;
  } else if (frame->format() == Format::BGR888) {
    cv::Mat img(frame->height(), frame->width(), CV_8UC3, frame->data());
    return img;
  } else {  // Format::GRAY
    return cv::Mat(frame->height(), frame->width(), CV_8UC1, frame->data());
  }
}

api::StreamData data2api(const device::StreamData &data) {
  return {data.img, frame2mat(data.frame), data.frame, data.frame_id};
}

// ObjMat/ObjMat2 > api::StreamData

api::StreamData obj_data_first(const ObjMat2 *obj) {
  return {obj->first_data, obj->first, nullptr, obj->first_id};
}

api::StreamData obj_data_second(const ObjMat2 *obj) {
  return {obj->second_data, obj->second, nullptr, obj->second_id};
}

api::StreamData obj_data(const ObjMat *obj) {
  return {obj->data, obj->value, nullptr, obj->id};
}

api::StreamData obj_data_first(const std::shared_ptr<ObjMat2> &obj) {
  return {obj->first_data, obj->first, nullptr, obj->first_id};
}

api::StreamData obj_data_second(const std::shared_ptr<ObjMat2> &obj) {
  return {obj->second_data, obj->second, nullptr, obj->second_id};
}

api::StreamData obj_data(const std::shared_ptr<ObjMat> &obj) {
  return {obj->data, obj->value, nullptr, obj->id};
}

// api::StreamData > ObjMat/ObjMat2

ObjMat data_obj(const api::StreamData &data) {
  return ObjMat{data.frame, data.frame_id, data.img};
}

ObjMat2 data_obj(const api::StreamData &first, const api::StreamData &second) {
  return ObjMat2{
      first.frame, first.frame_id, first.img,
      second.frame, second.frame_id, second.img};
}

MYNTEYE_END_NAMESPACE
