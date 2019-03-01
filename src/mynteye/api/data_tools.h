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
#ifndef MYNTEYE_API_DATA_TOOLS_H_
#define MYNTEYE_API_DATA_TOOLS_H_
#pragma once
#include <opencv2/imgproc/imgproc.hpp>
#include "mynteye/api/object.h"
#include "mynteye/api/api.h"
#include "mynteye/device/device.h"
MYNTEYE_BEGIN_NAMESPACE
cv::Mat frame2mat(const std::shared_ptr<device::Frame> &frame);
api::StreamData data2api(const device::StreamData &data);
api::StreamData obj_data_first(const ObjMat2 *obj);
api::StreamData obj_data_second(const ObjMat2 *obj);
api::StreamData obj_data(const ObjMat *obj);
api::StreamData obj_data_first(const std::shared_ptr<ObjMat2> &obj);
api::StreamData obj_data_second(const std::shared_ptr<ObjMat2> &obj);
api::StreamData obj_data(const std::shared_ptr<ObjMat> &obj);
ObjMat data_obj(const api::StreamData &data);
ObjMat2 data_obj(const api::StreamData &first, const api::StreamData &second);
MYNTEYE_END_NAMESPACE
#endif  // MYNTEYE_API_DATA_TOOLS_H_
