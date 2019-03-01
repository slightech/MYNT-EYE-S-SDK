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
#include <vector>

// #include <boost/python.hpp>
#include <boost/python/class.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/list.hpp>
#include <boost/python/module.hpp>
#include <boost/python/operators.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <opencv2/core/core.hpp>

#include "mynteye/logger.h"
#include "mynteye/api/api.h"
#include "mynteye/device/utils.h"

#include "array_indexing_suite.hpp"
#include "array_ref.hpp"

// #define PY_ARRAY_UNIQUE_SYMBOL pbcvt_ARRAY_API
// #include "pyboostcvconverter/pyboostcvconverter.hpp"
#include "np_opencv_converter.hpp"

#define ENUM_EXPORT_VALUES

namespace bp = boost::python;

namespace {

template <typename T>
inline void std_vector_assign(
    std::vector<T> &l, const bp::object &o) {  // NOLINT
  l.assign(bp::stl_input_iterator<T>(o), bp::stl_input_iterator<T>());
}

template <typename T>
inline std::vector<T> py_list_to_std_vector(const bp::object &o) {
  return std::vector<T>(
      bp::stl_input_iterator<T>(o), bp::stl_input_iterator<T>());
}

template <typename T>
inline bp::list std_vector_to_py_list(const std::vector<T> &v) {
  bp::list l;
  for (auto &&val : v) {
    l.append(val);
  }
  return l;
}

template <typename Container>
char **new_cstrings(const Container &strings, std::size_t n) {
  char **cstrings = new char *[n];
  for (std::size_t i = 0; i < n; i++) {
    cstrings[i] = new char[strings[i].size() + 1];
    std::strcpy(cstrings[i], strings[i].c_str());  // NOLINT
  }
  return cstrings;
}

void del_cstrings(char **cstrings, std::size_t n) {
  for (std::size_t i = 0; i < n; i++) {
    delete[] cstrings[i];
  }
  delete[] cstrings;
}

}  // namespace

MYNTEYE_BEGIN_NAMESPACE

namespace python {

// api wrapper

struct MYNTEYE_API StreamData {
  /** ImgData. */
  ImgData img;
  /** Frame. */
  PyObject *frame;

  bool operator==(const StreamData &other) const {
    return img.frame_id == other.img.frame_id &&
           img.timestamp == other.img.timestamp;
  }
};

struct MYNTEYE_API MotionData {
  /** ImuData. */
  ImuData imu;

  bool operator==(const MotionData &other) const {
    return imu.timestamp == other.imu.timestamp;
  }
};

class MYNTEYE_API APIWrap : public API {
 public:
  explicit APIWrap(std::shared_ptr<Device> device) : API(device) {}
  ~APIWrap() {}

  static std::shared_ptr<APIWrap> Create() {
    auto &&device = device::select();
    if (!device)
      return nullptr;
    return std::make_shared<APIWrap>(device);
  }

  static std::shared_ptr<APIWrap> Create(int argc, char *argv[]) {
    static glog_init _(argc, argv);
    auto &&device = device::select();
    if (!device)
      return nullptr;
    return std::make_shared<APIWrap>(device);
  }

  python::StreamData GetStreamData(const Stream &stream) {
    auto &&data = API::GetStreamData(stream);
    // return {*data.img, pbcvt::fromMatToNDArray(data.frame)};
    return {*data.img,
            fs::python::Mat_to_PyObject<cv::Mat>::convert(data.frame)};
  }

  std::vector<python::StreamData> GetStreamDatas(const Stream &stream) {
    std::vector<python::StreamData> datas;
    for (auto &&data : API::GetStreamDatas(stream)) {
      // datas.push_back({*data.img, pbcvt::fromMatToNDArray(data.frame)});
      datas.push_back(
          {*data.img,
           fs::python::Mat_to_PyObject<cv::Mat>::convert(data.frame)});
    }
    return datas;
  }

  std::vector<python::MotionData> GetMotionDatas() {
    std::vector<python::MotionData> datas;
    for (auto &&data : API::GetMotionDatas()) {
      datas.push_back({*data.imu});
    }
    return datas;
  }
};

// api create static methods

std::shared_ptr<APIWrap> (*api_create_1)() = &APIWrap::Create;

std::shared_ptr<APIWrap> api_create_2(bp::list argv) {
  auto &&args = py_list_to_std_vector<std::string>(argv);
  auto &&n = args.size();
  if (n == 0) {
    return APIWrap::Create();
  }
  char **cstrings = new_cstrings(args, n);
  auto &&api = APIWrap::Create(args.size(), cstrings);
  del_cstrings(cstrings, n);
  return api;
}

// glog_init create static methods

std::shared_ptr<glog_init> glog_init_create(bp::list argv) {
  auto &&args = py_list_to_std_vector<std::string>(argv);
  auto &&n = args.size();
  assert(n > 0);
  char **cstrings = new_cstrings(args, n);
  auto &&ret = std::make_shared<glog_init>(args.size(), cstrings);
  del_cstrings(cstrings, n);
  return ret;
}

// BOOST_PYTHON_MODULE

BOOST_PYTHON_MODULE(mynteye_py) {
  /*
  Py_Initialize();
  import_array();
  bp::to_python_converter<cv::Mat, pbcvt::matToNDArrayBoostConverter>();
  pbcvt::matFromNDArrayBoostConverter();
  */
  fs::python::init_and_export_converters();
  py::scope scope = py::scope();

  bp::class_<array_ref<double>>("DoubleArray")
      .def(array_indexing_suite<array_ref<double>>());

  bp::class_<array_ref<array_ref<double>>>("Double2DArray")
      .def(array_indexing_suite<array_ref<array_ref<double>>>{});

  // types.h - enums

  bp::enum_<Model>("Model")
      .value("STANDARD", Model::STANDARD)
#ifdef ENUM_EXPORT_VALUES
      .export_values()
#endif
      ;  // NOLINT

  bp::enum_<Stream>("Stream")
      .value("LEFT", Stream::LEFT)
      .value("RIGHT", Stream::RIGHT)
      .value("LEFT_RECTIFIED", Stream::LEFT_RECTIFIED)
      .value("RIGHT_RECTIFIED", Stream::RIGHT_RECTIFIED)
      .value("DISPARITY", Stream::DISPARITY)
      .value("DISPARITY_NORMALIZED", Stream::DISPARITY_NORMALIZED)
      .value("DEPTH", Stream::DEPTH)
      .value("POINTS", Stream::POINTS)
#ifdef ENUM_EXPORT_VALUES
      .export_values()
#endif
      ;  // NOLINT

  bp::enum_<Capabilities>("Capabilities")
      .value("STEREO", Capabilities::STEREO)
      .value("COLOR", Capabilities::COLOR)
      .value("DEPTH", Capabilities::DEPTH)
      .value("POINTS", Capabilities::POINTS)
      .value("FISHEYE", Capabilities::FISHEYE)
      .value("INFRARED", Capabilities::INFRARED)
      .value("INFRARED2", Capabilities::INFRARED2)
      .value("IMU", Capabilities::IMU)
#ifdef ENUM_EXPORT_VALUES
      .export_values()
#endif
      ;  // NOLINT

  bp::enum_<Info>("Info")
      .value("DEVICE_NAME", Info::DEVICE_NAME)
      .value("SERIAL_NUMBER", Info::SERIAL_NUMBER)
      .value("FIRMWARE_VERSION", Info::FIRMWARE_VERSION)
      .value("HARDWARE_VERSION", Info::HARDWARE_VERSION)
      .value("SPEC_VERSION", Info::SPEC_VERSION)
      .value("LENS_TYPE", Info::LENS_TYPE)
      .value("IMU_TYPE", Info::IMU_TYPE)
      .value("NOMINAL_BASELINE", Info::NOMINAL_BASELINE)
      .value("AUXILIARY_CHIP_VERSION", Info::AUXILIARY_CHIP_VERSION)
      .value("ISP_VERSION", Info::ISP_VERSION)
#ifdef ENUM_EXPORT_VALUES
      .export_values()
#endif
      ;  // NOLINT

  bp::enum_<Option>("Option")
      .value("GAIN", Option::GAIN)
      .value("BRIGHTNESS", Option::BRIGHTNESS)
      .value("CONTRAST", Option::CONTRAST)
      .value("FRAME_RATE", Option::FRAME_RATE)
      .value("IMU_FREQUENCY", Option::IMU_FREQUENCY)
      .value("EXPOSURE_MODE", Option::EXPOSURE_MODE)
      .value("MAX_GAIN", Option::MAX_GAIN)
      .value("MAX_EXPOSURE_TIME", Option::MAX_EXPOSURE_TIME)
      .value("DESIRED_BRIGHTNESS", Option::DESIRED_BRIGHTNESS)
      .value("IR_CONTROL", Option::IR_CONTROL)
      .value("HDR_MODE", Option::HDR_MODE)
      .value("ZERO_DRIFT_CALIBRATION", Option::ZERO_DRIFT_CALIBRATION)
      .value("ERASE_CHIP", Option::ERASE_CHIP)
#ifdef ENUM_EXPORT_VALUES
      .export_values()
#endif
      ;  // NOLINT

  bp::enum_<Source>("Source")
      .value("VIDEO_STREAMING", Source::VIDEO_STREAMING)
      .value("MOTION_TRACKING", Source::MOTION_TRACKING)
      .value("ALL", Source::ALL)
#ifdef ENUM_EXPORT_VALUES
      .export_values()
#endif
      ;  // NOLINT

  bp::enum_<AddOns>("AddOns")
      .value("INFRARED", AddOns::INFRARED)
      .value("INFRARED2", AddOns::INFRARED2)
#ifdef ENUM_EXPORT_VALUES
      .export_values()
#endif
      ;  // NOLINT

  bp::enum_<Format>("Format")
      .value("GREY", Format::GREY)
      .value("YUYV", Format::YUYV)
#ifdef ENUM_EXPORT_VALUES
      .export_values()
#endif
      ;  // NOLINT

  // types.h - types

  bp::class_<StreamRequest>("StreamRequest")
      .def_readonly("width", &StreamRequest::width)
      .def_readonly("height", &StreamRequest::height)
      .def_readonly("format", &StreamRequest::format)
      .def_readonly("fps", &StreamRequest::fps)
      .def(bp::self == bp::self)
      .def(bp::self != bp::self)
      .def(bp::self_ns::str(bp::self));

  bp::class_<std::vector<StreamRequest>>("StreamRequestVec")
      .def(bp::vector_indexing_suite<std::vector<StreamRequest>>());

  bp::class_<Intrinsics>("Intrinsics")
      .def_readonly("width", &Intrinsics::width)
      .def_readonly("height", &Intrinsics::height)
      .def_readonly("fx", &Intrinsics::fx)
      .def_readonly("fy", &Intrinsics::fy)
      .def_readonly("cx", &Intrinsics::cx)
      .def_readonly("cy", &Intrinsics::cy)
      .def_readonly("model", &Intrinsics::model)
      .add_property(
          "coeffs", +[](Intrinsics *o) { return array_ref<double>{o->coeffs}; })
      .def(bp::self_ns::str(bp::self));

  bp::class_<ImuIntrinsics>("ImuIntrinsics")
      .add_property(
          "scale",
          +[](ImuIntrinsics *o) {
            return array_ref<array_ref<double>>{o->scale};
          })
      .add_property(
          "drift",
          +[](ImuIntrinsics *o) { return array_ref<double>{o->drift}; })
      .add_property(
          "noise",
          +[](ImuIntrinsics *o) { return array_ref<double>{o->noise}; })
      .add_property(
          "bias", +[](ImuIntrinsics *o) { return array_ref<double>{o->bias}; })
      .def(bp::self_ns::str(bp::self));

  bp::class_<MotionIntrinsics>("MotionIntrinsics")
      .def_readonly("accel", &MotionIntrinsics::accel)
      .def_readonly("gyro", &MotionIntrinsics::gyro)
      .def(bp::self_ns::str(bp::self));

  bp::class_<Extrinsics>("Extrinsics")
      .add_property(
          "rotation",
          +[](Extrinsics *o) {
            return array_ref<array_ref<double>>{o->rotation};
          })
      .add_property(
          "translation",
          +[](Extrinsics *o) { return array_ref<double>{o->translation}; })
      .def("inverse", &Extrinsics::Inverse)
      .def(bp::self_ns::str(bp::self));

  bp::class_<ImgData>("ImgData")
      .def_readonly("frame_id", &ImgData::frame_id)
      .def_readonly("timestamp", &ImgData::timestamp)
      .def_readonly("exposure_time", &ImgData::exposure_time);

  // bp::register_ptr_to_python<std::shared_ptr<ImgData>>();

  bp::class_<ImuData>("ImuData")
      .def_readonly("timestamp", &ImuData::timestamp)
      .add_property(
          "accel", +[](ImuData *o) { return array_ref<double>{o->accel}; })
      .add_property(
          "gyro", +[](ImuData *o) { return array_ref<double>{o->gyro}; })
      .def_readonly("temperature", &ImuData::temperature);

  // bp::register_ptr_to_python<std::shared_ptr<ImuData>>();

  bp::class_<OptionInfo>("OptionInfo")
      .def_readonly("min", &OptionInfo::min)
      .def_readonly("max", &OptionInfo::max)
      .def_readonly("def", &OptionInfo::def)
      .def(bp::self_ns::str(bp::self));

  // api.h - API

  bp::class_<python::StreamData>("StreamData")
      .def_readonly("img", &python::StreamData::img)
      .def_readonly("frame", &python::StreamData::frame);

  bp::class_<std::vector<python::StreamData>>("StreamDataVec")
      .def(bp::vector_indexing_suite<std::vector<python::StreamData>>());

  bp::class_<python::MotionData>("MotionData")
      .def_readonly("imu", &python::MotionData::imu);

  bp::class_<std::vector<python::MotionData>>("MotionDataVec")
      .def(bp::vector_indexing_suite<std::vector<python::MotionData>>());

  bool (APIWrap::*supports_stream)(const Stream &) const = &APIWrap::Supports;
  bool (APIWrap::*supports_capabilities)(const Capabilities &) const =
      &APIWrap::Supports;
  bool (APIWrap::*supports_option)(const Option &) const = &APIWrap::Supports;
  bool (APIWrap::*supports_addons)(const AddOns &) const = &APIWrap::Supports;

  python::StreamData (APIWrap::*get_stream_data)(const Stream &) =
      &APIWrap::GetStreamData;
  std::vector<python::StreamData> (APIWrap::*get_stream_datas)(const Stream &) =
      &APIWrap::GetStreamDatas;
  std::vector<python::MotionData> (APIWrap::*get_motion_datas)() =
      &APIWrap::GetMotionDatas;

  bp::class_<APIWrap, boost::noncopyable>("API", bp::no_init)
      .def("create", api_create_1)
      .def("create", &api_create_2)
      .staticmethod("create")
      .add_property("model", &APIWrap::GetModel)
      .def("supports", supports_stream)
      .def("supports", supports_capabilities)
      .def("supports", supports_option)
      .def("supports", supports_addons)
      .def(
          "get_stream_requests", &APIWrap::GetStreamRequests,
          bp::return_value_policy<bp::reference_existing_object>())
      .def("config_stream_request", &APIWrap::ConfigStreamRequest)
      .def("get_info", &APIWrap::GetInfo)
      .def("get_intrinsics", &APIWrap::GetIntrinsics)
      .def("get_extrinsics", &APIWrap::GetExtrinsics)
      .def("get_motion_intrinsics", &APIWrap::GetMotionIntrinsics)
      .def("get_motion_extrinsics", &APIWrap::GetMotionExtrinsics)
      .def("log_option_infos", &APIWrap::LogOptionInfos)
      .def("get_option_info", &APIWrap::GetOptionInfo)
      .def("get_option_value", &APIWrap::GetOptionValue)
      .def("set_option_value", &APIWrap::SetOptionValue)
      .def("run_option_action", &APIWrap::RunOptionAction)
      // .def("set_stream_callback", &APIWrap::SetStreamCallback)
      // .def("set_motion_callback", &APIWrap::SetMotionCallback)
      // .def("has_stream_callback", &APIWrap::HasStreamCallback)
      // .def("has_motion_callback", &APIWrap::HasMotionCallback)
      .def("start", &APIWrap::Start)
      .def("stop", &APIWrap::Stop)
      .def("wait_for_streams", &APIWrap::WaitForStreams)
      .def("enable_stream_data", &APIWrap::EnableStreamData)
      .def("disable_stream_data", &APIWrap::DisableStreamData)
      .def("get_stream_data", get_stream_data)
      .def("get_stream_datas", get_stream_datas)
      .def(
          "enable_motion_datas", &APIWrap::EnableMotionDatas,
          (bp::arg("max_size") = std::numeric_limits<std::size_t>::max()))
      .def("get_motion_datas", get_motion_datas)
      .def("enable_plugin", &APIWrap::EnablePlugin);

  bp::register_ptr_to_python<std::shared_ptr<APIWrap>>();

  // logger.h - glog_init

  bp::class_<glog_init, boost::noncopyable>("glog_init", bp::no_init)
      .def("create", &glog_init_create)
      .staticmethod("create");

  bp::register_ptr_to_python<std::shared_ptr<glog_init>>();
}

}  // namespace python

MYNTEYE_END_NAMESPACE
