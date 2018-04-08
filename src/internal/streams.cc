#include "internal/streams.h"

#include <glog/logging.h>

#include <algorithm>
#include <chrono>

MYNTEYE_BEGIN_NAMESPACE

namespace {

void unpack_stereo_img_data(
    const void *data, const StreamRequest &request, ImgData &img) {  // NOLINT
  UNUSED(data)
  UNUSED(request)
  UNUSED(img)
}

void unpack_left_img_pixels(
    const void *data, const StreamRequest &request,
    Streams::frame_t &frame) {  // NOLINT
  CHECK_EQ(request.format, Format::YUYV);
  CHECK_EQ(frame.format(), Format::GREY);
  auto data_new = reinterpret_cast<const std::uint8_t *>(data);
  std::size_t n = frame.width() * frame.height();
  for (std::size_t i = 0; i < n; i++) {
    frame.data()[i] = *(data_new + (i * 2));
  }
}

void unpack_right_img_pixels(
    const void *data, const StreamRequest &request,
    Streams::frame_t &frame) {  // NOLINT
  CHECK_EQ(request.format, Format::YUYV);
  CHECK_EQ(frame.format(), Format::GREY);
  auto data_new = reinterpret_cast<const std::uint8_t *>(data);
  std::size_t n = frame.width() * frame.height();
  for (std::size_t i = 0; i < n; i++) {
    frame.data()[i] = *(data_new + (i * 2 + 1));
  }
}

}  // namespace

Streams::Streams(const std::vector<Stream> key_streams)
    : key_streams_(key_streams),
      stream_capabilities_(
          {Capabilities::STEREO, Capabilities::COLOR, Capabilities::DEPTH,
           Capabilities::POINTS, Capabilities::FISHEYE, Capabilities::INFRARED,
           Capabilities::INFRARED2}),
      unpack_img_data_map_(
          {{Stream::LEFT, unpack_stereo_img_data},
           {Stream::RIGHT, unpack_stereo_img_data}}),
      unpack_img_pixels_map_(
          {{Stream::LEFT, unpack_left_img_pixels},
           {Stream::RIGHT, unpack_right_img_pixels}}) {
  VLOG(2) << __func__;
}

Streams::~Streams() {
  VLOG(2) << __func__;
}

void Streams::ConfigStream(
    const Capabilities &capability, const StreamRequest &request) {
  if (!IsStreamCapability(capability)) {
    LOG(FATAL) << "Cannot config stream without stream capability";
  }
  VLOG(2) << "Config stream request of " << capability << ", " << request;
  stream_config_requests_[capability] = request;
}

void Streams::PushStream(const Capabilities &capability, const void *data) {
  if (!HasStreamConfigRequest(capability)) {
    LOG(FATAL) << "Cannot push stream without stream config request";
  }
  std::unique_lock<std::mutex> lock(mtx_);
  auto &&request = GetStreamConfigRequest(capability);
  switch (capability) {
    case Capabilities::STEREO: {
      // alloc
      AllocStreamData(Stream::LEFT, request, Format::GREY);
      AllocStreamData(Stream::RIGHT, request, Format::GREY);
      auto &&left_data = stream_datas_map_[Stream::LEFT].back();
      auto &&right_data = stream_datas_map_[Stream::RIGHT].back();
      // unpack img data
      unpack_img_data_map_[Stream::LEFT](data, request, *left_data.img);
      right_data.img = left_data.img;
      // unpack frame
      unpack_img_pixels_map_[Stream::LEFT](data, request, *left_data.frame);
      unpack_img_pixels_map_[Stream::RIGHT](data, request, *right_data.frame);
    } break;
    default:
      LOG(FATAL) << "Not supported " << capability << " now";
  }
  if (HasKeyStreamDatas())
    cv_.notify_one();
}

void Streams::WaitForStreams() {
  std::unique_lock<std::mutex> lock(mtx_);
  auto ready = std::bind(&Streams::HasKeyStreamDatas, this);
  if (!ready() && !cv_.wait_for(lock, std::chrono::seconds(2), ready)) {
    LOG(FATAL) << "Timeout waiting for key frames";
  }
}

Streams::stream_datas_t Streams::GetStreamDatas(const Stream &stream) {
  if (!HasStreamDatas(stream) || stream_datas_map_.at(stream).empty()) {
    LOG(WARNING) << "There are stream datas of " << stream
                 << ", do you first call WaitForStreams?";
    return {};
  }
  std::unique_lock<std::mutex> lock(mtx_);
  stream_datas_t datas = stream_datas_map_.at(stream);
  stream_datas_map_[stream].clear();
  return datas;
}

Streams::stream_data_t Streams::GetLatestStreamData(const Stream &stream) {
  return GetStreamDatas(stream).back();
}

const Streams::stream_datas_t &Streams::stream_datas(
    const Stream &stream) const {
  return stream_datas_map_.at(stream);
}

bool Streams::IsStreamCapability(const Capabilities &capability) const {
  return std::find(
             stream_capabilities_.begin(), stream_capabilities_.end(),
             capability) != stream_capabilities_.end();
}

bool Streams::HasStreamConfigRequest(const Capabilities &capability) const {
  return stream_config_requests_.find(capability) !=
         stream_config_requests_.end();
}

const StreamRequest &Streams::GetStreamConfigRequest(
    const Capabilities &capability) const {
  return stream_config_requests_.at(capability);
}

bool Streams::HasStreamDatas(const Stream &stream) const {
  return stream_datas_map_.find(stream) != stream_datas_map_.end();
}

void Streams::AllocStreamData(
    const Stream &stream, const StreamRequest &request) {
  AllocStreamData(stream, request, request.format);
}

void Streams::AllocStreamData(
    const Stream &stream, const StreamRequest &request, const Format &format) {
  static std::size_t stream_data_limits_max = 4;
  stream_data_t data;
  data.img = std::shared_ptr<ImgData>(new ImgData{0, 0, 0});
  data.frame =
      std::make_shared<frame_t>(request.width, request.height, format, nullptr);
  stream_datas_map_[stream].push_back(data);
  // If cached more then limits_max, drop the oldest one.
  if (stream_datas_map_.at(stream).size() > stream_data_limits_max) {
    auto &&datas = stream_datas_map_[stream];
    datas.erase(datas.begin());
    VLOG(2) << "Stream data of " << stream << " is dropped";
  }
}

bool Streams::HasKeyStreamDatas() const {
  for (auto &&s : key_streams_) {
    if (!HasStreamDatas(s))
      return false;
    if (stream_datas_map_.at(s).empty())
      return false;
  }
  return true;
}

MYNTEYE_END_NAMESPACE
