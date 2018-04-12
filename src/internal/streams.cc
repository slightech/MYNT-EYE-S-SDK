#include "internal/streams.h"

#include <glog/logging.h>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <stdexcept>

#include "internal/types.h"

MYNTEYE_BEGIN_NAMESPACE

namespace {

bool unpack_stereo_img_data(
    const void *data, const StreamRequest &request, ImgData *img) {
  CHECK_NOTNULL(img);
  CHECK_EQ(request.format, Format::YUYV);

  auto data_new = reinterpret_cast<const std::uint8_t *>(data);
  std::size_t data_n =
      request.width * request.height * bytes_per_pixel(request.format);
  auto data_end = data_new + data_n;

  // LOG(INFO) << "ImagePacket (raw): header=0x" << std::hex <<
  // static_cast<int>(*(data_end - 1))
  //   << ", size=0x" << std::hex << static_cast<int>(*(data_end - 2))
  //   << ", frame_id="<< std::dec << ((*(data_end - 3) << 8) + *(data_end - 4))
  //   << ", timestamp="<< std::dec << ((*(data_end - 5) << 24) + (*(data_end -
  //   6) << 16) + (*(data_end - 7) << 8) + *(data_end - 8))
  //   << ", exposure_time="<< std::dec << ((*(data_end - 9) << 8) + *(data_end
  //   - 10))
  //   << ", checksum=0x" << std::hex << static_cast<int>(*(data_end - 11));

  std::size_t packet_n = sizeof(ImagePacket);
  // LOG(INFO) << "ImagePacket Size: " << packet_n;
  std::vector<std::uint8_t> packet(packet_n);
  std::reverse_copy(data_end - packet_n, data_end, packet.begin());

  ImagePacket img_packet(packet.data());
  // LOG(INFO) << "ImagePacket (new): header=0x" << std::hex <<
  // static_cast<int>(img_packet.header)
  //   << ", size=0x" << std::hex << static_cast<int>(img_packet.size)
  //   << ", frame_id="<< std::dec << img_packet.frame_id
  //   << ", timestamp="<< std::dec << img_packet.timestamp
  //   << ", exposure_time="<< std::dec << img_packet.exposure_time
  //   << ", checksum=0x" << std::hex << static_cast<int>(img_packet.checksum);

  if (img_packet.header != 0x3B) {
    LOG(WARNING) << "Image packet header must be 0x3B, but 0x" << std::hex
                 << std::uppercase << std::setw(2) << std::setfill('0')
                 << static_cast<int>(img_packet.header) << " now";
    return false;
  }

  std::uint8_t checksum = 0;
  for (std::size_t i = 2, n = packet_n - 2; i <= n; i++) {  // content: [2,9]
    checksum = (checksum ^ packet[i]);
  }
  if (checksum != img_packet.checksum) {
    LOG(WARNING) << "Image packet checksum should be 0x" << std::hex
                 << std::uppercase << std::setw(2) << std::setfill('0')
                 << static_cast<int>(checksum) << ", but 0x" << std::setw(2)
                 << std::setfill('0') << static_cast<int>(img_packet.checksum)
                 << " now";
    return false;
  }

  img->frame_id = img_packet.frame_id;
  img->timestamp = img_packet.timestamp;
  img->exposure_time = img_packet.exposure_time;
  return true;
}

bool unpack_left_img_pixels(
    const void *data, const StreamRequest &request, Streams::frame_t *frame) {
  CHECK_NOTNULL(frame);
  CHECK_EQ(request.format, Format::YUYV);
  CHECK_EQ(frame->format(), Format::GREY);
  auto data_new = reinterpret_cast<const std::uint8_t *>(data);
  std::size_t n = frame->width() * frame->height();
  for (std::size_t i = 0; i < n; i++) {
    frame->data()[i] = *(data_new + (i * 2));
  }
  return true;
}

bool unpack_right_img_pixels(
    const void *data, const StreamRequest &request, Streams::frame_t *frame) {
  CHECK_NOTNULL(frame);
  CHECK_EQ(request.format, Format::YUYV);
  CHECK_EQ(frame->format(), Format::GREY);
  auto data_new = reinterpret_cast<const std::uint8_t *>(data);
  std::size_t n = frame->width() * frame->height();
  for (std::size_t i = 0; i < n; i++) {
    frame->data()[i] = *(data_new + (i * 2 + 1));
  }
  return true;
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
      // alloc left
      AllocStreamData(Stream::LEFT, request, Format::GREY);
      auto &&left_data = stream_datas_map_[Stream::LEFT].back();
      // unpack img data
      if (unpack_img_data_map_[Stream::LEFT](
              data, request, left_data.img.get())) {
        // alloc right
        AllocStreamData(Stream::RIGHT, request, Format::GREY);
        auto &&right_data = stream_datas_map_[Stream::RIGHT].back();
        *right_data.img = *left_data.img;
        // unpack frame
        unpack_img_pixels_map_[Stream::LEFT](
            data, request, left_data.frame.get());
        unpack_img_pixels_map_[Stream::RIGHT](
            data, request, right_data.frame.get());
      } else {
        // discard left
        DiscardStreamData(Stream::LEFT);
        LOG(WARNING) << "Image packet is unaccepted, frame dropped";
      }
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

void Streams::ConfigStreamLimits(
    const Stream &stream, std::size_t max_data_size) {
  CHECK_GT(max_data_size, 0);
  stream_limits_map_[stream] = max_data_size;
}

std::size_t Streams::GetStreamDataMaxSize(const Stream &stream) const {
  try {
    return stream_limits_map_.at(stream);
  } catch (const std::out_of_range &e) {
    return 4;  // default stream data max size
  }
}

Streams::stream_datas_t Streams::GetStreamDatas(const Stream &stream) {
  std::unique_lock<std::mutex> lock(mtx_);
  if (!HasStreamDatas(stream) || stream_datas_map_.at(stream).empty()) {
    LOG(WARNING) << "There are stream datas of " << stream
                 << ", do you first call WaitForStreams?";
    return {};
  }
  stream_datas_t datas = stream_datas_map_.at(stream);
  stream_datas_map_[stream].clear();
  return datas;
}

Streams::stream_data_t Streams::GetLatestStreamData(const Stream &stream) {
  return GetStreamDatas(stream).back();
}

const Streams::stream_datas_t &Streams::stream_datas(const Stream &stream) {
  try {
    return stream_datas_map_.at(stream);
  } catch (const std::out_of_range &e) {
    // Add empty vector of this stream key
    stream_datas_map_[stream] = {};
    return stream_datas_map_.at(stream);
  }
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
  stream_data_t data;
  if (stream == Stream::LEFT || stream == Stream::RIGHT) {
    data.img = std::make_shared<ImgData>();
  } else {
    data.img = nullptr;
  }
  data.frame =
      std::make_shared<frame_t>(request.width, request.height, format, nullptr);
  stream_datas_map_[stream].push_back(data);
  // If cached more then limits_max, drop the oldest one.
  if (stream_datas_map_.at(stream).size() > GetStreamDataMaxSize(stream)) {
    auto &&datas = stream_datas_map_[stream];
    datas.erase(datas.begin());
    VLOG(2) << "Stream data of " << stream << " is dropped as out of limits";
  }
}

void Streams::DiscardStreamData(const Stream &stream) {
  // Must discard after alloc, otherwise at will out of range when no this key.
  if (stream_datas_map_.at(stream).size() > 0) {
    auto &&datas = stream_datas_map_[stream];
    datas.erase(datas.end() - 1);
  } else {
    VLOG(2) << "Stream data of " << stream << " is empty, could not discard";
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
