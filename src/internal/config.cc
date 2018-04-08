#include "internal/config.h"

MYNTEYE_BEGIN_NAMESPACE

const std::map<Model, StreamSupports> stream_supports_map = {
    {Model::STANDARD, {Stream::LEFT, Stream::RIGHT}}};

const std::map<Model, CapabilitiesSupports> capabilities_supports_map = {
    {Model::STANDARD, {Capabilities::STEREO, Capabilities::IMU}}};

const std::map<Model, OptionSupports> option_supports_map = {
    {Model::STANDARD,
     {Option::GAIN, Option::BRIGHTNESS, Option::CONTRAST, Option::FRAME_RATE,
      Option::IMU_FREQUENCY, Option::EXPOSURE_MODE, Option::MAX_GAIN,
      Option::MAX_EXPOSURE_TIME, Option::DESIRED_BRIGHTNESS, Option::IR_CONTROL,
      Option::HDR_MODE, Option::ZERO_DRIFT_CALIBRATION, Option::ERASE_CHIP}}};

const std::map<Model, std::map<Capabilities, StreamRequests>>
    stream_requests_map = {
        {Model::STANDARD,
         {{Capabilities::STEREO, {{752, 480, Format::YUYV, 25}}}}}};

MYNTEYE_END_NAMESPACE
