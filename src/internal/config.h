#ifndef MYNTEYE_INTERNAL_CONFIG_H_  // NOLINT
#define MYNTEYE_INTERNAL_CONFIG_H_
#pragma once

#include <map>
#include <set>

#include "mynteye/mynteye.h"
#include "mynteye/types.h"

MYNTEYE_BEGIN_NAMESPACE

using StreamSupports = std::set<Stream>;
using CapabilitiesSupports = std::set<Capabilities>;
using OptionSupports = std::set<Option>;

extern const std::map<Model, StreamSupports> stream_supports_map;
extern const std::map<Model, CapabilitiesSupports> capabilities_supports_map;
extern const std::map<Model, OptionSupports> option_supports_map;

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_CONFIG_H_ NOLINT
