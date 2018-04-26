#ifndef MYNTEYE_SYNTHETIC_H_  // NOLINT
#define MYNTEYE_SYNTHETIC_H_
#pragma once

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

class API;

class Synthetic {
 public:
  explicit Synthetic(API *api);
  ~Synthetic();

 private:
  API *api_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_SYNTHETIC_H_ NOLINT
