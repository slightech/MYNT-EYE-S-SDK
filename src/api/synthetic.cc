#include "api/synthetic.h"

#include <glog/logging.h>

#include "api/api.h"

MYNTEYE_BEGIN_NAMESPACE

Synthetic::Synthetic(API *api) : api_(api) {
  VLOG(2) << __func__;
}

Synthetic::~Synthetic() {
  VLOG(2) << __func__;
}

MYNTEYE_END_NAMESPACE
