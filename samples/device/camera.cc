#include "glog_init.h"  // NOLINT

#include "device/context.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  glog_init _(argc, argv);

  Context context;

  return 0;
}
