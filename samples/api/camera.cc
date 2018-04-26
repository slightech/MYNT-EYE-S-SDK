#include "mynteye/glog_init.h"

#include "mynteye/api.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  glog_init _(argc, argv);

  auto &&api = API::Create();

  return 0;
}
