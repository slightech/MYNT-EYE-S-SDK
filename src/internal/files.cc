#include "internal/files.h"

#include <glog/logging.h>

#if defined(OS_WIN) && !defined(OS_MINGW) && !defined(OS_CYGWIN)
#include <direct.h>
#else
#include <sys/stat.h>
#endif

MYNTEYE_BEGIN_NAMESPACE

namespace files {

bool mkdir(const std::string &path) {
#if defined(OS_MINGW) || defined(OS_CYGWIN)
  const int status = ::mkdir(path.c_str());
#elif defined(OS_WIN)
  const int status = ::_mkdir(path.c_str());
#else
  const int status =
      ::mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
  if (status != 0 && errno != EEXIST) {
    VLOG(2) << "Create directory failed (status " << status
            << "), path: " << path;
    return false;
  }
  if (errno == EEXIST) {
    VLOG(2) << "Create directory needless (already exist), path: " << path;
    return true;
  } else {
    VLOG(2) << "Create directory success, path: " << path;
    return true;
  }
}

}  // namespace files

MYNTEYE_END_NAMESPACE
