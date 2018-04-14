#include <nodelet/loader.h>
#include <ros/ros.h>

#include "mynteye/glog_init.h"

int main(int argc, char *argv[]) {
  glog_init _(argc, argv);

  ros::init(argc, argv, "mynteye_wrapper_node");
  ros::console::set_logger_level(
      ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  nodelet.load(
      ros::this_node::getName(), "mynteye/ROSWrapperNodelet", remap, nargv);

  ros::spin();

  return 0;
}
