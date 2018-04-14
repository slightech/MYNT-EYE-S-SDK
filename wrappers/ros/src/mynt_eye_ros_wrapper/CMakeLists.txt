cmake_minimum_required(VERSION 2.8.3)
project(mynt_eye_ros_wrapper)


get_filename_component(SDK_DIR "${PROJECT_SOURCE_DIR}/../../../.." ABSOLUTE)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# packages

function(checkPackage package customMessage)
  set(varName "${package}_FOUND")
  if (NOT "${${varName}}")
    string(REPLACE "_" "-" aptPackage ${package})
    if("${customMessage}" STREQUAL "")
      message(FATAL_ERROR "\n\n ${package} is missing, please try to install it with:\n sudo apt-get install ros-$(rosversion -d)-${aptPackage}\n\n")
    else()
      message(FATAL_ERROR "\n\n ${customMessage} \n\n")
    endif()
  endif()
endfunction(checkPackage)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  tf
)

checkPackage("cv_bridge" "")
checkPackage("geometry_msgs" "")
checkPackage("image_transport" "")
checkPackage("nodelet" "")
checkPackage("roscpp" "")
checkPackage("sensor_msgs" "")
checkPackage("std_msgs" "")
checkPackage("tf" "")

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport nodelet roscpp sensor_msgs std_msgs tf
)

get_filename_component(SDK_DIR "${PROJECT_SOURCE_DIR}/../../../.." ABSOLUTE)

LIST(APPEND CMAKE_PREFIX_PATH ${SDK_DIR}/third_party/glog/_build)
find_package(glog REQUIRED)
message(STATUS "Found glog: ${glog_VERSION}")

LIST(APPEND CMAKE_PREFIX_PATH ${SDK_DIR}/_install/lib/cmake)
find_package(mynteye REQUIRED)
message(STATUS "Found mynteye: ${mynteye_VERSION}")

# targets

add_compile_options(-std=c++11)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${SDK_DIR}/src  # for device layer interface
)

set(LINK_LIBS
  ${catkin_LIBRARIES}
  mynteye
)

add_library(mynteye_wrapper src/wrapper_nodelet.cc)
target_link_libraries(mynteye_wrapper ${LINK_LIBS})

add_executable(mynteye_wrapper_node src/wrapper_node.cc)
target_link_libraries(mynteye_wrapper_node mynteye_wrapper ${LINK_LIBS})

if(MSVC OR MSYS OR MINGW)
  target_compile_definitions(mynteye_wrapper_node
    PUBLIC GLOG_NO_ABBREVIATED_SEVERITIES
  )
endif()

# install

install(TARGETS mynteye_wrapper mynteye_wrapper_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
