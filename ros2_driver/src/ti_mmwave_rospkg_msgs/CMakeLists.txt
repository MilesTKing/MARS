cmake_minimum_required(VERSION 3.5)
project(ti_mmwave_rospkg_msgs)

# Default to C++14
# if(NOT WIN32)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS"} -std=c++14 -Wall -Wextra)
# endif()

## dependencies

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

## Generate messages in the 'msg' folder
set(msg_files
   "msg/RadarScan.msg"
   "msg/RadarOccupancy.msg"
   "msg/RadarClassifier.msg"
   "msg/RadarTrackArray.msg"
   "msg/RadarTrackContents.msg"
   "msg/RadarPointTrackID.msg"
   "msg/RadarMicroDopplerDataArray.msg"
   "msg/RadarMicroDopplerDataContents.msg"
   "msg/RadarMicroDopplerFeatureArray.msg"
   "msg/RadarMicroDopplerFeatureContents.msg"
)

set(srv_files
   "srv/MmwaveCli.srv"
)

rosidl_generate_interfaces(ti_mmwave_rospkg_msgs
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
  visualization_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()