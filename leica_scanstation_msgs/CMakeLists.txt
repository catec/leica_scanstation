project(leica_scanstation_msgs)
cmake_minimum_required(VERSION 3.5)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  std_srvs
  sensor_msgs
  message_generation
)

add_service_files(
  FILES 
  PointCloudFile.srv 
  MotorPose.srv 
  Scan.srv
)

generate_messages(
  DEPENDENCIES std_msgs sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)
