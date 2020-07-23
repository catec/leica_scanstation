cmake_minimum_required(VERSION 2.8.3)
project(leica_scanstation_ros)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
# SET(CMAKE_GENERATOR_PLATFORM x64)
find_package(Scanner REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  sensor_msgs
  roslib # necessary to getPath
  leica_scanstation_msgs
  leica_scanstation_utils
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES leica_scanstation_ros
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs sensor_msgs 
#  DEPENDS system_lib 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Scanner_INCLUDE_DIRS}
)

add_definitions(-D_AFXDLL)  # Set MFC aplication
ADD_DEFINITIONS(-D_UNICODE) # To avoid CString errors
# add_definitions(-DCMAKE_GENERATOR_PLATFORM=x64) # Build on x64 instead WIN32 (not working)
set(CMAKE_MFC_FLAG 2)


add_executable(${PROJECT_NAME}_node 
  src/main.cpp
  src/EventAnalyser.cpp
  src/LeicaNode.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Scanner_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node 
  leica_scanstation_msgs_generate_messages_cpp
)