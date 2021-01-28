project(leica_scanstation_ros)
cmake_minimum_required(VERSION 3.11.4)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Scanner REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  sensor_msgs
  diagnostic_msgs
  roslib # necessary to getPath
  leica_scanstation_msgs
  leica_scanstation_utils
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
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

add_library(${PROJECT_NAME} STATIC
  src/EventAnalyser.cpp
  src/LeicaNode.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Scanner_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node 
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

add_dependencies(${PROJECT_NAME}_node 
  leica_scanstation_msgs_generate_messages_cpp
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Place DLL files
file(GLOB DLL_FILES ${Scanner_LIBRARY_DIR}/*.dll)
file(INSTALL ${DLL_FILES}
DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION})

if($ENV{LEICA_EXPORT_PROGRAM})
  message(STATUS "LEICA_EXPORT_PROGRAM is on")
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/export)
  foreach(path ${CMAKE_PREFIX_PATH})
    file(GLOB ROS_DLL_FILES ${path}/bin/*.dll)
    file(COPY ${ROS_DLL_FILES}
    DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/export)
    unset(ROS_DLL_FILES)
  endforeach(path)
  file(GLOB SYS_FILES C:/Windows/System32/mfc140u.dll
                      C:/Windows/System32/vcruntime140_1.dll
                      C:/opt/python27amd64/python27.dll)
  file(COPY ${SYS_FILES}
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/export)                      
  file(INSTALL ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}/${PROJECT_NAME}_node.exe
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/export)     
endif() 
