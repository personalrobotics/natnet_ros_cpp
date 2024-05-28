cmake_minimum_required(VERSION 3.0.2)
project(natnet_ros_cpp)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
  sensor_msgs
  tf2 
  tf2_ros
  message_generation
)
find_package(Boost 1.65 REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)

#download Natnet SDK
execute_process(
	COMMAND chmod "+x" "${CMAKE_CURRENT_SOURCE_DIR}/install_sdk.sh"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
execute_process(
	COMMAND bash "-c" "${CMAKE_CURRENT_SOURCE_DIR}/install_sdk.sh"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Enable C++14 and warnings
#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)

set(NATNET_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/NatNetSDK/) 
set(NATNET_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/NatNetSDK/lib/libNatNet.so)
set(my_include_directories
  include/natnet_ros_cpp
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${NATNET_SDK_DIR}/include
)
set(QT_DEPEND
    Qt5Widgets
    Qt5PrintSupport
    )
set (QT_DEPEND_LIB
    Qt5::Widgets
    Qt5::Core
    Qt5::PrintSupport
    )
add_message_files(
  FILES
  PointArray.msg
  )

add_service_files(
  FILES
  MarkerPoses.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)
## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES natnet_ros_cpp
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime rospy std_msgs sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${my_include_directories}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(${PROJECT_NAME} src/natnet_ros.cpp src/internal.cpp src/nn_filter.cpp src/transforms.cpp)

add_executable(marker_poses_server src/marker_poses_server.cpp)
target_link_libraries(marker_poses_server ${catkin_LIBRARIES} ${NATNET_LIBRARY_DIR})
add_dependencies(marker_poses_server ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${NATNET_LIBRARY_DIR}
)

catkin_install_python(PROGRAMS scripts/helper_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)