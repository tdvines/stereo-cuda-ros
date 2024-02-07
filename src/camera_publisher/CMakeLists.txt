cmake_minimum_required(VERSION 3.0.2)
project(dual_camera_publisher)
set(CMAKE_CXX_STANDARD 11)  # Set C++ standard to C++11
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # Require C++11, if available

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs image_transport cv_bridge sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(dual_camera_publisher_node src/camera_publisher_node.cpp)
target_link_libraries(dual_camera_publisher_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
