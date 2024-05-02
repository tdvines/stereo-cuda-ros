cmake_minimum_required(VERSION 3.0.2)
project(disparity_calculator)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    image_transport
    sensor_msgs
    std_msgs
)

find_package(OpenCV 3.4 REQUIRED)
find_package(CUDA REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs image_transport cv_bridge sensor_msgs
)

# Add library target
add_library(disparity_calculator_lib src/disparity_calculator.cpp)

# Include directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
)

# Add executable target
add_executable(disparity_calculator_node src/main.cpp)

# Link the library and other dependencies to the executable
target_link_libraries(disparity_calculator_node 
    disparity_calculator_lib
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CUDA_LIBRARIES}
)

