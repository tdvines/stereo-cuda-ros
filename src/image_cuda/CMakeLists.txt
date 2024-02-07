cmake_minimum_required(VERSION 3.0.2)
project(image_cuda)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

# Find OpenCV package
find_package(OpenCV 3 REQUIRED)
find_package(CUDA REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs image_transport cv_bridge sensor_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES image_cuda
 CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
 DEPENDS 
)

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -arch=sm_53)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Add the library target
add_library(disparity_calculator SHARED src/disparity_calculator.cpp)
cuda_add_library(CUDALib ${CMAKE_CURRENT_SOURCE_DIR}/src/cuda_filter.cu)

# Link the OpenCV libraries
target_link_libraries(disparity_calculator ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}_node src/image_converter.cpp)

# Link the OpenCV libraries
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} disparity_calculator ${OpenCV_LIBS} CUDALib)

