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
find_package(OpenCV 3.4 REQUIRED)
find_package(CUDA REQUIRED)
find_package(PCL REQUIRED)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

# Set the path to the CUDA filter source file
set(CUDA_FILTER_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/src/cuda_filter.cu)

# Add library target for CUDA filter
cuda_add_library(CUDALib ${CUDA_FILTER_SOURCE})

# Link CUDA libraries
target_link_libraries(CUDALib ${CUDA_LIBRARIES})

# Specify the directory containing disparity_calculator headers
set(DISPARITY_CALCULATOR_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../disparity_calculator/include)

# Include disparity_calculator headers
include_directories(${DISPARITY_CALCULATOR_INCLUDE_DIRS})

# Specify the directory containing disparity_calculator library
set(DISPARITY_CALCULATOR_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../disparity_calculator/lib)

# Link disparity_calculator library
add_library(disparity_calculator SHARED IMPORTED)
set_target_properties(disparity_calculator PROPERTIES IMPORTED_LOCATION ${DISPARITY_CALCULATOR_LIB_DIR}/disparity_calculator_library.so)

# Add executable target for image_converter
add_executable(${PROJECT_NAME}_node src/image_converter.cpp)

# Link libraries with image_converter executable
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  CUDALib
  disparity_calculator
)
