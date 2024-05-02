# stereo-cuda-ros

ROS depth perception using stereo cameras. 
Built and tested on Jetson Nano utilizing CUDA in ROS. 


Requirements
============

## Jetpack 4.5.1
    https://developer.nvidia.com/embedded/downloads/archive

## ROS Melodic
Install desktop full version

    http://wiki.ros.org/melodic/Installation/Ubuntu




## VTK v9.0.3
    https://github.com/Kitware/VTK/tree/v9.0.3

Terminal command:

    git clone -b v9.0.3 https://github.com/Kitware/VTK.git

    cd VTK

    mkdir build && cd build

    ccmake ..

Install with ccmake .. and build with CUDA ON. 

- Turn ON the following:
    - VTK_USE_CUDA=ON

c configure x2, g generate. 

    make -j$(nproc)

    sudo make install

    sudo ldconfig


## Point Cloud Library
    https://github.com/PointCloudLibrary/pcl

Terminal command:

    git clone -b master https://github.com/PointCloudLibrary/pcl.git

    cd pcl

    mkdir build && cd build

    ccmake ..

Install with ccmake .. and build with CUDA and VTK ON. 

- Turn ON the following:
    - WITH_VTK=ON 
    - WITH_CUDA=ON 
    
c configure x2, g generate. 

    make 

Might take some time on Jetson Nano. 

    sudo make install

    sudo ldconfig


## Opencv 3.4 with contrib 
    https://github.com/opencv/opencv/tree/3.4

Terminal command:

    git clone -b 3.4 https://github.com/opencv/opencv.git

    cd opencv

    mkdir build && cd build

    ccmake ..

Install with ccmake .. and build with CUDA ON. 

- Turn ON the following:
    - WITH_CUDA=ON 
    - CUDA_ARCH_BIN=5.3 // for Jetson Nano
    - CUDA_ARCH_PTX="" 
    - WITH_CUBLAS=ON 
    - ENABLE_FAST_MATH=ON 
    - CUDA_FAST_MATH=ON
    - ENABLE_NEON=ON 
    - WITH_GSTREAMER=ON 
    - WITH_LIBV4L=ON
    - EIGEN_INCLUDE_PATH=/usr/include/eigen3
    - BUILD_opencv_python2=OFF 
    - BUILD_opencv_python3=ON
    - BUILD_TESTS=OFF 
    - BUILD_PERF_TESTS=OFF 
    - BUILD_EXAMPLES=OFF  
    - WITH_QT=ON 
    - WITH_OPENGL=ON 

c configure x2, g generate. 

    make -j$(nproc)

    sudo make install

    sudo ldconfig


