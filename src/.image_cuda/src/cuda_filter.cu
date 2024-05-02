#include "../include/cuda_filter.h"

__global__ void depthMapToPointCloud(const unsigned char* inputImage1, const unsigned char* inputImage2,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, int width, int height,
                                     float focal_length, float baseline) {
    // int x = blockIdx.x * blockDim.x + threadIdx.x;
    // int y = blockIdx.y * blockDim.y + threadIdx.y;

    // if (x < width && y < height) {
        // int index = y * width + x;
        // int index3 = index * 3; // index for RGB channels

        // unsigned char blue1 = inputImage1[index3];
        // unsigned char green1 = inputImage1[index3 + 1];
        // unsigned char red1 = inputImage1[index3 + 2];

        // unsigned char blue2 = inputImage2[index3];
        // unsigned char green2 = inputImage2[index3 + 1];
        // unsigned char red2 = inputImage2[index3 + 2];

        // // Convert to grayscale using luminance formula (Y = 0.299*R + 0.587*G + 0.114*B)
        // float gray1 = 0.299f * red1 + 0.587f * green1 + 0.114f * blue1;
        // float gray2 = 0.299f * red2 + 0.587f * green2 + 0.114f * blue2;

        // // Average the grayscale pixel values
        // float disparity = fabs(gray1 - gray2);
        // float depth = (baseline * focal_length) / disparity;

        // // Calculate the 3D coordinates of the point
        // float X = ((float)x - (float)width / 2.0f) * depth / focal_length;
        // float Y = ((float)y - (float)height / 2.0f) * depth / focal_length;
        // float Z = depth;

        // // Write point to point cloud
        // pcl::PointXYZ point;
        // point.x = X;
        // point.y = Y;
        // point.z = Z;
        // point_cloud->push_back(point);
    // }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr depthMapToPointCloudMsg(const cv::Mat& inputImage1, const cv::Mat& inputImage2,
                                                             float focal_length, float baseline) {
    int width = inputImage1.cols;
    int height = inputImage1.rows;

    // // Allocate GPU memory for the input images
    // unsigned char* d_inputImage1 = nullptr;
    // unsigned char* d_inputImage2 = nullptr;
    // cudaMalloc((void**)&d_inputImage1, width * height * 3 * sizeof(unsigned char));
    // cudaMalloc((void**)&d_inputImage2, width * height * 3 * sizeof(unsigned char));
    // cudaMemcpy(d_inputImage1, inputImage1.data, width * height * 3 * sizeof(unsigned char), cudaMemcpyHostToDevice);
    // cudaMemcpy(d_inputImage2, inputImage2.data, width * height * 3 * sizeof(unsigned char), cudaMemcpyHostToDevice);

    // // Allocate GPU memory for the point cloud data
    // float* d_point_cloud_data = nullptr;
    // cudaMalloc((void**)&d_point_cloud_data, width * height * 3 * sizeof(float));

    // // Set up grid and block dimensions
    // dim3 blockDim(16, 16);
    // dim3 gridDim((width + blockDim.x - 1) / blockDim.x, (height + blockDim.y - 1) / blockDim.y);

    // // Launch the CUDA kernel
    // depthMapToPointCloud<<<gridDim, blockDim>>>(d_inputImage1, d_inputImage2, d_point_cloud_data, width, height,
    //                                             focal_length, baseline);

    // // Copy the result back from GPU to CPU
    // float* h_point_cloud_data = new float[width * height * 3];
    // cudaMemcpy(h_point_cloud_data, d_point_cloud_data, width * height * 3 * sizeof(float), cudaMemcpyDeviceToHost);

    // // Convert to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // for (int i = 0; i < width * height; ++i) {
    //     pcl::PointXYZ point;
    //     point.x = h_point_cloud_data[i * 3];
    //     point.y = h_point_cloud_data[i * 3 + 1];
    //     point.z = h_point_cloud_data[i * 3 + 2];
    //     point_cloud->push_back(point);
    // }

    // // Free GPU memory
    // cudaFree(d_inputImage1);
    // cudaFree(d_inputImage2);
    // cudaFree(d_point_cloud_data);
    // delete[] h_point_cloud_data;

    return point_cloud;
}

int main()
{
    // // Load an image using OpenCV
    // cv::Mat inputImage = cv::imread("input.jpg");

    // if (inputImage.empty())
    // {
    //     std::cerr << "Error: Could not load input image." << std::endl;
    //     return -1;
    // }

    // Call the image processing function
    // cv::Mat filteredImage = processImage(inputImage);

    // // Display or save the filtered image
    // cv::imwrite("output.jpg", filteredImage);
    // cv::imshow("Filtered Image", filteredImage);
    // cv::waitKey(0);

    return 0;
}