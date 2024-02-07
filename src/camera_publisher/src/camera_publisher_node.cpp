#include "camera_publisher_node.h"

namespace CSICameraROS {

    CSICameraROS::CSICameraROS(int camera_id, std::string camera_topic, int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
        : camera_id(camera_id),
          camera_topic_(camera_topic),
          capture_width_(capture_width),
          capture_height_(capture_height),
          display_width_(display_width),
          display_height_(display_height),
          framerate_(framerate),
          flip_method_(flip_method),
          running_(false),
          nh_(),
          it_(nh_) {
        image_pub_0 = it_.advertise(camera_topic_, 1);

    }

    void CSICameraROS::start(const std::string& gstreamer_pipeline) {
        if (running_) {
            ROS_INFO("Camera already running");
            return;
        }
        video_capture_.open(gstreamer_pipeline, cv::CAP_GSTREAMER);
        if (!video_capture_.isOpened()) {
            ROS_ERROR("Failed to open camera");
            return;
        }
        running_ = true;
        read_thread_ = std::thread(&CSICameraROS::updateCamera, this);
    }

    bool CSICameraROS::read(cv::Mat& frame) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (!frame_.empty()) {
            frame_.copyTo(frame);
            return true;
        }
        return false;
    }

    void CSICameraROS::stop() {
        if (!running_) {
            return;
        }

        running_ = false;

        // Wait for read thread to finish
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
    }

    void CSICameraROS::release() {
        if (video_capture_.isOpened()) {
            video_capture_.release();
        }
    }

    void CSICameraROS::updateCamera() {
        while (running_) {
            cv::Mat frame;
            bool grabbed = video_capture_.read(frame);
            if (!grabbed || frame.empty()) {
                ROS_ERROR("Failed to capture frame");
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                frame_ = frame.clone();
            }
        }
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "raspberry_pi_dual_camera_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // std::string camera_topic_0, camera_topic_1;
    private_nh.param<std::string>("camera_topic_0_", camera_topic_0, "/camera0/image_raw");
    private_nh.param<std::string>("camera_topic_1_", camera_topic_1, "/camera1/image_raw");

    int camera_id_0 = 0;
    int camera_id_1 = 1; // Assuming second camera
    int capture_width = 1920;
    int capture_height = 1080;
    int display_width = 960;
    int display_height = 540;
    int framerate = 20;
    int flip_method = 0;

    // GStreamer pipeline for Camera 0
    std::string gstreamer_pipeline_0 = "nvarguscamerasrc sensor-id=" + std::to_string(camera_id_0)
        + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" + std::to_string(capture_height)
        + ", framerate=(fraction)" + std::to_string(framerate) + "/1 ! nvvidconv flip-method=" + std::to_string(flip_method)
        + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" + std::to_string(display_height)
        + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

    // GStreamer pipeline for Camera 1
    std::string gstreamer_pipeline_1 = "nvarguscamerasrc sensor-id=" + std::to_string(camera_id_1)
        + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" + std::to_string(capture_height)
        + ", framerate=(fraction)" + std::to_string(framerate) + "/1 ! nvvidconv flip-method=" + std::to_string(flip_method)
        + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" + std::to_string(display_height)
        + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

    CSICameraROS::CSICameraROS camera_pub_0(camera_id_0, camera_topic_0, capture_width, capture_height, display_width, display_height, framerate, flip_method);
    CSICameraROS::CSICameraROS camera_pub_1(camera_id_1, camera_topic_1, capture_width, capture_height, display_width, display_height, framerate, flip_method);

    camera_pub_0.start(gstreamer_pipeline_0);
    camera_pub_1.start(gstreamer_pipeline_1);

    // cv::namedWindow("CSI Camera 0", cv::WINDOW_AUTOSIZE);
    // cv::namedWindow("CSI Camera 1", cv::WINDOW_AUTOSIZE);

    while (ros::ok()) {
        cv::Mat frame_0, frame_1;
        bool success_0 = camera_pub_0.read(frame_0);
        bool success_1 = camera_pub_1.read(frame_1);

        if (success_0) {
            cv::imshow("CSI Camera 0", frame_0);
            sensor_msgs::ImagePtr msg_0 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_0).toImageMsg();
            camera_pub_0.image_pub_0.publish(msg_0);
        } else {
            ROS_ERROR("Failed to read frame from Camera 0");
        }

        if (success_1) {
            cv::imshow("CSI Camera 1", frame_1);
            sensor_msgs::ImagePtr msg_1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_1).toImageMsg();
            camera_pub_1.image_pub_0.publish(msg_1);
        } else {
            ROS_ERROR("Failed to read frame from Camera 1");
        }

        char key = cv::waitKey(10);
        if (key == 27) {
            ROS_ERROR("Break");
            break;
        }

        ros::spinOnce();
    }

    camera_pub_0.stop();
    camera_pub_0.release();

    camera_pub_1.stop();
    camera_pub_1.release();

    cv::destroyAllWindows();

    return 0;
}
