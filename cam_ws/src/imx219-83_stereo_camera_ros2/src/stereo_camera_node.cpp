#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace cv;
using namespace std;


class StereoCameraNode : public rclcpp::Node
{
public:
    StereoCameraNode()
        : Node("stereo_camera_node"), cameras_initialized_(false)
    {
        // Declare params for camera info file paths (can be overridden via launch/CLI)
        // Defaults point to package config directory used previously
        this->declare_parameter<std::string>(
            "left_ini",
            "");
        this->declare_parameter<std::string>(
            "right_ini",
            "");

        // Initialize cameras
        cam0_ = std::make_unique<cv::VideoCapture>(
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1 ! "
            "nvvidconv flip-method=2 ! "
            "video/x-raw, format=GRAY8 ! "
            "appsink drop=true sync=true",
            cv::CAP_GSTREAMER);

        cam1_ = std::make_unique<cv::VideoCapture>(
            "nvarguscamerasrc sensor-id=1 ! "
            "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1 ! "
            "nvvidconv flip-method=2 ! "
            "video/x-raw, format=GRAY8 ! "
            "appsink drop=true sync=true",
            cv::CAP_GSTREAMER);



        if (!cam0_->isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "cam0 is not opened.");
            return;
        }
        if (!cam1_->isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "cam1 is not opened.");
            return;
        }

        cameras_initialized_ = true;

        RCLCPP_INFO(this->get_logger(), "Resolution image cam0 : %dx%d",
                    static_cast<int>(cam0_->get(cv::CAP_PROP_FRAME_WIDTH)),
                    static_cast<int>(cam0_->get(cv::CAP_PROP_FRAME_HEIGHT)));
        RCLCPP_INFO(this->get_logger(), "Frames per second using cam0 : %.2f",
                    cam0_->get(cv::CAP_PROP_FPS));
        RCLCPP_INFO(this->get_logger(), "Resolution image cam1 : %dx%d",
                    static_cast<int>(cam1_->get(cv::CAP_PROP_FRAME_WIDTH)),
                    static_cast<int>(cam1_->get(cv::CAP_PROP_FRAME_HEIGHT)));
        RCLCPP_INFO(this->get_logger(), "Frames per second using cam1 : %.2f",
                    cam1_->get(cv::CAP_PROP_FPS));
    }

    void initialize(rclcpp::Node::SharedPtr node_ptr)
    {
        if (!cameras_initialized_)
        {
            RCLCPP_ERROR(this->get_logger(), "Cameras not initialized. Cannot initialize publishers.");
            return;
        }
        
        // Initialize image transport and publishers
        // This must be called after the node is created as a shared_ptr
        it_ = std::make_shared<image_transport::ImageTransport>(node_ptr);
        pub_left_camera_ = it_->advertise("/stereo/left/image_raw", 1);
        pub_right_camera_ = it_->advertise("/stereo/right/image_raw", 1);

        // camera_info publishers
        pub_left_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/stereo/left/camera_info", 1);
        pub_right_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/stereo/right/camera_info", 1);

        // Hard-coded CameraInfo values (from left.ini / right.ini)
        // LEFT
        camera_info_left_ = sensor_msgs::msg::CameraInfo();
        camera_info_left_.header.frame_id = "imx_219_left_link";
        camera_info_left_.width = 640;
        camera_info_left_.height = 480;
        camera_info_left_.distortion_model = "plumb_bob";
        camera_info_left_.d = {-0.042341, 0.188809, 0.002007, -0.001397, 0.0};
        // K (row-major)
        camera_info_left_.k[0] = 578.150101; camera_info_left_.k[1] = 0.0;        camera_info_left_.k[2] = 317.685694;
        camera_info_left_.k[3] = 0.0;        camera_info_left_.k[4] = 771.864750; camera_info_left_.k[5] = 240.599903;
        camera_info_left_.k[6] = 0.0;        camera_info_left_.k[7] = 0.0;        camera_info_left_.k[8] = 1.0;
        // R
        camera_info_left_.r[0]=0.997598; camera_info_left_.r[1]=-0.040442; camera_info_left_.r[2]=-0.056232;
        camera_info_left_.r[3]=0.040156; camera_info_left_.r[4]=0.999174;  camera_info_left_.r[5]=-0.006198;
        camera_info_left_.r[6]=0.056437; camera_info_left_.r[7]=0.003925;  camera_info_left_.r[8]=0.998398;
        // P (3x4)
        camera_info_left_.p[0]=888.815717; camera_info_left_.p[1]=0.0;        camera_info_left_.p[2]=362.861237; camera_info_left_.p[3]=0.0;
        camera_info_left_.p[4]=0.0;        camera_info_left_.p[5]=888.815717; camera_info_left_.p[6]=253.260841; camera_info_left_.p[7]=0.0;
        camera_info_left_.p[8]=0.0;        camera_info_left_.p[9]=0.0;        camera_info_left_.p[10]=1.0;       camera_info_left_.p[11]=0.0;

        // RIGHT
        camera_info_right_ = sensor_msgs::msg::CameraInfo();
        camera_info_right_.header.frame_id = "imx_219_right_link";
        camera_info_right_.width = 640;
        camera_info_right_.height = 480;
        camera_info_right_.distortion_model = "plumb_bob";
        camera_info_right_.d = {-0.070246, 0.331961, 0.002207, -0.001475, 0.0};
        // K
        camera_info_right_.k[0] = 574.116424; camera_info_right_.k[1] = 0.0;        camera_info_right_.k[2] = 307.087843;
        camera_info_right_.k[3] = 0.0;        camera_info_right_.k[4] = 766.076052; camera_info_right_.k[5] = 263.691838;
        camera_info_right_.k[6] = 0.0;        camera_info_right_.k[7] = 0.0;        camera_info_right_.k[8] = 1.0;
        // R
        camera_info_right_.r[0]=0.998057; camera_info_right_.r[1]=-0.036558; camera_info_right_.r[2]=-0.050451;
        camera_info_right_.r[3]=0.036814; camera_info_right_.r[4]=0.999314;  camera_info_right_.r[5]=0.004137;
        camera_info_right_.r[6]=0.050265; camera_info_right_.r[7]=-0.005987; camera_info_right_.r[8]=0.998718;
        // P
        camera_info_right_.p[0]=888.815717; camera_info_right_.p[1]=0.0;        camera_info_right_.p[2]=362.861237; camera_info_right_.p[3]=-51.697515;
        camera_info_right_.p[4]=0.0;        camera_info_right_.p[5]=888.815717; camera_info_right_.p[6]=253.260841; camera_info_right_.p[7]=0.0;
        camera_info_right_.p[8]=0.0;        camera_info_right_.p[9]=0.0;        camera_info_right_.p[10]=1.0;      camera_info_right_.p[11]=0.0;
 
        // Create timer for publishing at ~50 Hz (20 ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&StereoCameraNode::timer_callback, this));
    }
    
    bool is_initialized() const
    {
        return cameras_initialized_;
    }

    ~StereoCameraNode()
    {
        if (cam0_ && cam0_->isOpened())
        {
            cam0_->release();
        }
        if (cam1_ && cam1_->isOpened())
        {
            cam1_->release();
        }
    }

private:
    void timer_callback()
    {
        Mat cam0Frame;
        Mat cam1Frame;

        // Read left camera and record its capture time immediately after read
        if (!cam0_->read(cam0Frame))
        {
            RCLCPP_WARN(this->get_logger(), "No frame from cam0");
            return;
        }
        rclcpp::Time left_stamp = this->now();

        // Read right camera and record its capture time immediately after read
        if (!cam1_->read(cam1Frame))
        {
            RCLCPP_WARN(this->get_logger(), "No frame from cam1");
            return;
        }
        rclcpp::Time right_stamp = this->now();

        // Convert to ROS messages with per-image timestamps and distinct frame_ids
        std_msgs::msg::Header left_header;
        left_header.stamp = left_stamp;
        left_header.frame_id = "imx_219_left_link";

        std_msgs::msg::Header right_header;
        right_header.stamp = right_stamp;
        right_header.frame_id = "imx_219_right_link";

        cv::Mat cam0_rgb, cam1_rgb;
        //cv::cvtColor(cam0Frame, cam0_rgb, cv::COLOR_BGR2RGB);
        //cv::cvtColor(cam1Frame, cam1_rgb, cv::COLOR_BGR2RGB);

        auto imageLeftMsg =
            cv_bridge::CvImage(
                left_header,
                sensor_msgs::image_encodings::MONO8,
                cam0Frame
            ).toImageMsg();

        auto imageRightMsg =
            cv_bridge::CvImage(
                right_header,
                sensor_msgs::image_encodings::MONO8,
                cam1Frame
            ).toImageMsg();



        // attach stamps to camera_info and publish them
        camera_info_left_.header.stamp = left_stamp;
        camera_info_left_.header.frame_id = left_header.frame_id;
        camera_info_right_.header.stamp = right_stamp;
        camera_info_right_.header.frame_id = right_header.frame_id;

        // publish images and camera_info
        pub_left_camera_.publish(imageLeftMsg);
        pub_right_camera_.publish(imageRightMsg);

        pub_left_info_->publish(camera_info_left_);
        pub_right_info_->publish(camera_info_right_);
    }

    std::unique_ptr<VideoCapture> cam0_;
    std::unique_ptr<VideoCapture> cam1_;
    cv::Matx33d t1_;
    cv::Matx33d t2_;
    bool cameras_initialized_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher pub_left_camera_;
    image_transport::Publisher pub_right_camera_;
    rclcpp::TimerBase::SharedPtr timer_;

    // new members for camera info
    sensor_msgs::msg::CameraInfo camera_info_left_;
    sensor_msgs::msg::CameraInfo camera_info_right_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_left_info_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_right_info_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoCameraNode>();
    
    if (!node->is_initialized())
    {
        RCLCPP_ERROR(rclcpp::get_logger("stereo_camera_node"), "Failed to initialize cameras. Exiting.");
        rclcpp::shutdown();
        return 1;
    }
    
    node->initialize(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

