#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <chrono>

using namespace cv;
using namespace std;

constexpr char STEREO_PARAMS_PATH[] = "./Stereo Calibration/data/stereocalib.yml";

class StereoCameraNode : public rclcpp::Node
{
public:
    StereoCameraNode()
        : Node("stereo_camera_node"), cameras_initialized_(false)
    {
        // Initialize cameras
        cam0_ = std::make_unique<VideoCapture>(
            "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=640, height=480, framerate=(fraction)30/1 ! nvvidconv flip-method=2 ! videoconvert ! appsink",
            cv::CAP_GSTREAMER);
        cam1_ = std::make_unique<VideoCapture>(
            "nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=640, height=480, framerate=(fraction)30/1 ! nvvidconv flip-method=2 ! videoconvert ! appsink",
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

        // Create timer for publishing at 10 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
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

        // print capture times and difference between them (in nanoseconds)
        {
            long long left_ns = static_cast<long long>(left_stamp.nanoseconds());
            long long right_ns = static_cast<long long>(right_stamp.nanoseconds());
            long long diff_ns = left_ns - right_ns;
            if (diff_ns < 0) diff_ns = -diff_ns;
            RCLCPP_INFO(this->get_logger(), "Capture times (ns) left: %lld, right: %lld, abs_diff: %lld",
                        left_ns, right_ns, diff_ns);
        }


        // Convert to ROS messages with per-image timestamps and distinct frame_ids
        std_msgs::msg::Header left_header;
        left_header.stamp = left_stamp;
        left_header.frame_id = "imx_219_left_link";

        std_msgs::msg::Header right_header;
        right_header.stamp = right_stamp;
        right_header.frame_id = "imx_219_right_link";

        sensor_msgs::msg::Image::SharedPtr imageLeftMsg =
            cv_bridge::CvImage(left_header, "rgb8", cam0Frame).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr imageRightMsg =
            cv_bridge::CvImage(right_header, "rgb8", cam1Frame).toImageMsg();

        // compute publish time and latencies from capture to publish (in milliseconds)
        {
            rclcpp::Time publish_time = this->now();
            long long left_latency_ms = (publish_time - left_stamp).nanoseconds() / 1000000LL;
            long long right_latency_ms = (publish_time - right_stamp).nanoseconds() / 1000000LL;
            RCLCPP_INFO(this->get_logger(), "Publish time now (ns): %lld, latency left: %lld ms, latency right: %lld ms",
                        static_cast<long long>(publish_time.nanoseconds()),
                        left_latency_ms, right_latency_ms);
        }

        pub_left_camera_.publish(imageLeftMsg);
        pub_right_camera_.publish(imageRightMsg);
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

