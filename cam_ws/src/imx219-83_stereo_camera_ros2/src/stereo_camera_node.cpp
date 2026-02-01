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
        cam0_ = std::make_unique<VideoCapture>(
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1 ! "
            "nvvidconv flip-method=2 ! "
            "video/x-raw, format=RGB ! "
            "videoconvert ! "
            "appsink drop=true sync=false",
            cv::CAP_GSTREAMER);

        cam1_ = std::make_unique<VideoCapture>(
            "nvarguscamerasrc sensor-id=1 ! "
            "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1 ! "
            "nvvidconv flip-method=2 ! "
            "video/x-raw, format=RGB ! "
            "videoconvert ! "
            "appsink drop=true sync=false",
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

        // read file paths from parameters (overrides the compile-time/static paths)
        std::string left_ini = this->get_parameter("left_ini").as_string();
        std::string right_ini = this->get_parameter("right_ini").as_string();

        if (left_ini.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Parameter left_ini is empty. Left camera_info will be empty.");
        }
        else
        {
            bool left_ok = parseIniToCameraInfo(left_ini, camera_info_left_, "imx_219_left_link");
            if (!left_ok)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to parse left ini '%s'", left_ini.c_str());
            }
        }

        if (right_ini.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Parameter right_ini is empty. Right camera_info will be empty.");
        }
        else
        {
            bool right_ok = parseIniToCameraInfo(right_ini, camera_info_right_, "imx_219_right_link");
            if (!right_ok)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to parse right ini '%s'", right_ini.c_str());
            }
        }

        // Create timer for publishing at ~50 Hz (20 ms)
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
    // Helper utilities for parsing
    static inline std::string trim(const std::string &s)
    {
        auto start = s.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) return "";
        auto end = s.find_last_not_of(" \t\r\n");
        return s.substr(start, end - start + 1);
    }

    static inline std::vector<std::string> split_tokens(const std::string &s)
    {
        std::istringstream iss(s);
        std::vector<std::string> out;
        std::string tok;
        while (iss >> tok) out.push_back(tok);
        return out;
    }

    bool parseIniToCameraInfo(const std::string &path, sensor_msgs::msg::CameraInfo &ci, const std::string &frame_id)
    {
        std::ifstream ifs(path);
        if (!ifs.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera info file: %s", path.c_str());
            return false;
        }

        ci = sensor_msgs::msg::CameraInfo();
        ci.header.frame_id = frame_id;
        ci.distortion_model = "plumb_bob";

        std::string line;
        while (std::getline(ifs, line))
        {
            line = trim(line);
            if (line.empty()) continue;
            // keys are single words like "width", "height", "camera matrix", "distortion", "rectification", "projection"
            if (line == "width")
            {
                // next non-empty line is width value
                while (std::getline(ifs, line) && trim(line).empty()) {}
                if (!ifs) break;
                ci.width = std::stoi(trim(line));
            }
            else if (line == "height")
            {
                while (std::getline(ifs, line) && trim(line).empty()) {}
                if (!ifs) break;
                ci.height = std::stoi(trim(line));
            }
            else if (line == "camera matrix")
            {
                // read 3 rows
                std::vector<double> K;
                for (int r = 0; r < 3; ++r)
                {
                    if (!std::getline(ifs, line)) break;
                    line = trim(line);
                    if (line.empty()) { --r; continue; }
                    auto toks = split_tokens(line);
                    for (auto &t : toks) K.push_back(std::stod(t));
                }
                if (K.size() == 9)
                {
                    for (int i = 0; i < 9; ++i) ci.k[i] = K[i];
                }
            }
            else if (line == "distortion")
            {
                // single line with coefficients
                while (std::getline(ifs, line) && trim(line).empty()) {}
                if (!ifs) break;
                auto toks = split_tokens(trim(line));
                ci.d.clear();
                for (auto &t : toks) ci.d.push_back(std::stod(t));
            }
            else if (line == "rectification")
            {
                std::vector<double> R;
                for (int r = 0; r < 3; ++r)
                {
                    if (!std::getline(ifs, line)) break;
                    line = trim(line);
                    if (line.empty()) { --r; continue; }
                    auto toks = split_tokens(line);
                    for (auto &t : toks) R.push_back(std::stod(t));
                }
                if (R.size() == 9)
                {
                    for (int i = 0; i < 9; ++i) ci.r[i] = R[i];
                }
            }
            else if (line == "projection")
            {
                std::vector<double> P;
                for (int r = 0; r < 3; ++r)
                {
                    if (!std::getline(ifs, line)) break;
                    line = trim(line);
                    if (line.empty()) { --r; continue; }
                    auto toks = split_tokens(line);
                    for (auto &t : toks) P.push_back(std::stod(t));
                }
                if (P.size() == 12)
                {
                    for (int i = 0; i < 12; ++i) ci.p[i] = P[i];
                }
            }
        }

        // if K not set from file, try to fill from projection P (fx = P[0], fy = P[5], cx = P[2], cy = P[6])
        bool K_valid = true;
        for (int i = 0; i < 9; ++i) if (ci.k[i] == 0.0) { K_valid = false; break; }
        if (!K_valid)
        {
            if (ci.p[0] != 0.0 || ci.p[5] != 0.0)
            {
                ci.k[0] = ci.p[0];
                ci.k[1] = 0.0;
                ci.k[2] = ci.p[2];
                ci.k[3] = 0.0;
                ci.k[4] = ci.p[5];
                ci.k[5] = ci.p[6];
                ci.k[6] = 0.0;
                ci.k[7] = 0.0;
                ci.k[8] = 1.0;
            }
        }

        return true;
    }

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
            RCLCPP_INFO(this->get_logger(), "Capture times (ns) left: %lld, right: %lld, diff (left - right): %lld",
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

        // attach stamps to camera_info and publish them
        camera_info_left_.header.stamp = left_stamp;
        camera_info_left_.header.frame_id = left_header.frame_id;
        camera_info_right_.header.stamp = right_stamp;
        camera_info_right_.header.frame_id = right_header.frame_id;

        // compute publish time and latencies from capture to publish (in milliseconds)
        {
            rclcpp::Time publish_time = this->now();
            long long left_latency_ms = (publish_time - left_stamp).nanoseconds() / 1000000LL;
            long long right_latency_ms = (publish_time - right_stamp).nanoseconds() / 1000000LL;
            RCLCPP_INFO(this->get_logger(), "Publish time now (ns): %lld, latency left: %lld ms, latency right: %lld ms",
                        static_cast<long long>(publish_time.nanoseconds()),
                        left_latency_ms, right_latency_ms);
        }

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

