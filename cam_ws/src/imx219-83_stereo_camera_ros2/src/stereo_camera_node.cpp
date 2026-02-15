#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

using namespace cv;
using namespace std;

class StereoCameraNode : public rclcpp::Node {
public:
  StereoCameraNode() : Node("stereo_camera_node"), cameras_initialized_(false) {
    // Declare params for camera info (can be overridden via launch/CLI or
    // loaded from YAML param files via --params-file cameras.yaml).

    // LEFT defaults
    this->declare_parameter<std::string>("camera_info_left.frame_id",
                                         "imx_219_left_link");
    this->declare_parameter<int>("camera_info_left.width", 640);
    this->declare_parameter<int>("camera_info_left.height", 480);
    this->declare_parameter<std::string>("camera_info_left.distortion_model",
                                         "plumb_bob");
    this->declare_parameter<std::vector<double>>(
        "camera_info_left.d", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});

    this->declare_parameter<std::vector<double>>(
        "camera_info_left.k",
        std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    this->declare_parameter<std::vector<double>>(
        "camera_info_left.r",
        std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    this->declare_parameter<std::vector<double>>(
        "camera_info_left.p",
        std::vector<double>{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                            1.0, 0.0});

    // RIGHT defaults
    this->declare_parameter<std::string>("camera_info_right.frame_id",
                                         "imx_219_right_link");
    this->declare_parameter<int>("camera_info_right.width", 640);
    this->declare_parameter<int>("camera_info_right.height", 480);
    this->declare_parameter<std::string>("camera_info_right.distortion_model",
                                         "plumb_bob");
    this->declare_parameter<std::vector<double>>(
        "camera_info_right.d", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>(
        "camera_info_right.k",
        std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    this->declare_parameter<std::vector<double>>(
        "camera_info_right.r",
        std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    this->declare_parameter<std::vector<double>>(
        "camera_info_right.p",
        std::vector<double>{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                            1.0, 0.0});

    // GStreamer pipeline tunables
    this->declare_parameter<int>("framerate", 30);
    this->declare_parameter<bool>("drop", true);
    this->declare_parameter<bool>("sync", true);

    // Option to publish RGB instead of MONO8. Set via parameter or YAML file.
    this->declare_parameter<bool>("publish_rgb", false);

    // Cameras are opened in `initialize()` after params are loaded.
  }

  void initialize(rclcpp::Node::SharedPtr node_ptr) {
    // Read publish_rgb param first so we can open cameras with the
    // appropriate pixel format.
    this->get_parameter("publish_rgb", publish_rgb_);

    // Read width/height parameters for both cameras so we can build
    // pipelines that match the requested image size.
    int width_left = 640, height_left = 480;
    int width_right = 640, height_right = 480;
    this->get_parameter("camera_info_left.width", width_left);
    this->get_parameter("camera_info_left.height", height_left);
    this->get_parameter("camera_info_right.width", width_right);
    this->get_parameter("camera_info_right.height", height_right);

    // Build gstreamer pipelines depending on desired output format and size
    std::string pixel_format = publish_rgb_ ? "BGR" : "GRAY8";

    int framerate = 30;
    bool drop = true;
    bool sync = true;
    this->get_parameter("framerate", framerate);
    this->get_parameter("drop", drop);
    this->get_parameter("sync", sync);

    auto make_pipeline = [&](int sensor_id, int width, int height,
                             const std::string &format) {
      std::ostringstream ss;
      ss << "nvarguscamerasrc sensor-id=" << sensor_id
         << " ! video/x-raw(memory:NVMM), width=" << width
         << ", height=" << height
         << ", framerate=" << framerate << "/1"
         << " ! nvvidconv flip-method=2"
         << " ! video/x-raw, format=" << format
         << " ! appsink"
         << " drop=" << (drop ? "true" : "false")
         << " sync=" << (sync ? "true" : "false");
      return ss.str();
    };

    std::string pipeline0 = make_pipeline(0, width_left, height_left, pixel_format);
    std::string pipeline1 = make_pipeline(1, width_right, height_right, pixel_format);

    RCLCPP_INFO(this->get_logger(),
                "Initializing GStreamer cam0 with pipeline: %s",
                pipeline0.c_str());
    RCLCPP_INFO(this->get_logger(),
                "Initializing GStreamer cam1 with pipeline: %s",
                pipeline1.c_str());

    cam0_ = std::make_unique<cv::VideoCapture>(pipeline0, cv::CAP_GSTREAMER);
    cam1_ = std::make_unique<cv::VideoCapture>(pipeline1, cv::CAP_GSTREAMER);

    if (!cam0_->isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "cam0 is not opened.");
    }
    if (!cam1_->isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "cam1 is not opened.");
    }

    cameras_initialized_ =
        cam0_ && cam0_->isOpened() && cam1_ && cam1_->isOpened();

    if (!cameras_initialized_) {
      RCLCPP_ERROR(this->get_logger(),
                   "Cameras not initialized. Cannot initialize publishers.");
      return;
    }

    // Initialize image transport and publishers
    // This must be called after the node is created as a shared_ptr
    it_ = std::make_shared<image_transport::ImageTransport>(node_ptr);
    pub_left_camera_ = it_->advertise("/stereo/left/image_raw", 1);
    pub_right_camera_ = it_->advertise("/stereo/right/image_raw", 1);

    // camera_info publishers
    pub_left_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/stereo/left/camera_info", 1);
    pub_right_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/stereo/right/camera_info", 1);

    // Populate CameraInfo messages from parameters (allows loading from
    // YAML param files). We keep the message objects default-constructed and
    // fill fields from parameters declared in the constructor.
    camera_info_left_ = sensor_msgs::msg::CameraInfo();
    camera_info_right_ = sensor_msgs::msg::CameraInfo();

    // Helper to copy vectors into fixed-size arrays safely
    auto copy_into = [](const std::vector<double> &src, auto &dst) {
      size_t n = std::min(src.size(), dst.size());
      for (size_t i = 0; i < n; ++i) {
        dst[i] = src[i];
      }
    };

    std::vector<double> vec;

    // LEFT
    std::string frame_id_left;
    std::string dist_model_left = "plumb_bob";
    this->get_parameter("camera_info_left.frame_id", frame_id_left);
    this->get_parameter("camera_info_left.distortion_model", dist_model_left);
    camera_info_left_.header.frame_id = frame_id_left;
    camera_info_left_.width = width_left;
    camera_info_left_.height = height_left;
    camera_info_left_.distortion_model = dist_model_left;

    if (this->get_parameter("camera_info_left.d", vec)) {
      camera_info_left_.d = vec;
    }
    if (this->get_parameter("camera_info_left.k", vec)) {
      copy_into(vec, camera_info_left_.k);
    }
    if (this->get_parameter("camera_info_left.r", vec)) {
      copy_into(vec, camera_info_left_.r);
    }
    if (this->get_parameter("camera_info_left.p", vec)) {
      copy_into(vec, camera_info_left_.p);
    }

    // RIGHT
    std::string frame_id_right;
    std::string dist_model_right = "plumb_bob";
    this->get_parameter("camera_info_right.frame_id", frame_id_right);
    this->get_parameter("camera_info_right.distortion_model", dist_model_right);
    camera_info_right_.header.frame_id = frame_id_right;
    camera_info_right_.width = width_right;
    camera_info_right_.height = height_right;
    camera_info_right_.distortion_model = dist_model_right;

    if (this->get_parameter("camera_info_right.d", vec)) {
      camera_info_right_.d = vec;
    }
    if (this->get_parameter("camera_info_right.k", vec)) {
      copy_into(vec, camera_info_right_.k);
    }
    if (this->get_parameter("camera_info_right.r", vec)) {
      copy_into(vec, camera_info_right_.r);
    }
    if (this->get_parameter("camera_info_right.p", vec)) {
      copy_into(vec, camera_info_right_.p);
    }

    // read publish_rgb option
    this->get_parameter("publish_rgb", publish_rgb_);

    // Create timer for publishing at ~50 Hz (20 ms)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&StereoCameraNode::timer_callback, this));
  }

  bool is_initialized() const { return cameras_initialized_; }

  ~StereoCameraNode() {
    if (cam0_ && cam0_->isOpened()) {
      cam0_->release();
    }
    if (cam1_ && cam1_->isOpened()) {
      cam1_->release();
    }
  }

  // publish_rgb flag set from params
  bool publish_rgb_ = false;

private:
  void timer_callback() {
    Mat cam0Frame;
    Mat cam1Frame;

    // Read left camera and record its capture time immediately after read
    if (!cam0_->read(cam0Frame)) {
      RCLCPP_WARN(this->get_logger(), "No frame from cam0");
      return;
    }
    rclcpp::Time left_stamp = this->now();

    // Read right camera and record its capture time immediately after read
    if (!cam1_->read(cam1Frame)) {
      RCLCPP_WARN(this->get_logger(), "No frame from cam1");
      return;
    }
    rclcpp::Time right_stamp = this->now();

    // Convert to ROS messages with per-image timestamps and distinct frame_ids
    std_msgs::msg::Header left_header;
    left_header.stamp = left_stamp;
    left_header.frame_id = camera_info_left_.header.frame_id;

    std_msgs::msg::Header right_header;
    right_header.stamp = right_stamp;
    right_header.frame_id = camera_info_right_.header.frame_id;

    // Choose encoding and convert if publishing RGB
    std::string encoding = sensor_msgs::image_encodings::MONO8;
    cv::Mat left_to_pub = cam0Frame;
    cv::Mat right_to_pub = cam1Frame;

    if (publish_rgb_) {
      // camera pipeline already outputs BGR; convert to RGB for ROS
      cv::Mat left_rgb, right_rgb;
      cv::cvtColor(cam0Frame, left_rgb, cv::COLOR_BGR2RGB);
      cv::cvtColor(cam1Frame, right_rgb, cv::COLOR_BGR2RGB);
      left_to_pub = left_rgb;
      right_to_pub = right_rgb;
      encoding = sensor_msgs::image_encodings::RGB8;
    }

    auto imageLeftMsg =
        cv_bridge::CvImage(left_header, encoding, left_to_pub).toImageMsg();

    auto imageRightMsg =
        cv_bridge::CvImage(right_header, encoding, right_to_pub).toImageMsg();

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

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StereoCameraNode>();
  // Initialize (this will open cameras and publishers)
  node->initialize(node);

  if (!node->is_initialized()) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_camera_node"),
                 "Failed to initialize cameras. Exiting.");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
