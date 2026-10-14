#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

class RedBoxCameraViewNode : public rclcpp::Node
{
public:
  RedBoxCameraViewNode()
  : Node("red_box_camera_view"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_, this, true)
  {
    const auto image_topic = declare_parameter<std::string>(
      "image_topic", "/detected_object/debug_image");
    const auto pose_topic = declare_parameter<std::string>(
      "pose_topic", "/detected_object/pose");
    const auto output_image_topic = declare_parameter<std::string>(
      "output_image_topic", "/detected_object/camera_view");
    size_x_m_ = declare_parameter<double>("size_x_m", 0.10);
    size_y_m_ = declare_parameter<double>("size_y_m", 0.10);
    size_z_m_ = declare_parameter<double>("size_z_m", 0.10);
    tcp_frame_ = declare_parameter<std::string>("tcp_frame", "left_gripper_base");
    tcp_offset_x_m_ = declare_parameter<double>("tcp_offset_x_m", 0.0);
    tcp_offset_y_m_ = declare_parameter<double>("tcp_offset_y_m", 0.0);
    tcp_offset_z_m_ = declare_parameter<double>("tcp_offset_z_m", 0.16);
    show_window_ = declare_parameter<bool>("show_window", true);
    if (size_x_m_ <= 0.0 || size_y_m_ <= 0.0 || size_z_m_ <= 0.0 ||
        tcp_frame_.empty() || !std::isfinite(tcp_offset_x_m_) ||
        !std::isfinite(tcp_offset_y_m_) || !std::isfinite(tcp_offset_z_m_)) {
      throw std::invalid_argument("Invalid box dimensions or TCP settings");
    }
    tcp_is_estimate_ = tcp_frame_ == "left_gripper_base";
    if (show_window_) {
      const char* display = std::getenv("DISPLAY");
      const char* wayland = std::getenv("WAYLAND_DISPLAY");
      if ((!display || !*display) && (!wayland || !*wayland)) {
        throw std::runtime_error("No display server; set show_window:=false and view the image in RViz");
      }
      cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    }
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(output_image_topic, 10);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      std::bind(&RedBoxCameraViewNode::onImage, this, std::placeholders::_1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, 10,
      std::bind(&RedBoxCameraViewNode::onPose, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Camera view: %s; box pose: %s", image_topic.c_str(),
                pose_topic.c_str());
    RCLCPP_INFO(get_logger(), "TCP point: %s + (%.3f, %.3f, %.3f) m%s",
                tcp_frame_.c_str(), tcp_offset_x_m_, tcp_offset_y_m_, tcp_offset_z_m_,
                tcp_is_estimate_ ? " (estimated)" : "");
  }

  ~RedBoxCameraViewNode() override
  {
    if (show_window_) {
      cv::destroyWindow(window_name_);
    }
  }

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr image)
  {
    if (image->encoding != sensor_msgs::image_encodings::RGB8 &&
        image->encoding != sensor_msgs::image_encodings::BGR8) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Expected rgb8 or bgr8 image, got %s", image->encoding.c_str());
      return;
    }
    if (image->step < image->width * 3 ||
        image->data.size() < static_cast<std::size_t>(image->step) * image->height) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid image layout");
      return;
    }
    const cv::Mat view(image->height, image->width, CV_8UC3,
                       const_cast<std::uint8_t*>(image->data.data()), image->step);
    cv::Mat bgr;
    if (image->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::cvtColor(view, bgr, cv::COLOR_RGB2BGR);
    } else {
      bgr = view.clone();
    }
    bool current_distance = false;
    if (last_distance_) {
      const double age = std::abs(
        (rclcpp::Time(image->header.stamp) - rclcpp::Time(last_distance_->stamp)).seconds());
      if (age <= 0.1) {
        current_distance = true;
        const auto label = cv::format("%s to box center: %.3f m   surface gap: %.3f m",
                                       tcp_is_estimate_ ? "TCP est" : "TCP",
                                       last_distance_->center_m, last_distance_->surface_m);
        const cv::Point origin(8, std::max(22, static_cast<int>(image->height) - 205) + 63);
        cv::putText(bgr, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.52,
                    cv::Scalar(0, 0, 0), 3);
        cv::putText(bgr, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.52,
                    cv::Scalar(255, 255, 255), 1);
      }
    }
    if (!current_distance) {
      const cv::Point origin(8, std::max(22, static_cast<int>(image->height) - 205) + 63);
      const std::string label = "TCP distance: unavailable";
      cv::putText(bgr, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.52,
                  cv::Scalar(0, 0, 0), 3);
      cv::putText(bgr, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.52,
                  cv::Scalar(255, 255, 255), 1);
    }
    sensor_msgs::msg::Image output = *image;
    output.encoding = sensor_msgs::image_encodings::BGR8;
    output.step = static_cast<std::uint32_t>(bgr.cols * bgr.channels());
    output.data.assign(bgr.ptr<std::uint8_t>(),
                       bgr.ptr<std::uint8_t>() + bgr.total() * bgr.channels());
    image_pub_->publish(output);
    if (show_window_) {
      cv::imshow(window_name_, bgr);
      cv::waitKey(1);
    }
  }

  void onPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose)
  {
    geometry_msgs::msg::PointStamped tcp_local, tcp_world;
    tcp_local.header.stamp = pose->header.stamp;
    tcp_local.header.frame_id = tcp_frame_;
    tcp_local.point.x = tcp_offset_x_m_;
    tcp_local.point.y = tcp_offset_y_m_;
    tcp_local.point.z = tcp_offset_z_m_;
    try {
      const auto transform = tf_buffer_.lookupTransform(
        pose->header.frame_id, tcp_frame_, pose->header.stamp,
        tf2::durationFromSec(0.2));
      tf2::doTransform(tcp_local, tcp_world, transform);
    } catch (const tf2::TransformException& ex) {
      last_distance_.reset();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Cannot calculate TCP distance: %s", ex.what());
      return;
    }

    const auto& object = pose->pose.position;
    const tf2::Vector3 delta(tcp_world.point.x - object.x,
                             tcp_world.point.y - object.y,
                             tcp_world.point.z - object.z);
    const auto& rotation = pose->pose.orientation;
    tf2::Quaternion object_orientation(rotation.x, rotation.y, rotation.z, rotation.w);
    if (object_orientation.length2() < 1e-12) {
      last_distance_.reset();
      return;
    }
    object_orientation.normalize();
    const tf2::Vector3 local = tf2::quatRotate(object_orientation.inverse(), delta);
    const double outside_x = std::max(0.0, std::abs(local.x()) - size_x_m_ / 2.0);
    const double outside_y = std::max(0.0, std::abs(local.y()) - size_y_m_ / 2.0);
    const double outside_z = std::max(0.0, std::abs(local.z()) - size_z_m_ / 2.0);
    const double center_distance = delta.length();
    const double surface_distance = std::sqrt(
      outside_x * outside_x + outside_y * outside_y + outside_z * outside_z);
    last_distance_ = Distance{pose->header.stamp, center_distance, surface_distance};
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Red box center %s (m): x=%.3f y=%.3f z=%.3f; size (m): %.3f x %.3f x %.3f; "
      "%s distance to center=%.3f m, surface gap=%.3f m",
      pose->header.frame_id.c_str(), pose->pose.position.x, pose->pose.position.y,
      pose->pose.position.z, size_x_m_, size_y_m_, size_z_m_,
      tcp_is_estimate_ ? "TCP estimate" : "TCP", center_distance, surface_distance);
  }

  struct Distance
  {
    builtin_interfaces::msg::Time stamp;
    double center_m;
    double surface_m;
  };

  const std::string window_name_ = "Red box camera";
  bool show_window_ = true;
  bool tcp_is_estimate_ = true;
  double size_x_m_, size_y_m_, size_z_m_;
  double tcp_offset_x_m_, tcp_offset_y_m_, tcp_offset_z_m_;
  std::string tcp_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::optional<Distance> last_distance_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RedBoxCameraViewNode>());
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("red_box_camera_view"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
