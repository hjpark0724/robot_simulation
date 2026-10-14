#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <onnxruntime_cxx_api.h>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

class ColorObjectPoseNode : public rclcpp::Node
{
public:
  ColorObjectPoseNode()
  : Node("color_object_pose"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_, this, true),
    onnx_env_(ORT_LOGGING_LEVEL_WARNING, "color_object_pose")
  {
    const auto image_topic = declare_parameter<std::string>("image_topic", "/left_camera/rgbd/image");
    const auto cloud_topic = declare_parameter<std::string>("cloud_topic", "/left_camera/rgbd/points");
    const auto pose_topic = declare_parameter<std::string>("pose_topic", "/detected_object/pose");
    const auto debug_image_topic = declare_parameter<std::string>(
      "debug_image_topic", "/detected_object/debug_image");
    const auto model_path = declare_parameter<std::string>("model_path", "");
    target_class_id_ = declare_parameter<int>("target_class_id", 0);
    confidence_threshold_ = declare_parameter<double>("confidence_threshold", 0.5);
    nms_threshold_ = declare_parameter<double>("nms_threshold", 0.45);
    depth_region_fraction_ = declare_parameter<double>("depth_region_fraction", 0.5);
    target_frame_ = declare_parameter<std::string>("target_frame", "world");
    min_range_m_ = declare_parameter<double>("min_range_m", 0.1);
    max_range_m_ = declare_parameter<double>("max_range_m", 5.0);
    object_size_x_m_ = declare_parameter<double>("object_size_x_m", 0.10);
    object_size_y_m_ = declare_parameter<double>("object_size_y_m", 0.10);
    object_size_z_m_ = declare_parameter<double>("object_size_z_m", 0.10);
    object_yaw_rad_ = declare_parameter<double>("object_yaw_rad", 0.0);

    if (model_path.empty()) {
      throw std::invalid_argument("Set model_path to a YOLOv8 detection ONNX model");
    }
    if (target_class_id_ < -1 || confidence_threshold_ <= 0.0 ||
        confidence_threshold_ > 1.0 || nms_threshold_ <= 0.0 || nms_threshold_ >= 1.0 ||
        depth_region_fraction_ <= 0.0 || depth_region_fraction_ > 1.0 ||
        max_range_m_ <= min_range_m_ || object_size_x_m_ <= 0 ||
        object_size_y_m_ <= 0 || object_size_z_m_ <= 0 ||
        min_range_m_ <= 0.0 || !std::isfinite(object_yaw_rad_)) {
      throw std::invalid_argument("Invalid YOLO or object geometry parameters");
    }

    Ort::SessionOptions session_options;
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    onnx_session_ = Ort::Session(onnx_env_, model_path.c_str(), session_options);
    if (onnx_session_.GetInputCount() != 1 || onnx_session_.GetOutputCount() != 1) {
      throw std::invalid_argument("Expected a YOLOv8 detection ONNX model with one input and one output");
    }
    const auto input_shape = onnx_session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (input_shape.size() != 4 || input_shape[1] != 3 ||
        (input_shape[0] > 8) ||
        (input_shape[2] > 0 && input_shape[3] > 0 && input_shape[2] != input_shape[3])) {
      throw std::invalid_argument("Expected NCHW YOLOv8 input with three channels and square images");
    }
    model_batch_ = input_shape[0] > 0 ? static_cast<int>(input_shape[0]) : 1;
    model_size_ = input_shape[2] > 0 ? static_cast<int>(input_shape[2]) : 640;
    if (input_shape[3] > 0 && input_shape[3] != model_size_) {
      throw std::invalid_argument("YOLOv8 ONNX input must be square");
    }
    auto allocator = Ort::AllocatorWithDefaultOptions();
    input_name_ = onnx_session_.GetInputNameAllocated(0, allocator).get();
    output_name_ = onnx_session_.GetOutputNameAllocated(0, allocator).get();

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);
    debug_pub_ = create_publisher<Image>(debug_image_topic, 10);
    surface_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/detected_object/surface_point", 10);

    image_sub_.subscribe(this, image_topic, rmw_qos_profile_sensor_data);
    cloud_sub_.subscribe(this, cloud_topic, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), image_sub_, cloud_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.04));
    sync_->registerCallback(std::bind(
      &ColorObjectPoseNode::onImages, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "YOLOv8 model %s; RGB %s; cloud %s", model_path.c_str(),
                image_topic.c_str(), cloud_topic.c_str());
  }

private:
  using Image = sensor_msgs::msg::Image;
  using Cloud = sensor_msgs::msg::PointCloud2;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Cloud>;

  static std::optional<std::uint32_t> floatFieldOffset(const Cloud& cloud, const char* name)
  {
    for (const auto& field : cloud.fields) {
      if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
          field.count == 1 && field.offset + sizeof(float) <= cloud.point_step) {
        return field.offset;
      }
    }
    return std::nullopt;
  }

  static float readFloat(const std::uint8_t* point, std::uint32_t offset)
  {
    float value;
    std::memcpy(&value, point + offset, sizeof(value));
    return value;
  }

  static float median(std::vector<float>& values)
  {
    const auto middle = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
  }

  struct Detection
  {
    cv::Rect box;
    float confidence;
  };

  std::optional<Detection> detect(const cv::Mat& image, bool is_bgr)
  {
    // Letterbox without cropping so ONNX output coordinates map to the source image.
    const float scale = std::min(static_cast<float>(model_size_) / image.cols,
                                 static_cast<float>(model_size_) / image.rows);
    const int resized_width = std::min(model_size_, static_cast<int>(std::round(image.cols * scale)));
    const int resized_height = std::min(model_size_, static_cast<int>(std::round(image.rows * scale)));
    const int pad_left = (model_size_ - resized_width) / 2;
    const int pad_top = (model_size_ - resized_height) / 2;
    cv::Mat resized, letterboxed;
    cv::resize(image, resized, cv::Size(resized_width, resized_height));
    cv::copyMakeBorder(resized, letterboxed, pad_top, model_size_ - resized_height - pad_top,
                       pad_left, model_size_ - resized_width - pad_left,
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    cv::Mat blob = cv::dnn::blobFromImage(letterboxed, 1.0 / 255.0,
                                          cv::Size(model_size_, model_size_),
                                          cv::Scalar(), is_bgr, false, CV_32F);
    // Some exported models have a fixed batch larger than one. Duplicate this
    // frame to satisfy their input shape and read only the first result.
    std::vector<float> batched_blob(blob.total() * model_batch_);
    for (int batch = 0; batch < model_batch_; ++batch) {
      std::copy(blob.ptr<float>(), blob.ptr<float>() + blob.total(),
                batched_blob.begin() + batch * blob.total());
    }
    std::array<int64_t, 4> shape{model_batch_, 3, model_size_, model_size_};
    const auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto input = Ort::Value::CreateTensor<float>(memory_info, batched_blob.data(),
                                                  batched_blob.size(), shape.data(), shape.size());
    const char* input_names[] = {input_name_.c_str()};
    const char* output_names[] = {output_name_.c_str()};
    auto outputs = onnx_session_.Run(Ort::RunOptions{nullptr}, input_names, &input, 1,
                                     output_names, 1);
    const auto output_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    if (output_shape.size() != 3 || output_shape[0] != model_batch_) {
      throw std::runtime_error("Unsupported YOLOv8 output shape; export detection model without NMS");
    }
    const bool channels_first = output_shape[1] < output_shape[2];
    const int64_t channels = channels_first ? output_shape[1] : output_shape[2];
    const int64_t count = channels_first ? output_shape[2] : output_shape[1];
    if (channels < 5 || channels > 1000 || count < 1000 ||
        (target_class_id_ >= 0 && target_class_id_ >= channels - 4)) {
      throw std::runtime_error("YOLOv8 output classes do not match target_class_id");
    }
    const float* data = outputs[0].GetTensorData<float>();
    const auto at = [&](int64_t row, int64_t channel) {
      return channels_first ? data[channel * count + row] : data[row * channels + channel];
    };
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    for (int64_t row = 0; row < count; ++row) {
      float score = -1.0F;
      if (target_class_id_ >= 0) {
        score = at(row, 4 + target_class_id_);
      } else {
        for (int64_t cls = 4; cls < channels; ++cls) {
          score = std::max(score, at(row, cls));
        }
      }
      if (!std::isfinite(score) || score < confidence_threshold_) {
        continue;
      }
      const float cx = at(row, 0), cy = at(row, 1), w = at(row, 2), h = at(row, 3);
      if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(w) ||
          !std::isfinite(h) || w <= 0.0F || h <= 0.0F) {
        continue;
      }
      const int left = std::clamp(static_cast<int>(std::floor((cx - w / 2 - pad_left) / scale)),
                                  0, image.cols);
      const int top = std::clamp(static_cast<int>(std::floor((cy - h / 2 - pad_top) / scale)),
                                 0, image.rows);
      const int right = std::clamp(static_cast<int>(std::ceil((cx + w / 2 - pad_left) / scale)),
                                   0, image.cols);
      const int bottom = std::clamp(static_cast<int>(std::ceil((cy + h / 2 - pad_top) / scale)),
                                    0, image.rows);
      if (right > left && bottom > top) {
        boxes.emplace_back(left, top, right - left, bottom - top);
        scores.push_back(score);
      }
    }
    std::vector<int> selected;
    cv::dnn::NMSBoxes(boxes, scores, static_cast<float>(confidence_threshold_),
                       static_cast<float>(nms_threshold_), selected);
    if (selected.empty()) {
      return std::nullopt;
    }
    const int best = *std::max_element(selected.begin(), selected.end(),
      [&](int a, int b) { return scores[a] < scores[b]; });
    return Detection{boxes[best], scores[best]};
  }

  void publishDebug(const Image& image, const std::optional<Detection>& detection,
                    const geometry_msgs::msg::PoseStamped* pose)
  {
    if (debug_pub_->get_subscription_count() == 0) {
      return;
    }
    Image debug = image;
    cv::Mat view(debug.height, debug.width, CV_8UC3, debug.data.data(), debug.step);
    if (detection) {
      cv::rectangle(view, detection->box, cv::Scalar(0, 255, 0), 2);
    }
    const auto draw_line = [&](const std::string& value, int row) {
      const cv::Point origin(8, std::max(22, static_cast<int>(debug.height) - 205) + row * 21);
      cv::putText(view, value, origin, cv::FONT_HERSHEY_SIMPLEX, 0.52,
                  cv::Scalar(0, 0, 0), 3);
      cv::putText(view, value, origin, cv::FONT_HERSHEY_SIMPLEX, 0.52,
                  cv::Scalar(255, 255, 255), 1);
    };
    if (!detection) {
      draw_line("Red box: no detection", 0);
    } else {
      const auto& box = detection->box;
      draw_line(cv::format("Red box %.2f  px x=%d y=%d w=%d h=%d",
                           detection->confidence, box.x, box.y, box.width, box.height), 0);
      if (pose) {
        draw_line(cv::format("World center (m): x=%.3f y=%.3f z=%.3f",
                             pose->pose.position.x, pose->pose.position.y,
                             pose->pose.position.z), 1);
      } else {
        draw_line("World center: depth or TF unavailable", 1);
      }
      draw_line(cv::format("Configured size (m): %.3f x %.3f x %.3f",
                           object_size_x_m_, object_size_y_m_, object_size_z_m_), 2);
    }
    debug_pub_->publish(debug);
  }

  void onImages(const Image::ConstSharedPtr& image, const Cloud::ConstSharedPtr& cloud)
  {
    if (image->width != cloud->width || image->height != cloud->height ||
        image->height == 0 || cloud->height == 0 || cloud->is_bigendian ||
        image->step < image->width * 3 ||
        image->data.size() < static_cast<std::size_t>(image->step) * image->height ||
        cloud->row_step < cloud->width * cloud->point_step ||
        cloud->data.size() < static_cast<std::size_t>(cloud->row_step) * cloud->height) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "RGB and organized XYZ cloud dimensions or layout do not match");
      return;
    }

    const bool is_bgr = image->encoding == sensor_msgs::image_encodings::BGR8;
    if (!is_bgr && image->encoding != sensor_msgs::image_encodings::RGB8) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Expected rgb8 or bgr8 image, got %s", image->encoding.c_str());
      return;
    }

    const auto x_offset = floatFieldOffset(*cloud, "x");
    const auto y_offset = floatFieldOffset(*cloud, "y");
    const auto z_offset = floatFieldOffset(*cloud, "z");
    if (!x_offset || !y_offset || !z_offset) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Point cloud must contain float32 x, y, z fields");
      return;
    }

    const cv::Mat color(image->height, image->width, CV_8UC3,
                        const_cast<std::uint8_t*>(image->data.data()), image->step);
    const auto detection = detect(color, is_bgr);
    if (!detection) {
      publishDebug(*image, detection, nullptr);
      return;
    }

    std::vector<float> xs, ys, zs;
    // A central crop excludes most background from an imperfect detection box.
    const auto& box = detection->box;
    const int left = box.x + static_cast<int>(box.width * (1.0 - depth_region_fraction_) / 2.0);
    const int top = box.y + static_cast<int>(box.height * (1.0 - depth_region_fraction_) / 2.0);
    const int right = box.x + box.width - (left - box.x);
    const int bottom = box.y + box.height - (top - box.y);
    for (int v = top; v < bottom; v += 2) {
      for (int u = left; u < right; u += 2) {
        const auto* point = cloud->data.data() +
          static_cast<std::size_t>(v) * cloud->row_step +
          static_cast<std::size_t>(u) * cloud->point_step;
        const float x = readFloat(point, *x_offset);
        const float y = readFloat(point, *y_offset);
        const float z = readFloat(point, *z_offset);
        const double range = std::sqrt(x * x + y * y + z * z);
        if (std::isfinite(range) && range >= min_range_m_ && range <= max_range_m_) {
          xs.push_back(x);
          ys.push_back(y);
          zs.push_back(z);
        }
      }
    }
    if (xs.size() < 10) {
      publishDebug(*image, detection, nullptr);
      return;
    }

    geometry_msgs::msg::PointStamped surface_camera, surface_world;
    surface_camera.header = cloud->header;
    surface_camera.point.x = median(xs);
    surface_camera.point.y = median(ys);
    surface_camera.point.z = median(zs);
    try {
      const auto transform = tf_buffer_.lookupTransform(
        target_frame_, surface_camera.header.frame_id,
        surface_camera.header.stamp, tf2::durationFromSec(0.2));
      tf2::doTransform(surface_camera, surface_world, transform);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Cannot transform detected surface to %s: %s",
                           target_frame_.c_str(), ex.what());
      publishDebug(*image, detection, nullptr);
      return;
    }
    surface_pub_->publish(surface_world);

    // The observed colored surface is assumed to be the object's local -X face.
    // Orientation is configured, not recovered from an unmarked color blob.
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, object_yaw_rad_);
    const tf2::Vector3 center_offset = tf2::quatRotate(
      orientation, tf2::Vector3(object_size_x_m_ / 2.0, 0.0, 0.0));
    geometry_msgs::msg::PoseStamped object_pose;
    object_pose.header = surface_world.header;
    object_pose.pose.position.x = surface_world.point.x + center_offset.x();
    object_pose.pose.position.y = surface_world.point.y + center_offset.y();
    object_pose.pose.position.z = surface_world.point.z + center_offset.z();
    object_pose.pose.orientation = tf2::toMsg(orientation);
    pose_pub_->publish(object_pose);
    publishDebug(*image, detection, &object_pose);
  }

  std::string target_frame_;
  int target_class_id_, model_size_, model_batch_;
  double confidence_threshold_, nms_threshold_, depth_region_fraction_;
  double min_range_m_, max_range_m_, object_size_x_m_, object_size_y_m_, object_size_z_m_;
  double object_yaw_rad_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  Ort::Env onnx_env_;
  Ort::Session onnx_session_{nullptr};
  std::string input_name_, output_name_;
  message_filters::Subscriber<Image> image_sub_;
  message_filters::Subscriber<Cloud> cloud_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr surface_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ColorObjectPoseNode>());
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("color_object_pose"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
