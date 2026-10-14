#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("move_left_tcp_pregrasp");
  const double object_size_x = node->declare_parameter<double>("object_size_x_m", 0.10);
  const double object_size_y = node->declare_parameter<double>("object_size_y_m", 0.10);
  const double object_size_z = node->declare_parameter<double>("object_size_z_m", 0.10);
  const double gap = node->declare_parameter<double>("gap_m", 0.03);
  const double tcp_offset_z = node->declare_parameter<double>("tcp_offset_z_m", 0.16);
  if (object_size_x <= 0.0 || object_size_y <= 0.0 || object_size_z <= 0.0 ||
      gap < 0.0 || !std::isfinite(tcp_offset_z)) {
    RCLCPP_ERROR(node->get_logger(), "Invalid object dimensions, gap, or TCP offset");
    rclcpp::shutdown();
    return 1;
  }

  std::mutex pose_mutex;
  std::optional<geometry_msgs::msg::PoseStamped> object_pose;
  auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/detected_object/pose", 10,
    [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(pose_mutex);
      object_pose = *msg;
    });
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer, node, false);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  int result = 1;
  try {
    moveit::planning_interface::MoveGroupInterface move_group(node, "left_arm");
    move_group.setPlanningTime(15.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);

    const auto deadline = std::chrono::steady_clock::now() + 10s;
    geometry_msgs::msg::PoseStamped target_object;
    bool found = false;
    while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(pose_mutex);
        if (object_pose) {
          target_object = *object_pose;
          found = true;
        }
      }
      if (found) {
        break;
      }
      std::this_thread::sleep_for(100ms);
    }
    if (!found || target_object.header.frame_id != "world" ||
        (node->now() - rclcpp::Time(target_object.header.stamp)).seconds() > 2.0) {
      throw std::runtime_error("No fresh detected object Pose in world frame");
    }

    const auto arm_tf = tf_buffer.lookupTransform(
      "world", "left_arm_link6", tf2::TimePointZero, tf2::durationFromSec(2.0));
    tf2::Transform world_object, world_arm;
    tf2::fromMsg(target_object.pose, world_object);
    tf2::fromMsg(arm_tf.transform, world_arm);
    // The current URDF makes left_gripper_base coincide with left_arm_link6.
    // The 0.16 m offset is a provisional TCP position, not a measured tool tip.
    const tf2::Vector3 tcp_goal = world_object *
      tf2::Vector3(-(object_size_x / 2.0 + gap), 0.0, 0.0);
    const tf2::Vector3 arm_goal = tcp_goal -
      tf2::quatRotate(world_arm.getRotation(), tf2::Vector3(0.0, 0.0, tcp_offset_z));
    geometry_msgs::msg::Pose arm_goal_pose;
    arm_goal_pose.position.x = arm_goal.x();
    arm_goal_pose.position.y = arm_goal.y();
    arm_goal_pose.position.z = arm_goal.z();
    arm_goal_pose.orientation = tf2::toMsg(world_arm.getRotation());
    RCLCPP_INFO(node->get_logger(),
                "Object center (%.3f, %.3f, %.3f); TCP goal (%.3f, %.3f, %.3f); "
                "arm link6 goal (%.3f, %.3f, %.3f)",
                target_object.pose.position.x, target_object.pose.position.y,
                target_object.pose.position.z, tcp_goal.x(), tcp_goal.y(), tcp_goal.z(),
                arm_goal.x(), arm_goal.y(), arm_goal.z());

    moveit_msgs::msg::CollisionObject box;
    box.header.frame_id = "world";
    box.id = "red_target_box";
    shape_msgs::msg::SolidPrimitive shape;
    shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    shape.dimensions = {object_size_x, object_size_y, object_size_z};
    box.primitives.push_back(shape);
    box.primitive_poses.push_back(target_object.pose);
    box.operation = moveit_msgs::msg::CollisionObject::ADD;
    moveit::planning_interface::PlanningSceneInterface scene;
    if (!scene.applyCollisionObject(box)) {
      throw std::runtime_error("Could not add red box to MoveIt planning scene");
    }

    move_group.setPoseReferenceFrame("world");
    move_group.setEndEffectorLink("left_arm_link6");
    move_group.setStartStateToCurrentState();
    if (!move_group.setPoseTarget(arm_goal_pose, "left_arm_link6")) {
      throw std::runtime_error("MoveIt rejected the target pose");
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      throw std::runtime_error("MoveIt could not plan a collision-free pregrasp path");
    }
    RCLCPP_INFO(node->get_logger(), "Planning succeeded; executing left arm trajectory");
    if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      throw std::runtime_error("Left arm trajectory execution failed");
    }
    const auto reached_tf = tf_buffer.lookupTransform(
      "world", "left_arm_link6", tf2::TimePointZero, tf2::durationFromSec(2.0));
    tf2::Transform reached_arm;
    tf2::fromMsg(reached_tf.transform, reached_arm);
    const tf2::Vector3 reached_tcp = reached_arm * tf2::Vector3(0.0, 0.0, tcp_offset_z);
    const tf2::Vector3 goal_error = reached_tcp - tcp_goal;
    RCLCPP_INFO(node->get_logger(),
                "Reached provisional TCP (%.3f, %.3f, %.3f); goal error %.4f m",
                reached_tcp.x(), reached_tcp.y(), reached_tcp.z(), goal_error.length());
    result = 0;
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(node->get_logger(), "%s", ex.what());
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return result;
}
