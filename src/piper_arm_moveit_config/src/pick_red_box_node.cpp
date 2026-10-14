#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

static moveit_msgs::msg::CollisionObject boxObject(
  const std::string& id, const geometry_msgs::msg::Pose& pose,
  const std::vector<double>& dimensions)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = id;
  object.header.frame_id = "world";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.assign(dimensions.begin(), dimensions.end());
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

static void moveArm(moveit::planning_interface::MoveGroupInterface& arm,
                    const geometry_msgs::msg::Pose& goal, const std::string& label)
{
  arm.setStartStateToCurrentState();
  arm.clearPoseTargets();
  if (!arm.setPoseTarget(goal, "left_arm_link6")) {
    throw std::runtime_error("MoveIt rejected " + label + " pose");
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    throw std::runtime_error("Could not plan " + label);
  }
  if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    throw std::runtime_error("Could not execute " + label);
  }
}

static void moveGripper(moveit::planning_interface::MoveGroupInterface& gripper,
                        double half_gap, const std::string& label)
{
  gripper.setStartStateToCurrentState();
  if (!gripper.setJointValueTarget(std::vector<double>{half_gap, -half_gap})) {
    throw std::runtime_error("MoveIt rejected " + label + " gripper target");
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (gripper.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS ||
      gripper.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    throw std::runtime_error("Could not " + label + " gripper");
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pick_red_box");
  const double box_x = node->declare_parameter<double>("box_x_m", 0.65);
  const double box_y = node->declare_parameter<double>("box_y_m", 0.29);
  const double box_z = node->declare_parameter<double>("box_z_m", 0.10);
  const double box_size = node->declare_parameter<double>("box_size_m", 0.05);
  const double lift_height = node->declare_parameter<double>("lift_height_m", 0.10);
  if (box_size <= 0 || lift_height <= 0 || !std::isfinite(box_x + box_y + box_z)) {
    RCLCPP_ERROR(node->get_logger(), "Invalid box pose or dimensions");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor] { executor.spin(); });
  int result = 1;
  try {
    moveit::planning_interface::MoveGroupInterface arm(node, "left_arm");
    moveit::planning_interface::MoveGroupInterface gripper(node, "left_gripper");
    arm.setPoseReferenceFrame("world");
    arm.setEndEffectorLink("left_arm_link6");
    arm.setPlanningTime(20.0);
    arm.setNumPlanningAttempts(20);
    arm.setMaxVelocityScalingFactor(0.08);
    arm.setMaxAccelerationScalingFactor(0.08);
    gripper.setPlanningTime(5.0);
    gripper.setMaxVelocityScalingFactor(0.2);
    gripper.setMaxAccelerationScalingFactor(0.2);

    moveit::planning_interface::PlanningSceneInterface scene;
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = box_x;
    box_pose.position.y = box_y;
    box_pose.position.z = box_z;
    geometry_msgs::msg::Pose stand_pose = box_pose;
    stand_pose.position.z = 0.0375;
    if (!scene.applyCollisionObjects({
          boxObject("red_target_box", box_pose, {box_size, box_size, box_size}),
          boxObject("red_target_stand", stand_pose, {0.035, 0.035, 0.075})})) {
      throw std::runtime_error("Could not add box and stand to planning scene");
    }

    // The Piper fingers extend from z=0.059 to z=0.136 in the gripper frame.
    // Pitching the flange 90 degrees points them along world +X; the jaws close on Y.
    tf2::Quaternion orientation;
    orientation.setRPY(0, M_PI_2, 0);
    geometry_msgs::msg::Pose goal;
    goal.orientation = tf2::toMsg(orientation);
    goal.position.y = box_y;
    goal.position.z = box_z + 0.003;

    RCLCPP_INFO(node->get_logger(), "Opening gripper");
    moveGripper(gripper, 0.05, "open");
    goal.position.x = box_x - 0.18;
    RCLCPP_INFO(node->get_logger(), "Moving to pregrasp");
    moveArm(arm, goal, "pregrasp");

    // Gazebo resolves the finger / box contact. Remove the box only from MoveIt
    // so that planned contact is allowed; keep the pedestal as an obstacle.
    scene.removeCollisionObjects({"red_target_box"});
    goal.position.x = box_x - 0.11;
    RCLCPP_INFO(node->get_logger(), "Advancing fingers around box");
    moveArm(arm, goal, "grasp approach");

    RCLCPP_INFO(node->get_logger(), "Closing gripper around 5 cm box");
    moveGripper(gripper, 0.018, "close");
    std::this_thread::sleep_for(1s);
    goal.position.z += lift_height;
    RCLCPP_INFO(node->get_logger(), "Lifting %.3f m", lift_height);
    moveArm(arm, goal, "lift");
    RCLCPP_INFO(node->get_logger(), "Pick motion finished; verify the Gazebo box pose before declaring a pickup");
    result = 0;
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(node->get_logger(), "%s", ex.what());
  }
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return result;
}
