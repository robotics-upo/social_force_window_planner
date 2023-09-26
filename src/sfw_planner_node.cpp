/**
 * Social Force Window Planner
 *
 * Author: Noé Pérez-Higueras
 * Service Robotics Lab, Pablo de Olavide University 2022
 *
 * Software License Agreement (MIT License)
 *
 */

#include "social_force_window_planner/sfw_planner_node.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <cmath>
#include <nav_msgs/msg/path.hpp>
#include <sys/time.h>

using std::abs;
using std::hypot;
using std::max;
using std::min;

namespace social_force_window_planner {

/**
 * Find element in iterator with the minimum calculated value
 */
template <typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal) {
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

void SFWPlannerNode::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    const std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {

  node_ = parent;
  auto node = node_.lock();

  name_ = name;

  RCLCPP_INFO(logger_,
              "CONFIGURING CONTROLLER: %s OF TYPE "
              "social_force_window_planner::SFWPlannerNode",
              name_.c_str());

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;

  // sensor interface
  sensor_iface_ = std::make_shared<SFMSensorInterface>(node, tf, name);

  global_path_pub_ =
      node->create_publisher<nav_msgs::msg::Path>("robot_global_plan", 1);

  local_path_pub_ =
      node->create_publisher<nav_msgs::msg::Path>("robot_local_plan", 1);

  traj_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "robot_local_trajectories", 1);

  sfw_planner_ =
      std::make_shared<SFWPlanner>(node, name, sensor_iface_, *costmap_,
                                   costmap_ros_->getRobotFootprint());
}

void SFWPlannerNode::cleanup() {
  RCLCPP_INFO(logger_,
              "Cleaning up controller: %s of type"
              "social_force_window_planner::SFWPlannerNode",
              name_.c_str());
  global_path_pub_.reset();
  local_path_pub_.reset();
  traj_pub_.reset();
}

void SFWPlannerNode::activate() {
  RCLCPP_INFO(logger_,
              "Activating controller: %s of type "
              "social_force_window_planner::SFWPlannerNode",
              name_.c_str());
  global_path_pub_->on_activate();
  local_path_pub_->on_activate();
  traj_pub_->on_activate();
}

void SFWPlannerNode::deactivate() {
  RCLCPP_INFO(logger_,
              "Deactivating controller: %s of type "
              "social_force_window_planner::SFWPlannerNode",
              name_.c_str());
  global_path_pub_->on_deactivate();
  local_path_pub_->on_deactivate();
  traj_pub_->on_deactivate();
  sensor_iface_->stop();
}

void SFWPlannerNode::setPlan(const nav_msgs::msg::Path &path) {
  // RCLCPP_INFO(logger_, "setPlan called!!!!");
  sensor_iface_->start();
  global_plan_ = path;
}

nav_msgs::msg::Path SFWPlannerNode::transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped &rpose) {
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, rpose, robot_pose)) {
    throw nav2_core::PlannerException(
        "Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
  const double max_costmap_dim =
      std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
  const double max_transform_dist =
      max_costmap_dim * costmap->getResolution() / 2.0;

  // First find the closest pose on the path to the robot
  auto transformation_begin = min_by(
      global_plan_.poses.begin(), global_plan_.poses.end(),
      [&robot_pose](const geometry_msgs::msg::PoseStamped &ps) {
        return nav2_util::geometry_utils::euclidean_distance(robot_pose, ps);
      });

  // Find points definitely outside of the costmap so we won't transform them.
  auto transformation_end = std::find_if(
      transformation_begin, std::end(global_plan_.poses),
      [&](const auto &global_plan_pose) {
        return nav2_util::geometry_utils::euclidean_distance(
                   robot_pose, global_plan_pose) > max_transform_dist;
      });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto &global_plan_pose) {
    geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
    stamped_pose.header.frame_id = global_plan_.header.frame_id;
    stamped_pose.header.stamp = robot_pose.header.stamp;
    stamped_pose.pose = global_plan_pose.pose;
    transformPose(costmap_ros_->getGlobalFrameID(), stamped_pose,
                  transformed_pose); // costmap_ros_->getBaseFrameID()
    return transformed_pose;
  };

  // Transform the near part of the global plan into the robot's frame of
  // reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(transformation_begin, transformation_end,
                 std::back_inserter(transformed_plan.poses),
                 transformGlobalPoseToLocal);
  transformed_plan.header.frame_id =
      costmap_ros_->getGlobalFrameID(); // costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  global_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool SFWPlannerNode::transformPose(
    const std::string frame, const geometry_msgs::msg::PoseStamped &in_pose,
    geometry_msgs::msg::PoseStamped &out_pose) const {
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf2::Duration tf_tol = tf2::durationFromSec(0.2);
    tf_->transform(in_pose, out_pose, frame, tf_tol);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

bool SFWPlannerNode::transformPoint(
    const std::string frame, const geometry_msgs::msg::PointStamped &in_point,
    geometry_msgs::msg::PointStamped &out_point) const {

  tf2::Duration tf_tol = tf2::durationFromSec(0.2);
  try {
    tf_->transform(in_point, out_point, frame, tf_tol);
    return true;
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPoint: %s", ex.what());
  }
  return false;
}

geometry_msgs::msg::TwistStamped SFWPlannerNode::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &speed,
    nav2_core::GoalChecker * goal_checker) {

  auto node = node_.lock();
  // RCLCPP_INFO(logger_, "ComputeVelocityCommands called!!!!");
  geometry_msgs::msg::TwistStamped velStamp;
  sensor_iface_->start();

  // std::vector<geometry_msgs::msg::PoseStamped> local_plan;
  // geometry_msgs::msg::PoseStamped global_pose;
  // if (!costmap_ros_->getRobotPose(global_pose)) {
  //   return false;
  // }
  // Get robot pose in the same frame of the costmap
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(costmap_ros_->getGlobalFrameID(), pose, robot_pose)) {
    throw nav2_core::PlannerException(
        "Unable to transform robot pose into costmap's frame");
  }

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  }

  // TODO: Check here if we have already reached the goal

  // std::vector<geometry_msgs::msg::PoseStamped> transformed_plan;
  // get the global plan in our frame
  // if (!transformGlobalPlan(*tf_, global_plan_, robot_pose, *costmap_,
  //                          costmap_ros_->getGlobalFrameID(),
  //                          transformed_plan)) { // TransformGlobalPlan
  //                          belongs
  //                                               // to goal_functions.cpp
  //   RCLCPP_WARN(
  //       node_->get_logger(),
  //       "Could not transform the global plan to the frame of the
  //       controller");
  //   return velStamp;
  // }
  auto transformed_plan = transformGlobalPlan(pose);

  // geometry_msgs::msg::PoseStamped robot_vel;
  // sensor_iface_->getRobotVel(robot_vel);

  // if the global plan passed in is empty... we won't do anything
  if (transformed_plan.poses.empty()) {
    RCLCPP_INFO(
        logger_,
        "ComputeVelocityCommands. transformed plan empty. Return zero vel");
    return velStamp;
  }

  // For timing uncomment
  // struct timeval start, end;
  // double start_t, end_t, t_diff;
  // gettimeofday(&start, NULL);

  // geometry_msgs::msg::PoseStamped goal_point = transformed_plan.poses.back();

  // RCLCPP_INFO(logger_, "Updating plan in local planner. Frame: %s",
  //            transformed_plan.header.frame_id.c_str());
  sfw_planner_->updatePlan(transformed_plan.poses);

  geometry_msgs::msg::Twist drive_cmds;
  // compute what trajectory to drive along
  bool ok =
      sfw_planner_->findBestAction(robot_pose, speed, drive_cmds); // robot_vel

  visualization_msgs::msg::MarkerArray markers = sfw_planner_->getMarkers();

  // For timing uncomment
  // gettimeofday(&end, NULL);
  // start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  // end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  // t_diff = end_t - start_t;
  // ROS_INFO("Cycle time: %.9f secs", t_diff);

  // pass along drive commands

  if (!ok) {
    RCLCPP_DEBUG(node->get_logger(),
                 "The sfw planner failed to find a valid plan. This "
                 "means that the footprint of the robot was in collision "
                 "for all simulated trajectories.");
    // publishPlan(transformed_plan, g_plan_pub_);
    global_path_pub_->publish(transformed_plan);
    traj_pub_->publish(markers);
    return velStamp;
  }

  // publish information to the visualizer
  // publishPlan(transformed_plan, g_plan_pub_);
  global_path_pub_->publish(transformed_plan);
  traj_pub_->publish(markers);

  velStamp.header.stamp = node->get_clock()->now();
  velStamp.header.frame_id = costmap_ros_->getGlobalFrameID();
  velStamp.twist = drive_cmds;
  return velStamp;
}

bool SFWPlannerNode::isGoalReached() {
  // if (!isInitialized()) {
  //   RCLCPP_ERROR(
  //       node_->get_logger(),
  //       "This planner has not been initialized, please call initialize() "
  //       "before using this planner");
  //   return false;
  // }
  // return flag set in controller
  // return reached_goal_;
  bool reached = sfw_planner_->isGoalReached();
  if (reached)
    RCLCPP_INFO(logger_, "GOAL REACHED!!!\n\n");
  return reached;
}
} // namespace social_force_window_planner

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(social_force_window_planner::SFWPlannerNode,
                       nav2_core::Controller)
