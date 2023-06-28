/**
 * Social Force Window Planner
 *
 * Author: Noé Pérez-Higueras
 * Service Robotics Lab, Pablo de Olavide University 2022
 *
 * Software License Agreement (MIT License)
 *
 */
#ifndef SFW_PLANNER_NODE_HPP_
#define SFW_PLANNER_NODE_HPP_

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

#include <angles/angles.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_publisher.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
//#include <nav_msgs/msg/odometry.h>
//#include <rclcpp/rclcpp.hpp>
#include <social_force_window_planner/costmap_model.hpp>
#include <social_force_window_planner/sensor_interface.hpp>
#include <social_force_window_planner/sfw_planner.hpp>

namespace social_force_window_planner {

/**
 * @class SFWPlannerNode
 * @brief A ROS wrapper for the trajectory controller that queries the param
 * server to construct a controller
 */
class SFWPlannerNode : public nav2_core::Controller {
public:
  /**
   * @brief Construct a new SFWPlannerROS object
   *
   */
  SFWPlannerNode() = default;
  /**
   * @brief Destroy the SFWPlannerROS object
   *
   */
  ~SFWPlannerNode() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of node
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void
  configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            const std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
            const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
      override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with
   * possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param results   Output param, if not NULL, will be filled in with full
   * evaluation results
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                          const geometry_msgs::msg::Twist &velocity,
                          nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path &path) override;

 /**
    * @brief Set new speed limit from callback
    * @param speed_limit Speed limit to use
    * @param percentage Bool if the speed limit is absolute or relative
    */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Transforms global plan into same frame as pose (robot_pose), clips
   * far away poses and possibly prunes passed poses
   * @param pose robot pose to transform
   * @return Path in new frame
   */
  nav_msgs::msg::Path
  transformGlobalPlan(const geometry_msgs::msg::PoseStamped &rpose);

  /**
   * @brief Transform a pose to another frame.
   * @param frame Frame ID to transform to
   * @param in_pose Pose input to transform
   * @param out_pose transformed output
   * @return bool if successful
   */
  bool transformPose(const std::string frame,
                     const geometry_msgs::msg::PoseStamped &in_pose,
                     geometry_msgs::msg::PoseStamped &out_pose) const;

  /**
   * @brief
   *
   * @param frame
   * @param in_point
   * @param out_point
   * @return true
   * @return false
   */
  bool transformPoint(const std::string frame,
                      const geometry_msgs::msg::PointStamped &in_point,
                      geometry_msgs::msg::PointStamped &out_point) const;

  bool isGoalReached();

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<SFWPlanner> sfw_planner_;
  std::shared_ptr<SFMSensorInterface> sensor_iface_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D *costmap_;
  rclcpp::Logger logger_{rclcpp::get_logger("SFWPlanner")};

  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>>
      global_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>>
      local_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      visualization_msgs::msg::MarkerArray>>
      traj_pub_;

  WorldModel *world_model_;

  bool initialized_;
};
} // namespace social_force_window_planner
#endif
