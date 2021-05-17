/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Author: Noé Pérez Higueras
 *********************************************************************/
#ifndef SFW_PLANNER_ROS_H_
#define SFW_PLANNER_ROS_H_

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <ros/ros.h>

#include <social_force_window_planner/costmap_model.h>
#include <social_force_window_planner/sensor_interface.h>
#include <social_force_window_planner/sfw_planner.h>
#include <social_force_window_planner/world_model.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2_ros/buffer.h>

#include <string>

#include <angles/angles.h>

#include <nav_core/base_local_planner.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <social_force_window_planner/SFWPlannerConfig.h>

#include <social_force_window_planner/odometry_helper_ros.h>

namespace sfw_planner {
/**
 * @class TrajectoryPlannerROS
 * @brief A ROS wrapper for the trajectory controller that queries the param
 * server to construct a controller
 */
class SFWPlannerROS : public nav_core::BaseLocalPlanner {
public:
  /**
   * @brief  Default constructor for the ros wrapper
   */
  SFWPlannerROS();

  /**
   * @brief  Constructs the ros wrapper
   * @param name The name to give this instance of the trajectory planner
   * @param tf A pointer to a transform listener
   * @param costmap The cost map to use for assigning costs to trajectories
   */
  SFWPlannerROS(std::string name, tf2_ros::Buffer *tf,
                costmap_2d::Costmap2DROS *costmap_ros);

  /**
   * @brief  Constructs the ros wrapper
   * @param name The name to give this instance of the trajectory planner
   * @param tf A pointer to a transform listener
   * @param costmap The cost map to use for assigning costs to trajectories
   */
  void initialize(std::string name, tf2_ros::Buffer *tf,
                  costmap_2d::Costmap2DROS *costmap_ros);

  /**
   * @brief  Destructor for the wrapper
   */
  ~SFWPlannerROS();

  /**
   * @brief  Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base
   * @param cmd_vel Will be filled with the velocity command to be passed to the
   * robot base
   * @return True if a valid trajectory was found, false otherwise
   */
  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

  /**
   * @brief  Set the plan that the controller is following
   * @param orig_global_plan The plan to pass to the controller
   * @return True if the plan was updated successfully, false otherwise
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

  /**
   * @brief  Check if the goal pose has been achieved
   * @return True if achieved, false otherwise
   */
  bool isGoalReached();

  bool isInitialized() { return initialized_; }

  /** @brief Return the inner TrajectoryPlanner object.  Only valid after
   * initialize(). */
  SFWPlanner *getPlanner() const { return tc_; }

private:
  /**
   * @brief Callback to update the local planner's parameters based on dynamic
   * reconfigure
   */
  void reconfigureCB(social_force_window_planner::SFWPlannerConfig &config,
                     uint32_t level);

  WorldModel
      *world_model_; ///< @brief The world model that the controller will use
  SFWPlanner *tc_;   ///< @brief The trajectory controller
  SFMSensorInterface *sensor_iface_;

  costmap_2d::Costmap2DROS
      *costmap_ros_; ///< @brief The ROS wrapper for the costmap the
                     /// controller will use
  costmap_2d::Costmap2D
      *costmap_; ///< @brief The costmap the controller will use
  tf2_ros::Buffer *tf_;
  std::string
      global_frame_; ///< @brief The frame in which the controller will run
  std::string
      robot_base_frame_; ///< @brief Used as the base frame id of the robot

  std::vector<geometry_msgs::PoseStamped> global_plan_;

  // Controller freq
  // double controller_freq_;

  // Robot Configuration Parameters
  double max_vel_x_, min_vel_x_;
  double max_vel_th_, min_vel_th_;
  double max_trans_acc_;
  double max_rot_acc_;
  double min_in_place_vel_th_;

  // Goal tolerance parameters
  double yaw_goal_tolerance_;
  double xy_goal_tolerance_;
  double wp_tolerance_;

  double sim_time_;
  double sim_granularity_;
  double angular_sim_granularity_;

  double sim_period_;
  bool rotating_to_goal_;
  bool reached_goal_;

  ros::Publisher g_plan_pub_, l_plan_pub_, traj_pub_;

  dynamic_reconfigure::Server<social_force_window_planner::SFWPlannerConfig>
      *dsrv_;
  social_force_window_planner::SFWPlannerConfig default_config_;
  bool setup_;

  bool initialized_;

  sfw_planner::OdometryHelperRos odom_helper_;

  std::vector<geometry_msgs::Point> footprint_spec_;
};
} // namespace sfw_planner
#endif
