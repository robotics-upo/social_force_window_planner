/**
 * Social Force Window Planner
 *
 * Author: Noé Pérez-Higueras
 * Service Robotics Lab, Pablo de Olavide University 2022
 *
 * Software License Agreement (MIT License)
 *
 */
#ifndef _SFW_PLANNER_HPP_
#define _SFW_PLANNER_HPP_

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdio.h>
#include <string>
#include <vector>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <tf2_ros/buffer.h>

// for obstacle data access
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

// Social Force Model
#include <lightsfm/angle.hpp>
#include <lightsfm/sfm.hpp>
#include <lightsfm/vector2d.hpp>

//#include <social_force_window_planner/SFWPlannerConfig.h>
#include <social_force_window_planner/costmap_model.hpp>
#include <social_force_window_planner/sensor_interface.hpp>
#include <social_force_window_planner/trajectory.hpp>

// we'll take in a path as a vector of poses
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// for some datatypes
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>

namespace social_force_window_planner {

struct ControllerParams {
  ControllerParams()
      : controller_frame_("odom"), robot_base_frame_("base_link"),
        max_vel_x_(0.7), min_vel_x_(0.1), max_vel_th_(0.5), min_vel_th_(0.1),
        max_trans_acc_(1.0), max_rot_acc_(1.0), min_in_place_vel_th_(0.3),
        yaw_goal_tolerance_(0.05), xy_goal_tolerance_(0.1), wp_tolerance_(0.5),
        sim_time_(1.0), sim_granularity_(0.025),
        angular_sim_granularity_(0.025), robot_radius_(0.35),
        people_radius_(0.35), is_circular_(true), sfm_goal_weight_(2.0),
        sfm_obstacle_weight_(20.0), sfm_people_weight_(12.0),
        social_weight_(1.2), costmap_weight_(2.0), angle_weight_(0.7),
        distance_weight_(1.0), vel_weight_(1.0) {}

  /**
   * @brief Get params from ROS parameter
   * @param node Ptr to node
   * @param name Name of plugin
   */
  void get(rclcpp_lifecycle::LifecycleNode *node, const std::string &name) {

    nav2_util::declare_parameter_if_not_declared(
        node, name + ".controller_frame", rclcpp::ParameterValue("odom"));
    node->get_parameter(name + ".controller_frame", controller_frame_);
    nav2_util::declare_parameter_if_not_declared(
        node, name + ".robot_base_frame", rclcpp::ParameterValue("base_link"));
    node->get_parameter(name + ".robot_base_frame", robot_base_frame_);

    // Robot Configuration Parameters
    nav2_util::declare_parameter_if_not_declared(node, name + ".max_trans_vel",
                                                 rclcpp::ParameterValue(0.7));
    node->get_parameter(name + ".max_trans_vel", max_vel_x_);
    nav2_util::declare_parameter_if_not_declared(node, name + ".min_trans_vel",
                                                 rclcpp::ParameterValue(0.1));
    node->get_parameter(name + ".min_trans_vel", min_vel_x_);
    nav2_util::declare_parameter_if_not_declared(node, name + ".max_rot_vel",
                                                 rclcpp::ParameterValue(0.5));
    node->get_parameter(name + ".max_rot_vel", max_vel_th_);
    nav2_util::declare_parameter_if_not_declared(node, name + ".min_rot_vel",
                                                 rclcpp::ParameterValue(0.1));
    node->get_parameter(name + ".min_rot_vel", min_vel_th_);
    nav2_util::declare_parameter_if_not_declared(node, name + ".max_trans_acc",
                                                 rclcpp::ParameterValue(1.0));
    node->get_parameter(name + ".max_trans_acc", max_trans_acc_);
    nav2_util::declare_parameter_if_not_declared(node, name + ".max_rot_acc",
                                                 rclcpp::ParameterValue(1.0));
    node->get_parameter(name + ".max_rot_acc", max_rot_acc_);
    nav2_util::declare_parameter_if_not_declared(
        node, name + ".min_in_place_rot_vel", rclcpp::ParameterValue(0.3));
    node->get_parameter(name + ".min_in_place_rot_vel", min_in_place_vel_th_);

    // Goal tolerance parameters
    nav2_util::declare_parameter_if_not_declared(
        node, name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.05));
    node->get_parameter(name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
    nav2_util::declare_parameter_if_not_declared(
        node, name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.10));
    node->get_parameter(name + ".xy_goal_tolerance", xy_goal_tolerance_);
    nav2_util::declare_parameter_if_not_declared(node, name + ".wp_tolerance",
                                                 rclcpp::ParameterValue(0.5));
    node->get_parameter(name + ".wp_tolerance", wp_tolerance_);
    nav2_util::declare_parameter_if_not_declared(node, name + ".sim_time",
                                                 rclcpp::ParameterValue(1.0));
    node->get_parameter(name + ".sim_time", sim_time_);
    nav2_util::declare_parameter_if_not_declared(
        node, name + ".sim_granularity", rclcpp::ParameterValue(0.025));
    node->get_parameter(name + ".sim_granularity", sim_granularity_);
    nav2_util::declare_parameter_if_not_declared(
        node, name + ".angular_sim_granularity",
        rclcpp::ParameterValue(sim_granularity_));
    node->get_parameter(name + ".angular_sim_granularity",
                        angular_sim_granularity_);

    // Assuming this planner is being run within the navigation stack, we can
    // just do an upward search for the frequency at which its being run. This
    // also allows the frequency to be overwritten locally.
    // private_nh.param("controller_freq", controller_freq_, 15.0);

    // Dimensions
    nav2_util::declare_parameter_if_not_declared(node, name + ".robot_radius",
                                                 rclcpp::ParameterValue(0.35));
    node->get_parameter(name + ".robot_radius", robot_radius_);
    nav2_util::declare_parameter_if_not_declared(node, name + ".people_radius",
                                                 rclcpp::ParameterValue(0.35));
    node->get_parameter(name + ".people_radius", people_radius_);
    nav2_util::declare_parameter_if_not_declared(node, name + ".is_circular",
                                                 rclcpp::ParameterValue(true));
    node->get_parameter(name + ".is_circular", is_circular_);

    // std::string odom_topic;
    // private_nh.param("odom_topic", odom_topic, std::string("odom"));
    // odom_helper_.setOdomTopic(odom_topic);

    // SFM weights
    nav2_util::declare_parameter_if_not_declared(
        node, name + ".sfm_goal_weight", rclcpp::ParameterValue(2.0));
    node->get_parameter(name + ".sfm_goal_weight", sfm_goal_weight_);
    nav2_util::declare_parameter_if_not_declared(
        node, name + ".sfm_obstacle_weight", rclcpp::ParameterValue(20.0));
    node->get_parameter(name + ".sfm_obstacle_weight", sfm_obstacle_weight_);
    nav2_util::declare_parameter_if_not_declared(
        node, name + ".sfm_people_weight", rclcpp::ParameterValue(12.0));
    node->get_parameter(name + ".sfm_people_weight", sfm_people_weight_);

    // cost function weights
    nav2_util::declare_parameter_if_not_declared(node, name + ".social_weight",
                                                 rclcpp::ParameterValue(1.2));
    node->get_parameter(name + ".social_weight", social_weight_);
    nav2_util::declare_parameter_if_not_declared(node, name + ".costmap_weight",
                                                 rclcpp::ParameterValue(2.0));
    node->get_parameter(name + ".costmap_weight", costmap_weight_);
    nav2_util::declare_parameter_if_not_declared(node, name + ".angle_weight",
                                                 rclcpp::ParameterValue(0.7));
    node->get_parameter(name + ".angle_weight", angle_weight_);
    nav2_util::declare_parameter_if_not_declared(
        node, name + ".distance_weight", rclcpp::ParameterValue(1.0));
    node->get_parameter(name + ".distance_weight", distance_weight_);
    nav2_util::declare_parameter_if_not_declared(
        node, name + ".velocity_weight", rclcpp::ParameterValue(1.0));
    node->get_parameter(name + ".velocity_weight", vel_weight_);

    // std::cout << std::endl
    //           << "SFM SENSOR INTERFACE:" << std::endl
    //           << "laser_topic: " << laser_topic_ << std::endl
    //           << "people_topic: " << people_topic_ << std::endl
    //           << "odom_topic: " << odom_topic_ << std::endl
    //           << "max_obstacle_dist: " << max_obstacle_dist_ << std::endl
    //           << "naive_goal_time: " << naive_goal_time_ << std::endl
    //           << "people_velocity: " << people_velocity_ << std::endl
    //           << std::endl;
  }

  std::string controller_frame_;
  std::string robot_base_frame_;
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

  // sim time
  double sim_time_;
  double sim_granularity_;
  double angular_sim_granularity_;

  float robot_radius_;
  float people_radius_;
  bool is_circular_;

  // double sim_period_;
  // bool rotating_to_goal_;
  // bool reached_goal_;

  // SFM weights
  float sfm_goal_weight_;
  float sfm_obstacle_weight_;
  float sfm_people_weight_;

  // cost function weights
  double social_weight_;
  double costmap_weight_;
  double angle_weight_;
  double distance_weight_;
  double vel_weight_;
};

/**
 * @class SFWPlaner
 * @brief Computes control velocities for a robot given a costmap, a plan, and
 * the robot's position in the world.
 */
class SFWPlanner {
public:
  SFWPlanner(const rclcpp_lifecycle::LifecycleNode::SharedPtr &parent,
             const std::string name,
             std::shared_ptr<SFMSensorInterface> &sensor_iface,
             const nav2_costmap_2d::Costmap2D &costmap,
             std::vector<geometry_msgs::msg::Point> footprint_spec);

  /**
   * @brief  Destructs a trajectory controller
   */
  ~SFWPlanner();

  struct vels_ {
    float vel_x;
    float vel_y;
    float vel_th;
  };

  /**
   * @brief  Given the current position, orientation, and velocity of the robot,
   * return a trajectory to follow
   * @param global_pose The current pose of the robot in world space
   * @param global_vel The current velocity of the robot in world space
   * @param drive_velocities Will be set to velocities to send to the robot base
   * @return True if a valid command was found, false otherwise
   */
  bool findBestAction(const geometry_msgs::msg::PoseStamped &global_pose,
                      const geometry_msgs::msg::Twist &global_vel,
                      geometry_msgs::msg::Twist &cmd_vel);

  /**
   * @brief  Update the plan that the controller is following
   * @param new_plan A new plan for the controller to follow
   * @param compute_dists Wheter or not to compute path/goal distances when a
   * plan is updated
   */
  bool updatePlan(const std::vector<geometry_msgs::msg::PoseStamped> &new_plan);

  bool isGoalReached();
  void resetGoal();

  /** @brief Set the footprint specification of the robot. */
  void setFootprint(std::vector<geometry_msgs::msg::Point> footprint) {
    footprint_spec_ = footprint;
  }

  /** @brief Return the footprint specification of the robot. */
  geometry_msgs::msg::Polygon getFootprintPolygon() const {
    return nav2_costmap_2d::toPolygon(footprint_spec_);
  }
  std::vector<geometry_msgs::msg::Point> getFootprint() const {
    return footprint_spec_;
  }

  visualization_msgs::msg::MarkerArray &getMarkers();

private:
  /**
   * @brief  Generate and score a single trajectory
   * @param x The x position of the robot
   * @param y The y position of the robot
   * @param theta The orientation of the robot
   * @param vlin The linear velocity of the robot
   * @param vang The angular velocity of the robot
   * @param vlin_samp The linear velocity used to seed the trajectory
   * @param vang_samp The angular velocity used to seed the trajectory
   * @param acc_trans The linear acceleration limit of the robot
   * @param acc_rot The angular acceleration limit of the robot
   * @param wpx x coordinate of the local waypoint
   * @param wpy y coordinate of the local waypoint
   * @param agents the current state of the social agents
   * @param traj Will be set to the generated trajectory with its associated
   * score
   */
  double scoreTrajectory(double x, double y, double theta, double vx, double vy,
                         double vtheta, double vx_samp, double vy_samp,
                         double vtheta_samp, double acc_x, double acc_y,
                         double acc_theta, double wpx, double wpy,
                         const std::vector<sfm::Agent> &agents,
                         Trajectory &traj);

  /**
   * @brief  compute the social work cost
   * @param agents The vector of social agents
   */
  double computeSocialWork(const std::vector<sfm::Agent> &agents);

  /**
   * @brief initialize the RViz markers for the trajectories
   */
  void initializeMarkers();

  /**
   * @brief  Checks the legality of the robot footprint at a position and
   * orientation using the world model
   * @param x_i The x position of the robot
   * @param y_i The y position of the robot
   * @param theta_i The orientation of the robot
   * @return
   */
  double footprintCost(double x_i, double y_i, double theta_i);

  /**
   * @brief  Compute if the robot can stop without colliding
   * @param vl_x The current linear velocity in x axis
   * @param vl_y The current linear velocity in y axis
   * @param va The current angular velocity
   * @param x The current position in x axis
   * @param y The current position in y axis
   * @param th The current heading
   * @param  dt The timestep to take
   * @return True if stopping is possible, False if possible collision
   */
  bool mayIStop(double vl_x, double vl_y, double va, double x, double y,
                double th, double dt);

  std::mutex configuration_mutex_;

  ControllerParams params_;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;

  std::shared_ptr<SFMSensorInterface> sensor_iface_;

  WorldModel *world_model_;

  const nav2_costmap_2d::Costmap2D &costmap_;

  std::vector<geometry_msgs::msg::Point> footprint_spec_;

  std::vector<geometry_msgs::msg::PoseStamped>
      global_plan_; ///< @brief The global path for
                    /// the robot to follow

  std::vector<double> linvels_;
  std::vector<double> angvels_;
  visualization_msgs::msg::MarkerArray markers_;

  double acc_lim_trans_,
      acc_lim_rot_; ///< @brief The acceleration limits of the robot

  double inscribed_radius_, circumscribed_radius_;

  // cost function weights
  // double ws_; //= 1.2;
  // double wc_; //= 2.0;
  // double wa_; //= 0.7;
  // double wd_; //= 1.0;
  // double wv_;

  // path tracking params
  int wp_index_;
  bool running_;
  double start_x_;
  double start_y_;
  double start_t_;
  double goal_x_;
  double goal_y_;
  double goal_t_;
  bool new_plan_;
  // double controller_freq_;
  bool goal_reached_;

  float inline normalizeAngle(float val, float min, float max) {
    float norm = 0.0;
    if (val >= min)
      norm = min + fmod((val - min), (max - min));
    else
      norm = max - fmod((min - val), (max - min));

    return norm;
  }

  /**
   * @brief  Compute x position based on velocity
   * @param  xi The current x position
   * @param  vx The current x velocity
   * @param  vy The current y velocity
   * @param  theta The current orientation
   * @param  dt The timestep to take
   * @return The new x position
   */
  inline double computeNewXPosition(double xi, double vx, double vy,
                                    double theta, double dt) {
    return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
  }

  /**
   * @brief  Compute y position based on velocity
   * @param  yi The current y position
   * @param  vx The current x velocity
   * @param  vy The current y velocity
   * @param  theta The current orientation
   * @param  dt The timestep to take
   * @return The new y position
   */
  inline double computeNewYPosition(double yi, double vx, double vy,
                                    double theta, double dt) {
    return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
  }

  /**
   * @brief  Compute orientation based on velocity
   * @param  thetai The current orientation
   * @param  vth The current theta velocity
   * @param  dt The timestep to take
   * @return The new orientation
   */
  inline double computeNewThetaPosition(double thetai, double vth, double dt) {
    return thetai + vth * dt;
  }

  // compute velocity based on acceleration
  /**
   * @brief  Compute velocity based on acceleration
   * @param vg The desired velocity, what we're accelerating up to
   * @param vi The current velocity
   * @param a_max An acceleration limit
   * @param  dt The timestep to take
   * @return The new velocity
   */
  inline double computeNewVelocity(double vg, double vi, double a_max,
                                   double dt) {
    if ((vg - vi) >= 0) {
      return std::min(vg, vi + a_max * dt);
    }
    return std::max(vg, vi - a_max * dt);
  }

  void getMaxSpeedToStopInTime(double time, double &vx, double &vy,
                               double &vth) {
    vx = acc_lim_trans_ * std::max(time, 0.0);
    vy = 0.0 * std::max(time, 0.0);
    vth = acc_lim_rot_ * std::max(time, 0.0);
  }

  double lineCost(int x0, int x1, int y0, int y1);
  double pointCost(int x, int y);
  // double headingDiff(int cell_x, int cell_y, double x, double y, double
  // heading);
};

} // namespace social_force_window_planner

#endif
