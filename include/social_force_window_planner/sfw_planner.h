/*********************************************************************
 *
 * Author: Fernando Caballero Benítez
 * Author: Noé Pérez Higueras
 *********************************************************************/

#ifndef __SFW_PLANNER_H__
#define __SFW_PLANNER_H__

#include <cmath>
#include <mutex>
#include <stdio.h>
#include <vector>

// for obstacle data access
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

// Social Force Model
#include <lightsfm/angle.hpp>
#include <lightsfm/sfm.hpp>
#include <lightsfm/vector2d.hpp>

#include <social_force_window_planner/SFWPlannerConfig.h>
#include <social_force_window_planner/sensor_interface.h>
#include <social_force_window_planner/trajectory.h>
#include <social_force_window_planner/world_model.h>

// we'll take in a path as a vector of poses
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2_ros/buffer.h>

// for some datatypes
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>

namespace sfw_planner {
/**
 * @class SFWPlaner
 * @brief Computes control velocities for a robot given a costmap, a plan, and
 * the robot's position in the world.
 */
class SFWPlanner {
public:
  /**
   * @brief  Constructs a trajectory controller
   * @param world_model The WorldModel the trajectory controller uses to check
   * for collisions
   * @param costmap A reference to the Costmap the controller should use
   * @param tf tf2 buffer for listening transformations
   * @param sensor_iface pointer to the object for dealing wih the sfm agents
   * @param footprint_spec A polygon representing the footprint of the robot.
   * (Must be convex)
   * @param max_trans_vel Maximum velocity of translation
   * @param min_trans_vel Minimun velocity of translation
   * @param max_rot_vel Maximum velocity of rotation
   * @param min_rot_vel Minimum velocity of rotation
   * @param min_in_place_rot_vel minimum rotational velocity for a rotation in
   * place
   * @param max_trans_acc Maximum acceleration of translation
   * @param max_rot_acc Maximum acceleration of rotation
   * @param yaw_goal_tolerance Tolerance in angle distance (rad) to consider
   * that the goal has been reached
   * @param xy_goal_tolerance Tolerance in distance (m) to consider that the
   * goal has been reached
   * @param wp_tolerance Distance from the robot to the point of the path to to
   * calculate the velocities to apply
   * @param sim_time The maximum number of seconds to expand the movement
   * @param sim_granularity The distance between simulation points should be
   * small enough that the robot doesn't hit things
   * @param angular_sim_granularity The distance between simulation points for
   * angular velocity should be small enough that the robot doesn't hit things
   * @param people_radius radius of the approximated circunference of the
   * people's footprint
   * @param robot_radius radius of the approximated circunference of the robot
   * footprint
   * @param sfm_goal_weight weight of the attraction force to the goal
   * @param sfm_obstacle_weight weight of the repulsion force to the obstacles
   * @param sfm_people_weight weight of the respulsion force to the people
   * @param sw The weight of the cost of the social work done in the planner
   * function
   * @param cw The weight of the costmap cost in the planner function
   * @param aw The weight of the cost of angle difference between robot heading
   * and local goal in the planner function
   * @param dw The weight of the cost of the distance to the local goal in the
   * planner function
   * @param vw The weight of the cost of the velocity difference
   * @param robot_frame frame name of the robot base
   * @param planner_frame frame name of the planner
   */
  SFWPlanner(WorldModel &world_model, const costmap_2d::Costmap2D &costmap,
             tf2_ros::Buffer *tf, SFMSensorInterface *sensor_iface,
             std::vector<geometry_msgs::Point> footprint_spec,
             double max_trans_vel = 0.6, double min_trans_vel = 0.1,
             double max_rot_vel = 0.5, double min_rot_vel = 0.1,
             double min_in_place_rot_vel = 0.3, double max_trans_acc = 1.0,
             double max_rot_acc = 1.0, double yaw_goal_tolerance = 0.1,
             double xy_goal_tolerance = 0.2, double wp_tolerance = 0.5,
             double sim_time = 1.0, double sim_granularity = 0.025,
             double people_radius = 0.35, double robot_radius = 0.35,
             bool is_circular = true, double sfm_goal_weight = 2.0,
             double sfm_obstacle_weight = 15.0, double sfm_people_weight = 10.0,
             double sw = 1.0, double cw = 1.0, double aw = 1.0, double dw = 1.0,
             double vw = 1.0, std::string robot_frame = "base_link",
             std::string planner_frame = "odom");

  /**
   * @brief  Destructs a trajectory controller
   */
  ~SFWPlanner();

  /**
   * @brief Reconfigures the trajectory planner
   */
  void reconfigure(social_force_window_planner::SFWPlannerConfig &cfg);

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
  bool findBestAction(geometry_msgs::PoseStamped &global_pose,
                      geometry_msgs::PoseStamped &global_vel,
                      geometry_msgs::Twist &cmd_vel);

  /**
   * @brief  Update the plan that the controller is following
   * @param new_plan A new plan for the controller to follow
   * @param compute_dists Wheter or not to compute path/goal distances when a
   * plan is updated
   */
  bool updatePlan(const std::vector<geometry_msgs::PoseStamped> &new_plan);

  bool isGoalReached();
  void resetGoal();

  /** @brief Set the footprint specification of the robot. */
  void setFootprint(std::vector<geometry_msgs::Point> footprint) {
    footprint_spec_ = footprint;
  }

  /** @brief Return the footprint specification of the robot. */
  geometry_msgs::Polygon getFootprintPolygon() const {
    return costmap_2d::toPolygon(footprint_spec_);
  }
  std::vector<geometry_msgs::Point> getFootprint() const {
    return footprint_spec_;
  }

  visualization_msgs::MarkerArray &getMarkers();

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
                         std::vector<sfm::Agent> agents, Trajectory &traj);

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

  // boost::mutex configuration_mutex_;
  std::mutex configuration_mutex_;

  const costmap_2d::Costmap2D &costmap_; ///< @brief Provides access to cost map
                                         /// information

  std::string robot_frame_;
  std::string planner_frame_;

  WorldModel
      &world_model_; ///< @brief The world model that the controller uses for
                     /// collision detection

  SFMSensorInterface *sensor_iface_; ///< @brief sensor access for sfm agents

  std::vector<geometry_msgs::Point>
      footprint_spec_; ///< @brief The footprint
                       /// specification of the robot

  std::vector<geometry_msgs::PoseStamped>
      global_plan_; ///< @brief The global path for
                    /// the robot to follow

  double sim_time_;        ///< @brief The number of seconds each trajectory is
                           ///< "rolled-out"
  double sim_granularity_; ///< @brief The distance between simulation points
  double angular_sim_granularity_; ///< @brief The distance between angular
                                   ///< simulation
                                   /// points

  std::vector<double> linvels_;
  std::vector<double> angvels_;
  visualization_msgs::MarkerArray markers_;

  double acc_lim_trans_,
      acc_lim_rot_; ///< @brief The acceleration limits of the robot

  double max_vel_x_, min_vel_x_, max_vel_th_, min_vel_th_,
      min_in_place_vel_th_; ///< @brief Velocity limits for the controller
  // double heading_diff_to_rotate_;

  double inscribed_radius_, circumscribed_radius_;

  // SFM params
  double people_radius_;
  double robot_radius_;
  bool is_circular_;
  double sfm_goal_weight_;
  double sfm_obstacle_weight_;
  double sfm_people_weight_;

  // cost function weights
  double ws_; //= 1.2;
  double wc_; //= 2.0;
  double wa_; //= 0.7;
  double wd_; //= 1.0;
  double wv_;

  // path tracking params
  int wp_index_;
  bool running_;
  double start_x_;
  double start_y_;
  double start_t_;
  double goal_x_;
  double goal_y_;
  double goal_t_;
  double goal_lin_tolerance_;
  double goal_ang_tolerance_;
  double wp_tolerance_;
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
}; // namespace sfw_planner

#endif
