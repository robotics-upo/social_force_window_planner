/*********************************************************************
 *
 * Author: Noé Pérez Higueras
 *********************************************************************/

#include <angles/angles.h>
#include <costmap_2d/footprint.h>
#include <math.h>
#include <social_force_window_planner/sfw_planner.h>
#include <sstream>
#include <string>

#include <ros/console.h>

// for computing path distance
#include <queue>

using namespace std;
using namespace costmap_2d;

namespace sfw_planner {

void SFWPlanner::reconfigure(
    social_force_window_planner::SFWPlannerConfig &cfg) {
  social_force_window_planner::SFWPlannerConfig config(cfg);

  configuration_mutex_.lock();

  acc_lim_trans_ = config.max_trans_acc;
  acc_lim_rot_ = config.max_rot_acc;
  max_vel_x_ = config.max_trans_vel;
  min_vel_x_ = config.min_trans_vel;
  max_vel_th_ = config.max_rot_vel;
  min_vel_th_ = config.min_rot_vel;
  min_in_place_vel_th_ = config.min_in_place_rot_vel;
  goal_lin_tolerance_ = config.xy_goal_tolerance;
  goal_ang_tolerance_ = config.yaw_goal_tolerance;
  wp_tolerance_ = config.wp_tolerance;
  sim_time_ = config.sim_time;
  sim_granularity_ = config.sim_granularity;
  // angular_sim_granularity_ = config.angular_sim_granularity;

  ws_ = config.social_weight;
  wc_ = config.costmap_weight;
  wa_ = config.angle_weight;
  wd_ = config.distance_weight;
  wv_ = config.velocity_weight;

  // Initialize set of linear vels [0, max_vel_x_]
  int n_linvels_ = 4;
  double linvel_step = max_vel_x_ / n_linvels_;
  linvels_.clear();
  for (unsigned int i = 0; i <= n_linvels_; i++) {
    linvels_.push_back(i * linvel_step);
  }
  // Initialize set of angular vels [-max_vel_th, max_vel_th]
  int n_angvels_ = 3; // 3 vels for each direction
  double angvel_step = max_vel_th_ / n_angvels_;
  angvels_.clear();
  angvels_.push_back(0.0);
  for (unsigned int i = 1; i <= n_angvels_; i++) {
    angvels_.push_back(i * angvel_step);
    angvels_.push_back(i * (-angvel_step));
  }
  initializeMarkers();
  configuration_mutex_.unlock();
}

SFWPlanner::SFWPlanner(
    WorldModel &world_model, const costmap_2d::Costmap2D &costmap,
    tf2_ros::Buffer *tf, SFMSensorInterface *sensor_iface,
    std::vector<geometry_msgs::Point> footprint_spec, double max_trans_vel,
    double min_trans_vel, double max_rot_vel, double min_rot_vel,
    double min_in_place_rot_vel, double max_trans_acc, double max_rot_acc,
    double yaw_goal_tolerance, double xy_goal_tolerance, double wp_tolerance,
    double sim_time, double sim_granularity, double people_radius,
    double robot_radius, bool is_circular, double sfm_goal_weight,
    double sfm_obstacle_weight, double sfm_people_weight, double sw, double cw,
    double aw, double dw, double vw, std::string robot_frame,
    std::string planner_frame)
    : world_model_(world_model), costmap_(costmap), sensor_iface_(sensor_iface),
      footprint_spec_(footprint_spec), ws_(sw), wc_(cw), wa_(aw), wd_(dw),
      wv_(vw), robot_frame_(robot_frame), planner_frame_(planner_frame) {
  costmap_2d::calculateMinAndMaxDistances(footprint_spec_, inscribed_radius_,
                                          circumscribed_radius_);

  std::printf("\n\n\n---FOOTPRINT----\n");
  std::printf("inscribed_radius: %.3f, circumscribed_radius: %.3f\n",
              inscribed_radius_, circumscribed_radius_);
  std::printf("Footprint_specs:\n");
  for (unsigned int i = 0; i < footprint_spec_.size(); i++) {
    std::printf("point %u: x=%.3f, y=%.3f\n", (i + 1), footprint_spec_[i].x,
                footprint_spec_[i].y);
  }
  std::printf("\n\n");

  // controller_freq_ = controller_freq;
  goal_reached_ = false;

  // For pure-pursuit
  running_ = false;
  new_plan_ = false;
  wp_index_ = -1;

  acc_lim_trans_ = max_trans_acc;
  acc_lim_rot_ = max_rot_acc;
  max_vel_x_ = max_trans_vel;
  min_vel_x_ = min_trans_vel;
  max_vel_th_ = max_rot_vel;
  min_vel_th_ = min_rot_vel;
  min_in_place_vel_th_ = min_in_place_rot_vel;
  // heading_diff_to_rotate_ = heading_diff_to_rotate;
  goal_lin_tolerance_ = xy_goal_tolerance;
  goal_ang_tolerance_ = yaw_goal_tolerance;
  wp_tolerance_ = wp_tolerance;
  sim_time_ = sim_time;
  sim_granularity_ = sim_granularity;

  people_radius_ = people_radius;
  robot_radius_ = robot_radius;
  is_circular_ = is_circular;
  sfm_goal_weight_ = sfm_goal_weight;
  sfm_obstacle_weight_ = sfm_obstacle_weight;
  sfm_people_weight_ = sfm_people_weight;

  // Initialize set of linear vels [0, max_vel_x_]
  int n_linvels_ = 4;
  double linvel_step = max_vel_x_ / n_linvels_;
  std::printf("Set of linear vels:\n[");
  for (unsigned int i = 0; i <= n_linvels_; i++) {
    linvels_.push_back(i * linvel_step);
    std::printf("%.2f, ", (i * linvel_step));
  }
  std::printf("]\n");
  // Initialize set of angular vels [-max_vel_th, max_vel_th]
  int n_angvels_ = 3; // 3 vels for each direction
  double angvel_step = max_vel_th_ / n_angvels_;
  std::printf("Set of angular vels:\n[");
  angvels_.push_back(0.0);
  std::printf("%.2f, ", 0.0);
  for (unsigned int i = 1; i <= n_angvels_; i++) {
    angvels_.push_back(i * angvel_step);
    angvels_.push_back(i * (-angvel_step));
    std::printf("%.2f, ", (i * angvel_step));
    std::printf("%.2f, ", (i * (-angvel_step)));
  }
  std::printf("]\n");
  initializeMarkers();
}

SFWPlanner::~SFWPlanner() {}

void SFWPlanner::initializeMarkers() {
  markers_.markers.resize(angvels_.size() * linvels_.size());
  unsigned counter = 0;
  for (unsigned i = 0; i < linvels_.size(); i++) {
    for (unsigned j = 0; j < angvels_.size(); j++) {
      markers_.markers[counter].header.frame_id = planner_frame_;
      markers_.markers[counter].ns = "trajectories";
      markers_.markers[counter].id = counter;
      markers_.markers[counter].type = 4;
      markers_.markers[counter].action = 0;
      markers_.markers[counter].lifetime = ros::Duration(0.3);
      markers_.markers[counter].scale.x = 0.01;
      markers_.markers[counter].color.a = 1.0;
      markers_.markers[counter].pose.orientation.w = 1.0;
      counter++;
    }
  }
}

visualization_msgs::MarkerArray &SFWPlanner::getMarkers() { return markers_; }

// given the current state of the robot, find a good control command
bool SFWPlanner::findBestAction(geometry_msgs::PoseStamped &global_pose,
                                geometry_msgs::PoseStamped &global_vel,
                                geometry_msgs::Twist &cmd_vel) {
  configuration_mutex_.lock();

  goal_reached_ = false;
  double vx, vy = 0.0, vt;

  // Check we have a path and we are running
  if (!running_) {
    vx = 0.0;
    vt = 0.0;
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = vt;
    configuration_mutex_.unlock();
    return true;
  }

  // Get current robot position and velocity in X, Y and Theta
  float rx, ry, rt, rvx, rvy, rvt;
  rx = global_pose.pose.position.x;
  ry = global_pose.pose.position.y;
  rt = tf2::getYaw(global_pose.pose.orientation);
  rvx = global_vel.pose.position.x;
  rvy = global_vel.pose.position.y;
  rvt = tf2::getYaw(global_vel.pose.orientation);

  // Get the agent states
  std::vector<sfm::Agent> agents = sensor_iface_->getAgents();

  // Check if we are close enough to the goal
  double dist_goal_sq =
      (rx - goal_x_) * (rx - goal_x_) + (ry - goal_y_) * (ry - goal_y_);
  // double dist_start =
  //    sqrt((rx - start_x_) * (rx - start_x_) + (ry - start_y_) * (ry -
  //    start_y_));

  // If we are in the goal tolerance...
  if (dist_goal_sq < (goal_lin_tolerance_ * goal_lin_tolerance_)) {
    // Stop the robot
    vx = 0.0;
    vy = 0.0;

    // Goal reached
    if (fabs(goal_t_ - rt) < goal_ang_tolerance_) {
      vt = 0.0;
      running_ = false;
      goal_reached_ = true;
    } else // Rotate at minumin velocity until reaching the goal angle
    {
      float ang_diff = goal_t_ - rt;
      ang_diff = normalizeAngle(ang_diff, -M_PI, M_PI);
      if (ang_diff > 0.0)
        vt = min_in_place_vel_th_;
      else
        vt = -min_in_place_vel_th_;

      // If the robot's footprint is not a circunference,
      // we check that the rotation is valid
      if (!is_circular_) {
        Trajectory t;
        if (scoreTrajectory(rx, ry, rt, rvx, rvy, rvt, vx, vy, vt,
                            acc_lim_trans_, 0.0, acc_lim_rot_, 0.0, 0.0, agents,
                            t) < 0.0) {
          // We can not rotate without collision
          cmd_vel.linear.x = vx;
          cmd_vel.linear.y = vy;
          cmd_vel.linear.z = 0.0;
          cmd_vel.angular.x = 0.0;
          cmd_vel.angular.y = 0.0;
          cmd_vel.angular.z = vt;
          configuration_mutex_.unlock();
          return false;
        }
      }
    }

    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = vt;
    configuration_mutex_.unlock();
    return true;
  }

  // Do we have a new plan? get the closest point into the plan
  if (new_plan_) {
    new_plan_ = false;
    double dist_sq;
    double min_dist = 9999.0;
    wp_index_ = 0;
    for (int i = global_plan_.size() - 1; i >= 0; i--) {
      double wpx = global_plan_[i].pose.position.x;
      double wpy = global_plan_[i].pose.position.y;
      // dist = sqrt((rx - wpx) * (rx - wpx) + (ry - wpy) * (ry - wpy));
      dist_sq = (rx - wpx) * (rx - wpx) + (ry - wpy) * (ry - wpy);
      if (dist_sq < (wp_tolerance_ * wp_tolerance_)) {
        wp_index_ = i;
        break;
      } else if (dist_sq < min_dist) { // find the closest point if none can be
                                       // found closer in wp_tolerance_ range
        min_dist = dist_sq;
        wp_index_ = i;
      }
    }
  }

  // Get current way-point in the path
  double wpx = global_plan_[wp_index_].pose.position.x;
  double wpy = global_plan_[wp_index_].pose.position.y;

  // Is this way-point still valid?
  // double dist_swp = sqrt((rx - wpx) * (rx - wpx) + (ry - wpy) * (ry - wpy));
  double dist_swp_sq = (rx - wpx) * (rx - wpx) + (ry - wpy) * (ry - wpy);
  while (dist_swp_sq < (wp_tolerance_ * wp_tolerance_) &&
         wp_index_ < (int)global_plan_.size() - 1) {
    wp_index_++;
    wpx = global_plan_[wp_index_].pose.position.x;
    wpy = global_plan_[wp_index_].pose.position.y;
    // dist_swp = sqrt((rx - wpx) * (rx - wpx) + (ry - wpy) * (ry - wpy));
    dist_swp_sq = (rx - wpx) * (rx - wpx) + (ry - wpy) * (ry - wpy);
  }

  // Transform way-point into local robot frame and get desired x,y,theta
  double dx = (wpx - rx) * cos(rt) + (wpy - ry) * sin(rt);
  double dy = -(wpx - rx) * sin(rt) + (wpy - ry) * cos(rt);
  double dt = atan2(dy, dx);

  // If we are approaching the goal (< dist_thres meters)
  double dist_thres = 1.5;
  if (dist_goal_sq < (dist_thres * dist_thres)) {
    vx = min_vel_x_ +
         (max_vel_x_ - min_vel_x_) * sqrt(dist_goal_sq) / dist_thres;
    vy = 0.0;
    vt = min_vel_th_ +
         (max_vel_th_ - min_vel_th_) * sqrt(dist_goal_sq) / dist_thres;
    if (dt < 0.0)
      vt *= -1;

    Trajectory t;
    if (scoreTrajectory(rx, ry, rt, rvx, rvy, rvt, vx, vy, vt, acc_lim_trans_,
                        0.0, acc_lim_rot_, wpx, wpy, agents, t) != -1) {
      cmd_vel.linear.x = vx;
      cmd_vel.linear.y = vy;
      cmd_vel.linear.z = 0.0;
      cmd_vel.angular.x = 0.0;
      cmd_vel.angular.y = 0.0;
      cmd_vel.angular.z = vt;
      configuration_mutex_.unlock();
      return true;
    }
  }

  // Compute and evaluate here the different commands
  ros::Time ti = ros::Time::now();
  Trajectory best_traj;
  best_traj.cost_ = -1.0;
  Trajectory second_traj;
  second_traj.cost_ = 1000.0;
  int i = 0;
  int best_i = 0;
  double best_cost = 10000.0;
  for (double linvel : linvels_) {
    for (double angvel : angvels_) {

      if (linvel == 0.0 && angvel == 0.0)
        continue;

      markers_.markers[i].header.stamp = ti;
      Trajectory t;
      // ros::Time tic = ros::Time::now();
      double cost = scoreTrajectory(rx, ry, rt, rvx, rvy, rvt, linvel, 0.0,
                                    angvel, acc_lim_trans_, 0.0, acc_lim_rot_,
                                    wpx, wpy, agents, t);
      // ros::Time tfc = ros::Time::now();
      // double t2 = (tfc - tic).toSec();
      // printf("%i - Scoring [%.2f, %.2f] cost: %.3f, time: %.2f ms\n", i,
      // linvel,
      //       angvel, cost, (t2 * 1000.0));

      markers_.markers[i].points.clear();
      for (unsigned int p = 0; p < t.getPointsSize(); p++) {
        double x, y, th;
        t.getPoint(p, x, y, th);
        geometry_msgs::Point point;
        point.x = x;
        point.y = y;
        point.z = 0.0;
        markers_.markers[i].points.push_back(point);
      }

      if (cost < 0.0) {

        markers_.markers[i].color.r = 1.0;
        markers_.markers[i].color.g = 0.0;
        markers_.markers[i].color.b = 0.0;
        markers_.markers[i].color.a = 0.6;
      } else {
        markers_.markers[i].color.r = 0.0;
        markers_.markers[i].color.g = 0.0;
        markers_.markers[i].color.b = 1.0;
        markers_.markers[i].color.a = 0.6;
      }
      // printf(
      //    "findBestAction. Evaluated lvel: %.2f, avel: %.2f -- cost: %.4f\n ",
      //    linvel, angvel, cost);
      // if (cost >= 0.0 && cost <= second_traj.cost_ && cost > best_cost) {
      //  second_traj = t;
      //}
      if (cost >= 0.0 && cost <= best_cost) {

        // prefer higher linear vel for equal-cost trajectories
        if (cost == best_cost && linvel < best_traj.xv_) {
          continue;
        }
        best_traj = t;
        best_cost = cost;
        best_i = i;
        vx = linvel;
        vy = 0.0;
        vt = angvel;
      }
      i++;
    }
  }
  ros::Time tf = ros::Time::now();
  double time = (tf - ti).toSec();
  // std::printf("Loop vels time: %.4f secs\n", time);

  if (best_traj.cost_ != -1) {

    // If the lowest cost is not moving, try the second best traj
    /*if (vx == 0.0 && vt == 0.0 && second_traj.cost_ >= 0.0) {
      std::printf("Using second traj!!!!!\n");
      vx = second_traj.xv_;
      vy = 0.0;
      vt = second_traj.thetav_;
    }*/
    for (unsigned int a = 0; a < markers_.markers[best_i].points.size(); a++) {
      markers_.markers[best_i].points[a].z = 0.1;
    }
    markers_.markers[best_i].color.r = 0.0;
    markers_.markers[best_i].color.g = 1.0;
    markers_.markers[best_i].color.b = 0.0;
    markers_.markers[best_i].color.a = 1.0;

    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = vt;

    std::printf("BEST TRAJ FOUND -- lvel: %.2f, avel: %.2f, cost: %.3f\n\n", vx,
                vt, best_traj.cost_);

    configuration_mutex_.unlock();
    return true;
  } else {
    // Stop the robot
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
    configuration_mutex_.unlock();
    return false;
  }
}

/**
 * create and score a trajectory given the current pose of the robot and
 * selected velocities
 */
double SFWPlanner::scoreTrajectory(double x, double y, double theta, double vx,
                                   double vy, double vtheta, double vx_samp,
                                   double vy_samp, double vtheta_samp,
                                   double acc_x, double acc_y, double acc_theta,
                                   double wpx, double wpy,
                                   std::vector<sfm::Agent> agents,
                                   Trajectory &traj) {
  // make sure the configuration doesn't change mid run
  // boost::mutex::scoped_lock l(configuration_mutex_);

  // ros::Time one = ros::Time::now();
  std::vector<sfm::Agent> myagents = agents;
  // ros::Time two = ros::Time::now();
  // double t = (two - one).toSec();
  // std::printf("ScoreTrajectory [%.2f, %.2f] nags: %i, time: %.1f ms\n",
  // vx_samp,
  //       vtheta_samp, (int)myagents.size(), (t * 1000.0));

  double x_i = x;
  double y_i = y;
  double theta_i = theta;

  double vx_i, vy_i, vtheta_i;

  vx_i = vx;
  vy_i = vy;
  vtheta_i = vtheta;

  // compute the magnitude of the velocities
  double vmag = hypotf(vx_samp, vy_samp);

  // compute the number of steps we must take along this trajectory to be "safe"
  int num_steps;

  // num_steps = int(max((vmag * sim_time_) / sim_granularity_,
  // fabs(vtheta_samp) / angular_sim_granularity_) + 0.5);

  num_steps = int(sim_time_ / sim_granularity_ + 0.5);

  // we at least want to take one step... even if we won't move, we want to
  // score our current position
  if (num_steps == 0) {
    num_steps = 1;
  }

  double dt = sim_time_ / num_steps;
  double time = 0.0;

  // create a potential trajectory
  traj.resetPoints();
  traj.xv_ = vx_samp;
  traj.yv_ = vy_samp;
  traj.thetav_ = vtheta_samp;
  traj.cost_ = -1.0;

  double social_work = 0.0;
  double costmap_cost = 0.0;

  for (int i = 0; i < num_steps; ++i) {
    // get map coordinates of a point
    unsigned int cell_x, cell_y;

    // we don't want a path that goes off the know map
    if (!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)) {
      traj.cost_ = -1.0;
      std::printf("scoreTrajectory. Returning because going off the map!\n");
      return -1.0;
    }

    // check the point on the trajectory for legality
    double footprint_cost = footprintCost(x_i, y_i, theta_i);

    if (footprint_cost >= 254.0) {
      traj.cost_ = -1.0;
      std::printf("scoreTrajectory. Returning because footprint cost invalid: "
                  "%.2f!!!\n\n",
                  footprint_cost);
      return -1.0;
    }

    // if the footprint hits an obstacle this trajectory is invalid
    if (footprint_cost < 0) // -1, -2 or -3
    {
      traj.cost_ = -1.0;
      std::printf("scoreTrajectory. Returning because footprint cost invalid: "
                  "%.2f!!!\n\n",
                  footprint_cost);
      return -1.0;
    }

    costmap_cost += (footprint_cost / 255.0);

    // the point is legal... add it to the trajectory
    traj.addPoint(x_i, y_i, theta_i);

    // calculate velocities
    vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
    vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
    vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

    // calculate positions
    x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
    y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
    theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

    // ros::Time t1 = ros::Time::now();
    // Compute Social Forces
    sfm::SFM.computeForces(myagents);
    // update agents
    sfm::SFM.updatePosition(myagents, dt);
    // ros::Time t2 = ros::Time::now();
    // double tm = (t2 - t1).toSec();
    // printf("SFM computations: %.4f secs\n", tm);

    // Update agent robot that does not move using SFM
    myagents[0].position.set(x_i, y_i);
    myagents[0].yaw = utils::Angle::fromRadian(theta_i);
    myagents[0].linearVelocity = hypotf(vx_i, vy_i);
    myagents[0].angularVelocity = vtheta_i;
    myagents[0].velocity.set(vx_i, vy_i);
    // myagents[0].obstacles1.clear();
    sfm::Goal g;
    g.center.set(wpx, wpy);
    g.radius = 0.20;
    myagents[0].goals.clear();
    myagents[0].goals.push_back(g);

    // Check if a collision with a dynamic obstacle is possible
    for (unsigned int j = 1; j < myagents.size(); j++) {
      double dx = myagents[0].position.getX() - myagents[j].position.getX();
      double dy = myagents[0].position.getY() - myagents[j].position.getY();
      double d = dx * dx + dy * dy; // hypotf(dx, dy);
      if (d <= (robot_radius_ * robot_radius_)) {
        printf("scoreTrajectory. Returning because possible collision with "
               "dynamic obstacle, d: "
               "%.2f!!!\n\n",
               d);
        traj.cost_ = -1.0;
        return -1.0;
      }
    }

    social_work += computeSocialWork(myagents);
    // traj.cost_ += computeCost(myagents);

    // increment time
    time += dt;
  } // end for i < numsteps

  // Check whether the robot could stop without colliding
  // if (!mayIStop(vx_i, vy_i, vtheta_i, x_i, y_i, theta_i, dt)) {
  //  traj.cost_ = -1.0;
  //  return -1.0;
  //}

  // Compute scoring function values:
  double dx = wpx - x_i;
  double dy = wpy - y_i;
  // Distance to goal
  // double d = hypotf(dx, dy);
  double d = dx * dx + dy * dy;
  double dtheta = atan2(dy, dx);
  // compute heading diff
  double ang_diff = dtheta - theta_i;
  ang_diff = normalizeAngle(ang_diff, -M_PI, M_PI);
  ang_diff = fabs(ang_diff) / M_PI;
  // compute diff with max vel
  double vel_diff = fabs(max_vel_x_ - vx_i) / max_vel_x_;
  // average costmap cost
  costmap_cost = costmap_cost / num_steps;

  // printf("costmap_cost: %.2f\n", costmap_cost);

  // DWA cost function:
  // total_cost = path_distance_bias_ * path_cost + goal_distance_bias_ *
  // goal_cost + occdist_scale_ * occ_cost;
  double cost = (wv_ * vel_diff) + (wd_ * d) + (wa_ * ang_diff) +
                (wc_ * costmap_cost) + (ws_ * social_work);
  // std::printf("-Scoring lv: %.2f, av: %.2f, d: %.3f, sw: %.3f, hdiff: %.3f,
  // Cost:
  // "
  //       "%.4f\n",
  //       vx_samp, vtheta_samp, d, (w * social_work), ang_diff, cost);
  traj.cost_ = cost;
  return traj.cost_;
}

double SFWPlanner::computeSocialWork(const std::vector<sfm::Agent> &agents) {

  // The social work of the robot
  double wr = agents[0].forces.socialForce.norm() +
              agents[0].forces.obstacleForce.norm();

  // wr += (1.0 - agents[0].forces.desiredForce.normalized().squaredNorm());

  // utils::Vector2d robot_forces =
  //    agents[0].forces.socialForce + agents[0].forces.obstacleForce;
  // utils::Vector2d fr = robot_forces.normalized();
  // double wr = fr.squaredNorm();

  // Compute the social work provoked by the robot in the other agents
  std::vector<sfm::Agent> agent_robot;
  agent_robot.push_back(agents[0]);
  double wp = 0.0;
  for (unsigned int i = 1; i < agents.size(); i++) {
    sfm::Agent agent = agents[i];
    sfm::SFM.computeForces(agent, agent_robot);
    wp += agent.forces.socialForce.norm();
  }
  // if (agents.size() > 1)
  //  wp = wp / (agents.size() - 1);
  // printf("\t Wr: %.4f, Wp: %.4f\n", wr, wp);
  // Sum Wr and Wp
  return wr + wp;
}

// we need to take the footprint of the robot into account when we calculate
// cost to obstacles
double SFWPlanner::footprintCost(double x_i, double y_i, double theta_i) {
  // check if the footprint is legal
  return world_model_.footprintCost(x_i, y_i, theta_i, footprint_spec_,
                                    inscribed_radius_, circumscribed_radius_);
}

// We check that we can stop without colliding
bool SFWPlanner::mayIStop(double vl_x, double vl_y, double va, double x,
                          double y, double th, double dt) {
  double lvx = vl_x;
  double lvy = vl_y;
  double av = va;
  double xp = x;
  double yp = y;
  double hp = th;
  int steps = 0;
  float ldist = 0.0;
  // std::printf("MayIstop: initial velx: %.2f, vely: %.2f\n", vl_x, vl_y);
  while (lvx > 0.0 || lvy > 0.0) {
    lvx = computeNewVelocity(0.0, lvx, acc_lim_trans_, dt);
    lvy = computeNewVelocity(0.0, lvy, acc_lim_trans_, dt);
    av = computeNewVelocity(0.0, av, acc_lim_rot_, dt);

    xp = computeNewXPosition(xp, lvx, lvy, av, dt);
    yp = computeNewYPosition(yp, lvx, lvy, av, dt);
    hp = computeNewThetaPosition(hp, av, dt);

    // std::printf("MayIStop. step:%i, clvx: %.2f, clvy: %.2f, x: %.2f y:
    // %.2f\n",
    //            steps, lvx, lvy, xp, yp);

    // float lin_dist = lv * dt;
    // ldist += lin_dist;
    /*hp = hp + (av * dt);
    // normalization just in case
    hp = normalizeAngle(hp, -M_PI, M_PI);
    xp = xp + lin_dist * cos(hp); // cos(th+av*dt/2.0)
    yp = yp + lin_dist * sin(hp);
    */
    double footprint_cost = footprintCost(xp, yp, hp);

    steps++;

    if (footprint_cost < 0 || footprint_cost >= 254.0) {
      std::printf("MayIStop. COLLISION steps: %i\n", steps);
      return false;
    }
  }
  // std::printf("\n");
  // std::printf("MayIStop. ilvx: %.2f, ilvy: %.2f steps: %i, dist: %.3f\n",
  // lvx,
  //            lvy, steps, ldist);
  return true;
}

// calculate the cost of a ray-traced line
double SFWPlanner::lineCost(int x0, int x1, int y0, int y1) {
  // Bresenham Ray-Tracing
  int deltax = abs(x1 - x0); // The difference between the x's
  int deltay = abs(y1 - y0); // The difference between the y's
  int x = x0;                // Start x off at the first pixel
  int y = y0;                // Start y off at the first pixel

  int xinc1, xinc2, yinc1, yinc2;
  int den, num, numadd, numpixels;

  double line_cost = 0.0;
  double point_cost = -1.0;

  if (x1 >= x0) // The x-values are increasing
  {
    xinc1 = 1;
    xinc2 = 1;
  } else // The x-values are decreasing
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (y1 >= y0) // The y-values are increasing
  {
    yinc1 = 1;
    yinc2 = 1;
  } else // The y-values are decreasing
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay) // There is at least one x-value for every y-value
  {
    xinc1 = 0; // Don't change the x when numerator >= denominator
    yinc2 = 0; // Don't change the y for every iteration
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax; // There are more x-values than y-values
  } else {              // There is at least one y-value for every x-value
    xinc2 = 0;          // Don't change the x for every iteration
    yinc1 = 0;          // Don't change the y when numerator >= denominator
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay; // There are more y-values than x-values
  }

  for (int curpixel = 0; curpixel <= numpixels; curpixel++) {
    point_cost = pointCost(x, y); // Score the current point

    if (point_cost < 0) {
      return -1;
    }

    if (line_cost < point_cost) {
      line_cost = point_cost;
    }

    num += numadd;    // Increase the numerator by the top of the fraction
    if (num >= den) { // Check if numerator >= denominator
      num -= den;     // Calculate the new numerator value
      x += xinc1;     // Change the x as appropriate
      y += yinc1;     // Change the y as appropriate
    }
    x += xinc2; // Change the x as appropriate
    y += yinc2; // Change the y as appropriate
  }

  return line_cost;
}

double SFWPlanner::pointCost(int x, int y) {
  unsigned char cost = costmap_.getCost(x, y);
  // if the cell is in an obstacle the path is invalid
  if (cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE ||
      cost == NO_INFORMATION) {
    return -1;
  }

  return cost;
}

bool SFWPlanner::updatePlan(
    const vector<geometry_msgs::PoseStamped> &new_plan) {
  goal_reached_ = false;

  // Copy new plan
  global_plan_.clear();
  global_plan_.resize(new_plan.size());
  for (unsigned int i = 0; i < new_plan.size(); ++i) {
    global_plan_[i] = new_plan[i];
  }

  // Check plan size
  if (global_plan_.size() == 0) {
    running_ = false;
    wp_index_ = -1;
    ROS_WARN("New local plan size = 0!");
    return true;
  }

  // Set the way-point index to the first point of the path
  wp_index_ = 0;
  running_ = true;
  new_plan_ = true;

  // Set plan goal point
  geometry_msgs::PoseStamped &goal_pose = global_plan_[global_plan_.size() - 1];
  goal_x_ = goal_pose.pose.position.x;
  goal_y_ = goal_pose.pose.position.y;
  goal_t_ = tf2::getYaw(goal_pose.pose.orientation);

  // Set the plan starting point
  geometry_msgs::PoseStamped &start_pose = global_plan_[0];
  start_x_ = start_pose.pose.position.x;
  start_y_ = start_pose.pose.position.y;
  start_t_ = tf2::getYaw(start_pose.pose.orientation);

  return true;
}

bool SFWPlanner::isGoalReached() {
  if (goal_reached_) {
    goal_reached_ = false; // we reset the flag
    return true;
  }
  return goal_reached_;
}

void SFWPlanner::resetGoal() { goal_reached_ = false; }

}; // namespace sfw_planner
