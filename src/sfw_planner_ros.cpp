
#include <social_force_window_planner/sfw_planner_ros.h>

#include <sys/time.h>

#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Path.h>
#include <social_force_window_planner/goal_functions.h>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(sfw_planner::SFWPlannerROS, nav_core::BaseLocalPlanner)

namespace sfw_planner {
void SFWPlannerROS::reconfigureCB(
    social_force_window_planner::SFWPlannerConfig &config, uint32_t level) {

  if (initialized_)
    tc_->reconfigure(config);
  // reached_goal_ = false;
}

SFWPlannerROS::SFWPlannerROS()
    : world_model_(nullptr), tc_(nullptr), costmap_ros_(nullptr), tf_(nullptr),
      setup_(false), initialized_(false), odom_helper_("odom") {} //

SFWPlannerROS::SFWPlannerROS(std::string name, tf2_ros::Buffer *tf,
                             costmap_2d::Costmap2DROS *costmap_ros)
    : world_model_(nullptr), tc_(nullptr), costmap_ros_(nullptr), tf_(nullptr),
      setup_(false), initialized_(false),
      odom_helper_("odom") { //, odom_helper_("odom")
  // initialize the planner
  initialize(name, tf, costmap_ros);
}

void SFWPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf,
                               costmap_2d::Costmap2DROS *costmap_ros) {
  if (!isInitialized()) {
    ros::NodeHandle private_nh("~/" + name);
    g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
    traj_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>(
        "trajectories", 1);

    tf_ = tf;
    costmap_ros_ = costmap_ros;

    // initialize the copy of the costmap the controller will use
    costmap_ = costmap_ros_->getCostmap();

    global_frame_ = costmap_ros_->getGlobalFrameID();
    robot_base_frame_ = costmap_ros_->getBaseFrameID();

    // Robot Configuration Parameters
    private_nh.param("max_trans_vel", max_vel_x_, 0.6);
    private_nh.param("min_trans_vel", min_vel_x_, 0.1);
    private_nh.param("max_rot_vel", max_vel_th_, 0.5);
    private_nh.param("min_rot_vel", min_vel_th_, 0.1);
    private_nh.param("max_trans_acc", max_trans_acc_, 1.0);
    private_nh.param("max_rot_acc", max_rot_acc_, 1.0);
    private_nh.param("min_in_place_rot_vel", min_in_place_vel_th_, 0.3);
    // double heading_diff_to_rotate;
    // private_nh.param("heading_diff_to_rotate", heading_diff_to_rotate, 1.0);

    // Goal tolerance parameters
    private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
    private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
    private_nh.param("wp_tolerance", wp_tolerance_, 0.5);

    private_nh.param("sim_time", sim_time_, 1.0);
    private_nh.param("sim_granularity", sim_granularity_, 0.025);
    private_nh.param("angular_sim_granularity", angular_sim_granularity_,
                     sim_granularity_);

    // Assuming this planner is being run within the navigation stack, we can
    // just do an upward search for the frequency at which its being run. This
    // also allows the frequency to be overwritten locally.
    // private_nh.param("controller_freq", controller_freq_, 15.0);

    // Dimensions
    float robot_radius;
    float people_radius;
    private_nh.param("robot_radius", robot_radius, float(0.35));
    private_nh.param("people_radius", people_radius, float(0.35));
    bool is_circular;
    private_nh.param("is_circular", is_circular, true);
    std::string odom_topic;
    private_nh.param("odom_topic", odom_topic, std::string("odom"));
    // OdometryHelperRos oh(odom_topic);
    // odom_helper_ = oh;
    odom_helper_.setOdomTopic(odom_topic);

    // if false, we just do a basic pure pursuit of the global plan
    // bool dwa;
    // private_nh.param("dwa", dwa, false);

    // SFM weights
    float sfm_goal_weight;
    float sfm_obstacle_weight;
    float sfm_people_weight;
    private_nh.param("sfm_goal_weight", sfm_goal_weight, float(2.0));
    private_nh.param("sfm_obstacle_weight", sfm_obstacle_weight, float(20.0));
    private_nh.param("sfm_people_weight", sfm_people_weight, float(12.0));

    // cost function weights
    double social_weight;
    double costmap_weight;
    double angle_weight;
    double distance_weight;
    double vel_weight;
    private_nh.param("social_weight", social_weight, 1.2);
    private_nh.param("costmap_weight", costmap_weight, 2.0);
    private_nh.param("angle_weight", angle_weight, 0.7);
    private_nh.param("distance_weight", distance_weight, 1.0);
    private_nh.param("velocity_weight", vel_weight, 1.0);

    world_model_ = new CostmapModel(*costmap_);

    footprint_spec_ = costmap_ros_->getRobotFootprint();

    sensor_iface_ =
        new SFMSensorInterface(&private_nh, tf, max_vel_x_, robot_radius,
                               people_radius, robot_base_frame_, global_frame_);

    tc_ = new SFWPlanner(
        *world_model_, *costmap_, tf, sensor_iface_, footprint_spec_,
        max_vel_x_, min_vel_x_, max_vel_th_, min_vel_th_, min_in_place_vel_th_,
        max_trans_acc_, max_rot_acc_, yaw_goal_tolerance_, xy_goal_tolerance_,
        wp_tolerance_, sim_time_, sim_granularity_, people_radius, robot_radius,
        is_circular, sfm_goal_weight, sfm_obstacle_weight, sfm_people_weight,
        social_weight, costmap_weight, angle_weight, distance_weight,
        vel_weight, robot_base_frame_, global_frame_);

    // BE CAREFUL, this will load the values of cfg params overwritting the read
    // ones from the yaml file.
    dsrv_ = new dynamic_reconfigure::Server<
        social_force_window_planner::SFWPlannerConfig>(private_nh);
    dynamic_reconfigure::Server<
        social_force_window_planner::SFWPlannerConfig>::CallbackType cb =
        boost::bind(&SFWPlannerROS::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    initialized_ = true;

  } else {
    ROS_WARN("This planner has already been initialized, doing nothing");
  }
}

SFWPlannerROS::~SFWPlannerROS() {
  // make sure to clean things up
  delete dsrv_;

  if (tc_ != nullptr)
    delete tc_;

  if (world_model_ != nullptr)
    delete world_model_;
}

bool SFWPlannerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
  if (!isInitialized()) {
    ROS_ERROR("This planner has not been initialized, please call initialize() "
              "before using this planner");
    return false;
  }

  // reset the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // reset the goal flag
  reached_goal_ = false;

  return true;
}

bool SFWPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  if (!isInitialized()) {
    ROS_ERROR("This planner has not been initialized, please call initialize() "
              "before using this planner");
    return false;
  }

  std::vector<geometry_msgs::PoseStamped> local_plan;

  // tf::Stamped<tf::Pose> global_pose;
  geometry_msgs::PoseStamped global_pose;
  if (!costmap_ros_->getRobotPose(global_pose)) {
    return false;
  }

  // TODO: Check here if we have already reached the goal

  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  // get the global plan in our frame
  if (!transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_,
                           global_frame_,
                           transformed_plan)) { // TransformGlobalPlan belongs
                                                // to goal_functions.cpp
    ROS_WARN(
        "Could not transform the global plan to the frame of the controller");
    return false;
  }

  // now we'll prune the plan based on the position of the robot
  // if(prune_plan_)
  // prunePlan(global_pose, transformed_plan, global_plan_);

  // tf::Stamped<tf::Pose> drive_cmds;
  // drive_cmds.frame_id_ = robot_base_frame_;

  geometry_msgs::PoseStamped robot_vel;
  odom_helper_.getRobotVel(robot_vel);

  // if the global plan passed in is empty... we won't do anything
  if (transformed_plan.empty())
    return false;

  // For timing uncomment
  // struct timeval start, end;
  // double start_t, end_t, t_diff;
  // gettimeofday(&start, NULL);

  geometry_msgs::PoseStamped goal_point = transformed_plan.back();

  tc_->updatePlan(transformed_plan);

  geometry_msgs::Twist drive_cmds;
  // compute what trajectory to drive along
  bool ok = tc_->findBestAction(global_pose, robot_vel, drive_cmds);

  visualization_msgs::MarkerArray markers = tc_->getMarkers();

  // For timing uncomment
  // gettimeofday(&end, NULL);
  // start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  // end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  // t_diff = end_t - start_t;
  // ROS_INFO("Cycle time: %.9f secs", t_diff);

  // pass along drive commands
  cmd_vel = drive_cmds;
  if (!ok) {
    ROS_DEBUG_NAMED("trajectory_planner_ros",
                    "The rollout planner failed to find a valid plan. This "
                    "means that the footprint of the robot was in collision "
                    "for all simulated trajectories.");
    publishPlan(transformed_plan, g_plan_pub_);
    traj_pub_.publish(markers);
    return false;
  }

  // publish information to the visualizer
  publishPlan(transformed_plan, g_plan_pub_);
  traj_pub_.publish(markers);
  return true;
}

bool SFWPlannerROS::isGoalReached() {
  if (!isInitialized()) {
    ROS_ERROR("This planner has not been initialized, please call initialize() "
              "before using this planner");
    return false;
  }
  // return flag set in controller
  // return reached_goal_;
  bool reached = tc_->isGoalReached();
  if (reached)
    printf("GOAL REACHED!!!\n\n");
  return reached;
}
} // namespace sfw_planner
