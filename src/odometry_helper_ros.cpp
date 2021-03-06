
#include <social_force_window_planner/odometry_helper_ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace sfw_planner {

OdometryHelperRos::OdometryHelperRos(std::string odom_topic) {
  setOdomTopic(odom_topic);
}

void OdometryHelperRos::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  ROS_INFO_ONCE("[odmetry_helper] odom received!");

  // we assume that the odometry is published in the frame of the base
  // boost::mutex::scoped_lock lock(odom_mutex_);
  odom_mutex_.lock();
  base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
  base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
  base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
  base_odom_.child_frame_id = msg->child_frame_id;
  odom_mutex_.unlock();
  //  ROS_DEBUG_NAMED("dwa_local_planner", "In the odometry callback with
  //  velocity values: (%.2f, %.2f, %.2f)",
  //      base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y,
  //      base_odom_.twist.twist.angular.z);
}

// copy over the odometry information
void OdometryHelperRos::getOdom(nav_msgs::Odometry &base_odom) {
  // boost::mutex::scoped_lock lock(odom_mutex_);
  odom_mutex_.lock();
  base_odom = base_odom_;
  odom_mutex_.unlock();
}

void OdometryHelperRos::getRobotVel(geometry_msgs::PoseStamped &robot_vel) {
  // Set current velocities from odometry
  geometry_msgs::Twist global_vel;
  {
    // boost::mutex::scoped_lock lock(odom_mutex_);
    odom_mutex_.lock();
    global_vel.linear.x = base_odom_.twist.twist.linear.x;
    global_vel.linear.y = base_odom_.twist.twist.linear.y;
    global_vel.angular.z = base_odom_.twist.twist.angular.z;

    robot_vel.header.frame_id = base_odom_.child_frame_id;
    odom_mutex_.unlock();
  }
  robot_vel.pose.position.x = global_vel.linear.x;
  robot_vel.pose.position.y = global_vel.linear.y;
  robot_vel.pose.position.z = 0;
  tf2::Quaternion q;
  q.setRPY(0, 0, global_vel.angular.z);
  tf2::convert(q, robot_vel.pose.orientation);
  robot_vel.header.stamp = ros::Time();
}

void OdometryHelperRos::setOdomTopic(std::string odom_topic) {
  if (odom_topic != odom_topic_) {
    odom_topic_ = odom_topic;

    if (odom_topic_ != "") {
      ros::NodeHandle gn;
      odom_sub_ = gn.subscribe<nav_msgs::Odometry>(
          odom_topic_, 1,
          boost::bind(&OdometryHelperRos::odomCallback, this, _1));
    } else {
      odom_sub_.shutdown();
    }
  }
}

} /* namespace sfw_planner */
