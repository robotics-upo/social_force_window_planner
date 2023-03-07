/**
 * Social Force Window Planner
 *
 * Author: Noé Pérez-Higueras
 * Service Robotics Lab, Pablo de Olavide University 2022
 *
 * Software License Agreement (MIT License)
 *
 */

#ifndef SFW_SENSOR_HPP_
#define SFW_SENSOR_HPP_

#include <chrono>

#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <sensor_msgs/msg/range.hpp>
#include <tf2/utils.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "message_filters/subscriber.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#ifdef TF2_CPP_HEADERS
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif

// sensor input for obstacles
#include <sensor_msgs/msg/laser_scan.hpp>
// Detection input for social navigation
//#include <dynamic_obstacle_detector/DynamicObstacles.h>
#include <people_msgs/msg/people.hpp>

// Social Force Model
#include <lightsfm/angle.hpp>
#include <lightsfm/sfm.hpp>
#include <lightsfm/vector2d.hpp>

#include <chrono>
#include <functional>
#include <math.h>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace social_force_window_planner {

struct InterfaceParams {
  InterfaceParams()
      : max_robot_vel_x_(0.7), robot_radius_(0.35), person_radius_(0.35),
        robot_frame_("base_link"), controller_frame_("odom"),
        max_obstacle_dist_(3.0), naive_goal_time_(2.0), people_velocity_(1.0),
        laser_topic_("scan"), people_topic_("people"), odom_topic_("odom") {}

  /**
   * @brief Get params from ROS parameter
   * @param node Ptr to node
   * @param name Name of plugin
   */
  void get(rclcpp_lifecycle::LifecycleNode *node, const std::string &name) {

    RCLCPP_INFO_ONCE(node->get_logger(),
                     "SENSOR INTERFACE reading params of CONTROLLER: %s ",
                     name.c_str());

    nav2_util::declare_parameter_if_not_declared(node, name + ".max_trans_vel",
                                                 rclcpp::ParameterValue(0.7));
    node->get_parameter(name + ".max_trans_vel", max_robot_vel_x_);

    nav2_util::declare_parameter_if_not_declared(node, name + ".robot_radius",
                                                 rclcpp::ParameterValue(0.35));
    node->get_parameter(name + ".robot_radius", robot_radius_);

    nav2_util::declare_parameter_if_not_declared(node, name + ".person_radius",
                                                 rclcpp::ParameterValue(0.35));
    node->get_parameter(name + ".person_radius", person_radius_);

    nav2_util::declare_parameter_if_not_declared(
        node, name + ".robot_base_frame", rclcpp::ParameterValue("base_link"));
    node->get_parameter(name + ".robot_base_frame", robot_frame_);
    nav2_util::declare_parameter_if_not_declared(
        node, name + ".controller_frame", rclcpp::ParameterValue("odom"));
    node->get_parameter(name + ".controller_frame", controller_frame_);

    // sensor interface parameters
    std::string localdomain = name + ".sensor_interface";
    nav2_util::declare_parameter_if_not_declared(
        node, localdomain + ".max_obstacle_dist", rclcpp::ParameterValue(3.0));
    node->get_parameter(localdomain + ".max_obstacle_dist", max_obstacle_dist_);
    nav2_util::declare_parameter_if_not_declared(
        node, localdomain + ".naive_goal_time", rclcpp::ParameterValue(2.0));
    node->get_parameter(localdomain + ".naive_goal_time", naive_goal_time_);
    nav2_util::declare_parameter_if_not_declared(
        node, localdomain + ".people_velocity", rclcpp::ParameterValue(1.0));
    node->get_parameter(localdomain + ".people_velocity", people_velocity_);

    nav2_util::declare_parameter_if_not_declared(
        node, localdomain + ".laser_topic", rclcpp::ParameterValue("scan"));
    node->get_parameter(localdomain + ".laser_topic", laser_topic_);

    nav2_util::declare_parameter_if_not_declared(
        node, localdomain + ".people_topic", rclcpp::ParameterValue("people"));
    node->get_parameter(localdomain + ".people_topic", people_topic_);
    // std::string obs_topic;
    // nav2_util::declare_parameter_if_not_declared(
    //    node, localdomain + ".dyn_obs_topic",
    //    rclcpp::ParameterValue("obstacles"));
    // node->get_parameter(localdomain + ".dyn_obs_topic", obs_topic);

    nav2_util::declare_parameter_if_not_declared(
        node, localdomain + ".odom_topic", rclcpp::ParameterValue("odom"));
    node->get_parameter(localdomain + ".odom_topic", odom_topic_);

    RCLCPP_INFO_ONCE(node->get_logger(),
                     "\nSFM SENSOR INTERFACE:\nlaser_topic: %s\npeople_topic: "
                     "%s\nodom_topic: %s\nmax_obstacle_dist: "
                     "%.3f\nnaive_goal_time: %.2f\npeople_velocity: %.2f\n",
                     laser_topic_.c_str(), people_topic_.c_str(),
                     odom_topic_.c_str(), max_obstacle_dist_, naive_goal_time_,
                     people_velocity_);

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
  float max_robot_vel_x_;
  float robot_radius_;
  float person_radius_;
  std::string robot_frame_;
  std::string controller_frame_;
  float max_obstacle_dist_;
  float naive_goal_time_;
  float people_velocity_;
  std::string laser_topic_;
  std::string people_topic_;
  std::string odom_topic_;
};

class SFMSensorInterface {
public:
  /**
   * @brief  Default constructor
   * @param parent pointer to a ros node handle to publish to topics
   * @param tf Pointer to tf2 buffer
   **/
  SFMSensorInterface(const rclcpp_lifecycle::LifecycleNode::SharedPtr &parent,
                     const std::shared_ptr<tf2_ros::Buffer> &tf,
                     const std::string name);

  /**
   * @brief  Destructor class
   */
  ~SFMSensorInterface();

  /**
   * @brief  Callback to process the laser scan sensory input.
   * @param laser laserScan message to be processed
   */
  void laserCb(const sensor_msgs::msg::LaserScan::SharedPtr laser);

  /**
   * @brief  Callback to process the people detected in the robot vecinity.
   * @param people message with the people to be processed
   */
  void peopleCb(const people_msgs::msg::People::SharedPtr people);

  /**
   * @brief  Callback to process the odometry messages with the robot movement.
   * @param odom messages with the obstacles to be processed.
   */
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr odom);

  /**
   * @brief  Tranform a coordinate vector from one frame to another
   * @param vector coordinate vector in the origin frame
   * @param from string with the name of the origin frame
   * @param to string with the name of the target frame
   * @return coordinate vector in the target frame
   */
  geometry_msgs::msg::Vector3
  transformVector(geometry_msgs::msg::Vector3 &vector,
                  builtin_interfaces::msg::Time t, std::string from,
                  std::string to);

  /**
   * @brief  returns the vector of sfm agents
   * @return agents vector
   */
  std::vector<sfm::Agent> getAgents();

  void getOdom(nav_msgs::msg::Odometry &base_odom);
  // void getRobotVel(geometry_msgs::msg::PoseStamped &robot_vel);

  void start() { running_ = true; };
  void stop() { running_ = false; };

private:
  /**
   * @brief  publish the transformed points in RViz
   * @param points vector with the coordinates of the points
   * @return none
   */
  void publish_obstacle_points(const std::vector<utils::Vector2d> &points);

  // void updateAgents();

  // rclcpp::Node *nh_; // Pointer to the node node handle
  // rclcpp::Node::SharedPtr n_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string name_;
  // tf2_ros::Buffer *tf_buffer_; // Pointer to the tfBuffer created in the node

  InterfaceParams iface_params_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // message_filters does not support LifecycleNode
  // message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  // std::shared_ptr<tf2_ros::MessageFilter<nav_msgs::msg::Odometry>>
  // filter_odom_;
  std::mutex odom_mutex_;
  nav_msgs::msg::Odometry base_odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  // message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser_sub_;
  // std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>
  //    filter_scan_;
  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
  // message_filters::Subscriber<people_msgs::msg::People> people_sub_;
  // std::shared_ptr<tf2_ros::MessageFilter<people_msgs::msg::People>>
  //    filter_people_;
  // rclcpp::Subscription<dynamic_obstacle_detector::DynamicObstacles>::SharedPtr
  //    dyn_obs_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar_sub_;
  std::shared_ptr<
      rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>>
      points_pub_;
  // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr points_pub_;

  std::vector<sfm::Agent> agents_; // 0: robot, 1..: Others
  // sfm::Agent robot_agent_;
  std::mutex agents_mutex_;
  std::vector<utils::Vector2d> obstacles_;
  std::mutex obs_mutex_;

  bool running_;

  bool laser_received_;
  bool odom_received_;
  rclcpp::Time last_laser_;
  // rclcpp::Time last_odom_;

  people_msgs::msg::People people_;
  std::mutex people_mutex_;
  // dynamic_obstacle_detector::DynamicObstacles dyn_obs_;
  // std::mutex obs_mutex_;

  // bool use_static_map_;
  // sfm::RosMap *static_map_;
};

} // namespace social_force_window_planner

#endif
