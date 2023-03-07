/**
 * Social Force Window Planner
 *
 * Author: Noé Pérez-Higueras
 * Service Robotics Lab, Pablo de Olavide University 2022
 *
 * Software License Agreement (MIT License)
 *
 */

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <social_force_window_planner/sensor_interface.hpp>

using std::placeholders::_1;

namespace social_force_window_planner {

SFMSensorInterface::SFMSensorInterface(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr &parent,
    const std::shared_ptr<tf2_ros::Buffer> &tf, const std::string name)
    : node_(parent), tf_buffer_(tf), name_(name) {

  RCLCPP_INFO(node_->get_logger(),
              "CONFIGURING SENSOR INTERFACE of CONTROLLER: %s ", name_.c_str());
  laser_received_ = false;
  running_ = false;
  last_laser_ = parent->get_clock()->now();

  iface_params_.get(node_.get(), name);

  // Initialize SFM. Just one agent (the robot)
  agents_.resize(1);
  agents_[0].desiredVelocity = iface_params_.max_robot_vel_x_;
  agents_[0].radius = iface_params_.robot_radius_;
  agents_[0].cyclicGoals = false;
  agents_[0].teleoperated = true;
  agents_[0].groupId = -1;

  std::chrono::duration<int> buffer_timeout(1);

  laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      iface_params_.laser_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SFMSensorInterface::laserCb, this, _1));

  // message_filters does not support LifecycleNode
  // laser_sub_.subscribe(this, iface_params_.laser_topic_,
  //                      rclcpp::SensorDataQoS()); // this or node_
  // filter_scan_ =
  //     std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
  //         laser_sub_, *tf_buffer_, iface_params_.robot_frame_, 100,
  //         node_->get_node_logging_interface(),
  //         node_->get_node_clock_interface(), buffer_timeout);
  // // Register a callback with tf2_ros::MessageFilter to be called when
  // // transforms are available
  // filter_scan_->registerCallback(&SFMSensorInterface::laserCb, this);

  people_sub_ = node_->create_subscription<people_msgs::msg::People>(
      iface_params_.people_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SFMSensorInterface::peopleCb, this, _1));

  // people_sub_.subscribe(this, iface_params_.people_topic_);
  // filter_people_ =
  //     std::make_shared<tf2_ros::MessageFilter<people_msgs::msg::People>>(
  //         people_sub_, *tf_buffer_, iface_params_.controller_frame_, 100,
  //         node_->get_node_logging_interface(),
  //         node_->get_node_clock_interface(), buffer_timeout);
  // // Register a callback with tf2_ros::MessageFilter to be called when
  // // transforms are available
  // filter_people_->registerCallback(&SFMSensorInterface::peopleCb, this);

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      iface_params_.odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SFMSensorInterface::odomCb, this, _1));

  // odom_sub_.subscribe(this, iface_params_.odom_topic_);
  // filter_odom_ =
  //     std::make_shared<tf2_ros::MessageFilter<nav_msgs::msg::Odometry>>(
  //         odom_sub_, *tf_buffer_, iface_params_.controller_frame_, 100,
  //         node_->get_node_logging_interface(),
  //         node_->get_node_clock_interface(), buffer_timeout);
  // // Register a callback with tf2_ros::MessageFilter to be called when
  // // transforms are available
  // filter_odom_->registerCallback(&SFMSensorInterface::odomCb, this);

  points_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/sfm/markers/obstacle_points", 0);
  points_pub_->on_activate();

  // sonars ?
  // sonar_sub_= n_.subscribe<sensor_msgs::Range>(
  //    rt.c_str(), 1, boost::bind(&SFMSensorInterface::sonarCb, this, _1));
}

/**
 * @brief  Destructor class
 */
SFMSensorInterface::~SFMSensorInterface() {}

/**
 * @brief  Callback to process the laser scan sensory input.
 * @param laser laserScan message to be processed
 */
void SFMSensorInterface::laserCb(
    const sensor_msgs::msg::LaserScan::SharedPtr laser) {

  if (!running_ || !odom_received_)
    return;

  laser_received_ = true;
  last_laser_ = node_->get_clock()->now();
  odom_mutex_.lock();
  builtin_interfaces::msg::Time t = base_odom_.header.stamp;
  odom_mutex_.unlock();

  RCLCPP_INFO_ONCE(node_->get_logger(), "laser received");

  std::vector<utils::Vector2d> points;
  float angle = laser->angle_min;

  for (unsigned int i = 0; i < laser->ranges.size(); i++) {

    if (!std::isnan(laser->ranges[i]) && std::isfinite(laser->ranges[i]) &&
        laser->ranges[i] < iface_params_.max_obstacle_dist_) {

      utils::Vector2d point(laser->ranges[i] * cos(angle),
                            laser->ranges[i] * sin(angle));
      points.push_back(point);
    }
    angle += laser->angle_increment;
    // alpha += angle_inc;
  }

  if (points.empty()) {
    RCLCPP_WARN(node_->get_logger(), "laser points are empty!");
    obs_mutex_.lock();
    obstacles_ = points;
    obs_mutex_.unlock();
    return;
  }

  // Transform points to
  // controller_frame_ if necessary
  if (laser->header.frame_id != iface_params_.controller_frame_) {
    geometry_msgs::msg::PointStamped out;
    // builtin_interfaces::msg::Time t = node_->get_clock()->now();
    for (unsigned int i = 0; i < points.size(); i++) {
      geometry_msgs::msg::PointStamped in;
      in.header.frame_id = laser->header.frame_id;
      in.header.stamp = t;
      in.point.x = points[i].getX();
      in.point.y = points[i].getY();
      in.point.z = 0.0;
      try {
        geometry_msgs::msg::PointStamped out =
            tf_buffer_->transform(in, iface_params_.controller_frame_);
        points[i].setX(out.point.x);
        points[i].setY(out.point.y);

      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(node_->get_logger(),
                    "Could NOT transform "
                    "laser point %i to %s: "
                    "%s",
                    (int)(i + 1), iface_params_.controller_frame_.c_str(),
                    ex.what());
        continue;
      }
    }
  }

  // Now check if the points
  // belong to a person or an
  // dynamic obstacle

  // transform people positions to
  // rcontroller frame
  people_mutex_.lock();
  people_msgs::msg::People people = people_;
  people_mutex_.unlock();

  std::vector<geometry_msgs::msg::Point> people_points;
  if (!people.people.empty() &&
      people.header.frame_id != iface_params_.controller_frame_) {
    geometry_msgs::msg::PointStamped person_point;
    person_point.header = people.header;

    for (auto person : people.people) {
      person_point.point = person.position;
      person_point.header.stamp = t; // node_->get_clock()->now();

      try {
        geometry_msgs::msg::PointStamped p_point = tf_buffer_->transform(
            person_point, iface_params_.controller_frame_);
        people_points.push_back(p_point.point);
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(node_->get_logger(),
                    "Could NOT transform "
                    "person point to %s: "
                    "%s",
                    iface_params_.controller_frame_.c_str(), ex.what());
        return;
      }
    }
  } else if (!people.people.empty()) {
    for (auto person : people.people) {
      people_points.push_back(person.position);
    }
  }
  // Remove the points in the
  // people radius
  if (!people_points.empty()) {
    std::vector<utils::Vector2d> points_aux;
    for (utils::Vector2d p : points) {
      bool remove = false;
      for (auto person : people_points) {
        float dx = p.getX() - person.x;
        float dy = p.getY() - person.y;
        float d = std::hypotf(dx, dy);
        if (d <= iface_params_.person_radius_) {
          remove = true;
          break;
        }
      }
      if (!remove)
        points_aux.push_back(p);
    }
    points.clear();
    points = points_aux;
  }
  // we can have one point per
  // sector as much
  // if (points.empty()) {
  //   obstacles_ = points;
  //   return;
  // }

  // // transform dynamic obstacles
  // // positions to robot frame
  // // obs_mutex_.lock();
  // dynamic_obstacle_detector::DynamicObstacles obstacles = dyn_obs_;
  // // obs_mutex_.unlock();

  // std::vector<geometry_msgs::msg::Point> ob_points;
  // if (obstacles.header.frame_id != odom_frame_) {
  //   geometry_msgs::msg::PointStamped ob_point;
  //   ob_point.header = obstacles.header;
  //   // ob_point.stamp = rclcpp::Node();
  //   for (auto obstacle : obstacles.obstacles) {
  //     ob_point.point = obstacle.position;
  //     ob_point.header.stamp = rclcpp::Node(0);
  //     try {
  //       geometry_msgs::msg::PointStamped o_point =
  //           tf_buffer_->transform(ob_point, odom_frame_);
  //       ob_points.push_back(o_point.point);
  //     } catch (tf2::TransformException &ex) {
  //       RCLCPP_WARN(this->get_logger(),
  //                   "Could NOT transform "
  //                   "obstacle point to %s: "
  //                   "%s",
  //                   odom_frame_.c_str(), ex.what());
  //       return;
  //     }
  //   }
  // }
  // // Remove the points in the
  // // object radius (approximated
  // // by person radius because we
  // // do not know the radius)
  // if (!ob_points.empty()) {
  //   std::vector<utils::Vector2d> points_aux;
  //   for (utils::Vector2d p : points) {
  //     bool remove = false;
  //     for (auto ob : ob_points) {
  //       float dx = p.getX() - ob.x;
  //       float dy = p.getY() - ob.y;
  //       float d = std::hypotf(dx, dy);
  //       if (d <= person_radius_) {
  //         remove = true;
  //         break;
  //       }
  //     }
  //     if (!remove)
  //       points_aux.push_back(p);
  //   }
  //   points.clear();
  //   points = points_aux;
  // }
  obs_mutex_.lock();
  obstacles_ = points;
  obs_mutex_.unlock();
  // RCLCPP_WARN(node_->get_logger(), "laserCb. points.size: %i!!!",
  //             (int)points.size());
  publish_obstacle_points(points);
}

/**
 * @brief  publish the transformed points in RViz
 * @param points vector with the coordinates of the points
 * @return none
 */
void SFMSensorInterface::publish_obstacle_points(
    const std::vector<utils::Vector2d> &points) {

  visualization_msgs::msg::Marker m;
  m.header.frame_id = iface_params_.controller_frame_; // robot_frame_
  m.header.stamp = node_->get_clock()->now();
  m.ns = "sfm_obstacle_points";
  m.type = visualization_msgs::msg::Marker::POINTS;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.15;
  m.scale.y = 0.15;
  m.scale.z = 0.15;
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.id = 1000;
  m.lifetime = rclcpp::Duration(0.3);
  // printf("Published Obstacles: ");
  for (utils::Vector2d p : points) {
    geometry_msgs::msg::Point pt;
    pt.x = p.getX();
    pt.y = p.getY();
    // printf("x: %.2f, y: %.2f -", pt.x, pt.y);
    pt.z = 0.2;
    m.points.push_back(pt);
  }
  // printf("\n");
  points_pub_->publish(m);
}

// /**
//  * @brief  Callback to process the moving obstacles detected in the robot
//  * vecinity.
//  * @param obs messages with the obstacles to be processed.
//  */
// void SFMSensorInterface::dynamicObsCb(
//     const dynamic_obstacle_detector::DynamicObstacles::ConstPtr &obs) {
//   RCLCPP_INFO_ONCE(this->get_logger(), "Dynamic obs received");

//   // obs_mutex_.lock();
//   dyn_obs_ = *obs;
//   // obs_mutex_.unlock();

//   std::vector<sfm::Agent> agents;

//   // check if people are not in odom frame
//   geometry_msgs::msg::PointStamped ps;
//   for (unsigned i = 0; i < obs->obstacles.size(); i++) {
//     sfm::Agent ag;
//     ps.header.frame_id = obs->header.frame_id;
//     ps.header.stamp = rclcpp::Node(0);
//     ps.point = obs->obstacles[i].position;
//     if (obs->header.frame_id != odom_frame_) {
//       geometry_msgs::msg::PointStamped p;
//       try {
//         p = tf_buffer_->transform(ps, odom_frame_);
//         ps = p;
//       } catch (tf2::TransformException &ex) {
//         RCLCPP_WARN(this->get_logger(), "No transform %s", ex.what());
//       }
//     }
//     ag.position.set(ps.point.x, ps.point.y);

//     geometry_msgs::msg::Vector3 velocity;
//     velocity.x = obs->obstacles[i].velocity.x;
//     velocity.y = obs->obstacles[i].velocity.y;

//     geometry_msgs::msg::Vector3 localV = SFMSensorInterface::transformVector(
//         velocity, obs->header.frame_id, odom_frame_);

//     ag.yaw = utils::Angle::fromRadian(atan2(localV.y, localV.x));
//     ag.velocity.set(localV.x, localV.y);
//     ag.linearVelocity = ag.velocity.norm();
//     ag.radius = person_radius_;
//     ag.teleoperated = false;
//     // if (fabs(people->people[i].vel) < 0.05) {
//     //	agents[i+1].velocity.set(0,0);
//     //}

//     // The SFM requires a local goal for each agent. We will assume that the
//     // goal for people depends on its current velocity
//     ag.goals.clear();
//     sfm::Goal naiveGoal;
//     // No group consideration for the moment
//     utils::Vector2d v = ag.position + naive_goal_time_ * ag.velocity;
//     naiveGoal.center.set(v.getX(), v.getY());
//     naiveGoal.radius = person_radius_;
//     ag.goals.push_back(naiveGoal);
//     ag.desiredVelocity = people_velocity_;
//     ag.groupId = -1;
//     agents.push_back(ag);
//   }

//   // Fill the obstacles of the agents
//   std::vector<utils::Vector2d> obs_points = obstacles_;
//   for (unsigned int i = 0; i < agents.size(); i++) {
//     agents[i].obstacles1.clear();
//     agents[i].obstacles1 = obs_points;
//   }

//   agents_mutex_.lock();
//   agents_.resize(obs->obstacles.size() + 1);
//   agents_[0].obstacles1 = obs_points;
//   for (unsigned int i = 1; i < agents_.size(); i++)
//     agents_[i] = agents[i - 1];
//   agents_mutex_.unlock();
// }

/**
 * @brief  Callback to process the people detected in the robot vecinity.
 * @param people message with the people to be processed
 */
void SFMSensorInterface::peopleCb(
    const people_msgs::msg::People::SharedPtr people) {

  if (!running_ || !odom_received_)
    return;

  RCLCPP_INFO_ONCE(node_->get_logger(), "People received");
  people_mutex_.lock();
  people_ = *people;
  people_mutex_.unlock();

  odom_mutex_.lock();
  builtin_interfaces::msg::Time t = base_odom_.header.stamp;
  odom_mutex_.unlock();

  // RCLCPP_INFO(node_->get_logger(), "PeopleCb.GetAgents: %i",
  //             (int)people_.people.size());
  // for (auto p : people_.people) {
  //   RCLCPP_INFO(node_->get_logger(),
  //               "\tPerson--x: %.2f, y:%.2f, vx: %.2f, vy: %.2f, vz: %.2f",
  //               p.position.x, p.position.y, p.velocity.x, p.velocity.y,
  //               p.velocity.z);
  // }

  std::vector<sfm::Agent> agents;

  // check if people are not in odom frame
  geometry_msgs::msg::PoseStamped ps;
  for (unsigned i = 0; i < people->people.size(); i++) {
    sfm::Agent ag;
    ag.id = std::stoi(people->people[i].tags[0]);
    ag.groupId = std::stoi(people->people[i].tags[1]);

    ps.header.frame_id = people->header.frame_id;
    // builtin_interfaces::msg::Time t = node_->get_clock()->now();
    ps.header.stamp = t;
    ps.pose.position = people->people[i].position;
    ps.pose.position.z = 0.0;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, people->people[i].position.z);
    ps.pose.orientation = tf2::toMsg(quat);
    if (people->header.frame_id != iface_params_.controller_frame_) {
      geometry_msgs::msg::PoseStamped p;
      try {
        p = tf_buffer_->transform(ps, iface_params_.controller_frame_);
        ps = p;
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(node_->get_logger(), "PeopleCallback. No transform %s",
                    ex.what());
        return;
      }
    }
    ag.position.set(ps.pose.position.x, ps.pose.position.y);

    geometry_msgs::msg::Vector3 velocity;
    velocity.x = people->people[i].velocity.x;
    velocity.y = people->people[i].velocity.y;
    velocity.z = 0.0;

    geometry_msgs::msg::Vector3 localV = SFMSensorInterface::transformVector(
        velocity, t, people->header.frame_id, iface_params_.controller_frame_);

    ag.velocity.set(localV.x, localV.y);
    ag.linearVelocity = ag.velocity.norm();
    if (fabs(ag.linearVelocity) < 0.09)
      ag.yaw.setRadian(tf2::getYaw(ps.pose.orientation));
    else
      ag.yaw = utils::Angle::fromRadian(
          atan2(ag.velocity.getY(), ag.velocity.getX()));
    ag.angularVelocity = people->people[i].velocity.z;
    ag.radius = iface_params_.person_radius_;
    ag.teleoperated = false;

    // The SFM requires a local goal for each agent. We will assume that the
    // goal for people depends on its current velocity
    ag.goals.clear();
    sfm::Goal naiveGoal;
    // No group consideration for the moment
    // naiveGoal.center =
    //    agents_[i + 1].position + naive_goal_time_ * agents_[i + 1].velocity;
    utils::Vector2d v =
        ag.position + iface_params_.naive_goal_time_ * ag.velocity;
    naiveGoal.center.set(v.getX(), v.getY());
    naiveGoal.radius = iface_params_.person_radius_;
    ag.goals.push_back(naiveGoal);
    ag.desiredVelocity = iface_params_.people_velocity_;
    agents.push_back(ag);

    // RCLCPP_INFO(
    //     node_->get_logger(),
    //     "\tsfm::agent--id: %i, x: %.2f, y:%.2f, vx: %.2f, vy: %.2f, vz:
    //     %.2f", ag.id, ag.position.getX(), ag.position.getY(),
    //     ag.velocity.getX(), ag.velocity.getY(), ag.angularVelocity);
  }

  // Fill the obstacles of the agents
  obs_mutex_.lock();
  std::vector<utils::Vector2d> obs_points = obstacles_;
  obs_mutex_.unlock();
  for (unsigned int i = 0; i < agents.size(); i++) {
    agents[i].obstacles1.clear();
    agents[i].obstacles1 = obs_points;
  }

  agents_mutex_.lock();
  agents_.resize(people->people.size() + 1);
  agents_[0].obstacles1 = obs_points;
  for (unsigned int i = 1; i < agents_.size(); i++)
    agents_[i] = agents[i - 1];
  agents_mutex_.unlock();
}

/**
 * @brief  Callback to process the odometry messages with the robot movement.
 * @param odom messages with the obstacles to be processed.
 */
void SFMSensorInterface::odomCb(const nav_msgs::msg::Odometry::SharedPtr odom) {

  if (!running_)
    return;

  RCLCPP_INFO_ONCE(node_->get_logger(), "Odom received");
  odom_received_ = true;
  // last_odom_ = rclcpp::Time(odom->header.stamp);
  // if (odom->header.frame_id != odom_frame_) {
  //   ROS_INFO("Odometry frame is %s, it should be %s",
  //            odom->header.frame_id.c_str(), odom_frame_.c_str());
  //   // return;
  // }

  odom_mutex_.lock();
  base_odom_ = *odom;
  odom_mutex_.unlock();

  agents_mutex_.lock();
  sfm::Agent agent = agents_[0];
  agents_mutex_.unlock();

  agent.position.set(odom->pose.pose.position.x, odom->pose.pose.position.y);
  agent.yaw =
      utils::Angle::fromRadian(tf2::getYaw(odom->pose.pose.orientation));

  agent.linearVelocity =
      std::sqrt(odom->twist.twist.linear.x * odom->twist.twist.linear.x +
                odom->twist.twist.linear.y * odom->twist.twist.linear.y);
  agent.angularVelocity = odom->twist.twist.angular.z;

  // The velocity in the odom messages is in the robot local frame!!!
  geometry_msgs::msg::Vector3 velocity;
  velocity.x = odom->twist.twist.linear.x;
  velocity.y = odom->twist.twist.linear.y;

  // this is throwing an extrapolation in the future error!!!
  // geometry_msgs::msg::Vector3 localV = SFMSensorInterface::transformVector(
  //     velocity, odom->header.stamp, iface_params_.robot_frame_,
  //     iface_params_.controller_frame_);
  // agent.velocity.set(localV.x, localV.y);
  agent.velocity.set(velocity.x, velocity.y);

  // Update agent[0] (the robot) with odom.
  agents_mutex_.lock();
  agents_[0] = agent;
  agents_mutex_.unlock();
}

// copy over the odometry information
void SFMSensorInterface::getOdom(nav_msgs::msg::Odometry &base_odom) {
  // boost::mutex::scoped_lock lock(odom_mutex_);
  odom_mutex_.lock();
  base_odom = base_odom_;
  odom_mutex_.unlock();
}

// void SFMSensorInterface::getRobotVel(
//     geometry_msgs::msg::PoseStamped &robot_vel) {
//   // Set current velocities from odometry
//   geometry_msgs::msg::Twist global_vel;

//   // boost::mutex::scoped_lock lock(odom_mutex_);
//   odom_mutex_.lock();
//   global_vel.linear.x = base_odom_.twist.twist.linear.x;
//   global_vel.linear.y = base_odom_.twist.twist.linear.y;
//   global_vel.angular.z = base_odom_.twist.twist.angular.z;

//   robot_vel.header.frame_id = base_odom_.child_frame_id;
//   odom_mutex_.unlock();

//   robot_vel.pose.position.x = global_vel.linear.x;
//   robot_vel.pose.position.y = global_vel.linear.y;
//   robot_vel.pose.position.z = 0;
//   tf2::Quaternion q;
//   q.setRPY(0, 0, global_vel.angular.z);
//   tf2::convert(q, robot_vel.pose.orientation);
//   robot_vel.header.stamp = base_odom_.header.stamp; // rclcpp::Time();
// }

/**
 * @brief  returns the vector of sfm agents
 * @return agents vector
 */
std::vector<sfm::Agent> SFMSensorInterface::getAgents() {
  iface_params_.get(node_.get(), name_);
  agents_mutex_.lock();
  std::vector<sfm::Agent> agents = agents_;
  // RCLCPP_INFO(node_->get_logger(), "SensorInterface.GetAgents: %i",
  //             (int)agents.size());
  // for (auto a : agents) {
  //   RCLCPP_INFO(node_->get_logger(), "ag %i, x: %.2f, y:%.2f, vx: %.2f",
  //   a.id,
  //               a.position.getX(), a.position.getY(), a.linearVelocity);
  // }
  agents_mutex_.unlock();
  return agents;
}

/**
 * @brief  Tranform a coordinate vector from one frame to another
 * @param vector coordinate vector in the origin frame
 * @param from string with the name of the origin frame
 * @param to string with the name of the target frame
 * @return coordinate vector in the target frame
 */
geometry_msgs::msg::Vector3
SFMSensorInterface::transformVector(geometry_msgs::msg::Vector3 &vector,
                                    builtin_interfaces::msg::Time t,
                                    std::string from, std::string to) {
  geometry_msgs::msg::Vector3 nextVector;

  geometry_msgs::msg::Vector3Stamped v;
  geometry_msgs::msg::Vector3Stamped nv;

  v.header.frame_id = from;
  v.header.stamp = t; // node_->get_clock()->now();
  v.vector.x = vector.x;
  v.vector.y = vector.y;
  v.vector.z = 0.0;

  try {
    nv = tf_buffer_->transform(v, to);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(
        node_->get_logger(),
        "TransformVector. No transform from %s frame to %s frame. Ex: %s",
        from.c_str(), to.c_str(), ex.what());
  }

  nextVector.x = nv.vector.x;
  nextVector.y = nv.vector.y;
  nextVector.z = 0.0;

  return nextVector;
}

} // namespace social_force_window_planner
