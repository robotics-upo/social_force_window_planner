/**
 * Class to adapt the sensory input and detections to be used by the
 * sfm_local_controller
 *
 * Author: Noé Pérez-Higueras
 * Service Robotics Lab, Pablo de Olavide University 2021
 *
 * Software License Agreement (BSD License)
 *
 */

#include <social_force_window_planner/sensor_interface.h>

namespace sfw_planner {

/**
 * @brief  Default constructor
 * @param n pointer to a ros node handle to publish to topics
 * @param tfBuffer Pointer to tf2 buffer
 * @param robot_max_lin_speed the maximum robot linear speed [m/s]
 * @param robot_radius the radius of the robot circunference [m]
 * @param person_radius the approximated radius of the people body
 * @param robot_frame the coordinate frame of the robot base
 * @param odom_frame the coordinate frame of the robot odometry
 **/
SFMSensorInterface::SFMSensorInterface(ros::NodeHandle *n,
                                       tf2_ros::Buffer *tfBuffer,
                                       float robot_max_lin_speed,
                                       float robot_radius, float person_radius,
                                       std::string robot_frame,
                                       std::string odom_frame)
    : nh_(n), n_(), tf_buffer_(tfBuffer), person_radius_(person_radius),
      robot_frame_(robot_frame), odom_frame_(odom_frame) {

  nh_->param("max_obstacle_dist", max_obstacle_dist_, float(3.0));
  nh_->param("naive_goal_time", naive_goal_time_, float(2.0));
  nh_->param("people_velocity", people_velocity_, float(1.0));

  laser_received_ = false;
  last_laser_ = ros::Time::now();

  std::string laser_topic;
  nh_->param("laser_topic", laser_topic, std::string("scan"));
  std::string people_topic;
  nh_->param("people_topic", people_topic, std::string("people"));
  std::string obs_topic;
  nh_->param("dyn_obs_topic", obs_topic, std::string("obstacles"));
  std::string odom_topic;
  nh_->param("odom_topic", odom_topic, std::string("odom"));

  std::cout << std::endl
            << "SFM SENSOR INTERFACE:" << std::endl
            << "laser_topic: " << laser_topic << std::endl
            << "people_topic: " << people_topic << std::endl
            << "dyn_obs_topic: " << obs_topic << std::endl
            << "odom_topic: " << odom_topic << std::endl
            << "max_obstacle_dist: " << max_obstacle_dist_ << std::endl
            << "naive_goal_time: " << naive_goal_time_ << std::endl
            << "people_velocity: " << people_velocity_ << std::endl
            << std::endl;

  // Initialize SFM. Just one agent (the robot)
  agents_.resize(1);
  agents_[0].desiredVelocity = robot_max_lin_speed;
  agents_[0].radius = robot_radius;
  agents_[0].cyclicGoals = false;
  agents_[0].teleoperated = true;
  agents_[0].groupId = -1;

  // ros::NodeHandle n;
  laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>(
      laser_topic.c_str(), 1, &SFMSensorInterface::laserCb, this);

  people_sub_ = n_.subscribe<people_msgs::People>(
      people_topic.c_str(), 1, &SFMSensorInterface::peopleCb, this);

  dyn_obs_sub_ = n_.subscribe<dynamic_obstacle_detector::DynamicObstacles>(
      obs_topic.c_str(), 1, &SFMSensorInterface::dynamicObsCb, this);

  odom_sub_ = n_.subscribe<nav_msgs::Odometry>(
      odom_topic.c_str(), 1, &SFMSensorInterface::odomCb, this);

  points_pub_ = n_.advertise<visualization_msgs::Marker>(
      "/sfm/markers/obstacle_points", 0);

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
    const sensor_msgs::LaserScan::ConstPtr &laser) {

  laser_received_ = true;
  last_laser_ = ros::Time::now();

  ROS_INFO_ONCE("laser received");

  std::vector<utils::Vector2d> points;
  float angle = laser->angle_min;

  for (unsigned int i = 0; i < laser->ranges.size(); i++) {

    if (laser->ranges[i] < max_obstacle_dist_) {

      utils::Vector2d point(laser->ranges[i] * cos(angle),
                            laser->ranges[i] * sin(angle));
      points.push_back(point);
    }
    angle += laser->angle_increment;
    // alpha += angle_inc;
  }

  if (points.empty()) {
    obstacles_ = points;
    return;
  }

  // Transform points to
  // odom_frame_ if necessary
  if (laser->header.frame_id != odom_frame_) {
    geometry_msgs::PointStamped out;
    for (unsigned int i = 0; i < points.size(); i++) {
      geometry_msgs::PointStamped in;
      in.header.frame_id = laser->header.frame_id;
      in.header.stamp = ros::Time(0);
      in.point.x = points[i].getX();
      in.point.y = points[i].getY();
      in.point.z = 0.0;
      try {
        geometry_msgs::PointStamped out =
            tf_buffer_->transform(in, odom_frame_);
        points[i].setX(out.point.x);
        points[i].setY(out.point.y);

      } catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT transform "
                 "laser point to %s: "
                 "%s",
                 robot_frame_.c_str(), ex.what());
        return;
      }
    }
  }

  // Now check if the points
  // belong to a person or an
  // dynamic obstacle

  // transform people positions to
  // robot frame
  // people_mutex_.lock();
  people_msgs::People people = people_;
  // people_mutex_.unlock();

  std::vector<geometry_msgs::Point> people_points;
  if (people.header.frame_id != odom_frame_) {
    geometry_msgs::PointStamped person_point;
    person_point.header = people.header;

    for (auto person : people.people) {
      person_point.point = person.position;
      person_point.header.stamp = ros::Time(0);
      try {
        geometry_msgs::PointStamped p_point =
            tf_buffer_->transform(person_point, odom_frame_);
        people_points.push_back(p_point.point);
      } catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT transform "
                 "person point to %s: "
                 "%s",
                 odom_frame_.c_str(), ex.what());
        return;
      }
    }
  } else {
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
        if (d <= person_radius_) {
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
  if (points.empty()) {
    obstacles_ = points;
    return;
  }

  // transform dynamic obstacles
  // positions to robot frame
  // obs_mutex_.lock();
  dynamic_obstacle_detector::DynamicObstacles obstacles = dyn_obs_;
  // obs_mutex_.unlock();

  std::vector<geometry_msgs::Point> ob_points;
  if (obstacles.header.frame_id != odom_frame_) {
    geometry_msgs::PointStamped ob_point;
    ob_point.header = obstacles.header;
    // ob_point.stamp = ros::Time();
    for (auto obstacle : obstacles.obstacles) {
      ob_point.point = obstacle.position;
      ob_point.header.stamp = ros::Time(0);
      try {
        geometry_msgs::PointStamped o_point =
            tf_buffer_->transform(ob_point, odom_frame_);
        ob_points.push_back(o_point.point);
      } catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT transform "
                 "obstacle point to %s: "
                 "%s",
                 odom_frame_.c_str(), ex.what());
        return;
      }
    }
  }
  // Remove the points in the
  // object radius (approximated
  // by person radius because we
  // do not know the radius)
  if (!ob_points.empty()) {
    std::vector<utils::Vector2d> points_aux;
    for (utils::Vector2d p : points) {
      bool remove = false;
      for (auto ob : ob_points) {
        float dx = p.getX() - ob.x;
        float dy = p.getY() - ob.y;
        float d = std::hypotf(dx, dy);
        if (d <= person_radius_) {
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

  obstacles_ = points;
  publish_obstacle_points(points);
}

/**
 * @brief  publish the transformed points in RViz
 * @param points vector with the coordinates of the points
 * @return none
 */
void SFMSensorInterface::publish_obstacle_points(
    const std::vector<utils::Vector2d> &points) {

  visualization_msgs::Marker m;
  m.header.frame_id = odom_frame_; // robot_frame_
  m.header.stamp = ros::Time::now();
  m.ns = "sfm_obstacle_points";
  m.type = visualization_msgs::Marker::POINTS;
  m.action = visualization_msgs::Marker::ADD;
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
  m.lifetime = ros::Duration(0.3);
  // printf("Published Obstacles: ");
  for (utils::Vector2d p : points) {
    geometry_msgs::Point pt;
    pt.x = p.getX();
    pt.y = p.getY();
    // printf("x: %.2f, y: %.2f -", pt.x, pt.y);
    pt.z = 0.2;
    m.points.push_back(pt);
  }
  // printf("\n");
  points_pub_.publish(m);
}

/**
 * @brief  Callback to process the moving obstacles detected in the robot
 * vecinity.
 * @param obs messages with the obstacles to be processed.
 */
void SFMSensorInterface::dynamicObsCb(
    const dynamic_obstacle_detector::DynamicObstacles::ConstPtr &obs) {
  ROS_INFO_ONCE("Dynamic obs received");

  // obs_mutex_.lock();
  dyn_obs_ = *obs;
  // obs_mutex_.unlock();

  std::vector<sfm::Agent> agents;

  // check if people are not in odom frame
  geometry_msgs::PointStamped ps;
  for (unsigned i = 0; i < obs->obstacles.size(); i++) {
    sfm::Agent ag;
    ps.header.frame_id = obs->header.frame_id;
    ps.header.stamp = ros::Time(0);
    ps.point = obs->obstacles[i].position;
    if (obs->header.frame_id != odom_frame_) {
      geometry_msgs::PointStamped p;
      try {
        p = tf_buffer_->transform(ps, odom_frame_);
        ps = p;
      } catch (tf2::TransformException &ex) {
        ROS_WARN("No transform %s", ex.what());
      }
    }
    ag.position.set(ps.point.x, ps.point.y);

    geometry_msgs::Vector3 velocity;
    velocity.x = obs->obstacles[i].velocity.x;
    velocity.y = obs->obstacles[i].velocity.y;

    geometry_msgs::Vector3 localV = SFMSensorInterface::transformVector(
        velocity, obs->header.frame_id, odom_frame_);

    ag.yaw = utils::Angle::fromRadian(atan2(localV.y, localV.x));
    ag.velocity.set(localV.x, localV.y);
    ag.linearVelocity = ag.velocity.norm();
    ag.radius = person_radius_;
    ag.teleoperated = false;
    // if (fabs(people->people[i].vel) < 0.05) {
    //	agents[i+1].velocity.set(0,0);
    //}

    // The SFM requires a local goal for each agent. We will assume that the
    // goal for people depends on its current velocity
    ag.goals.clear();
    sfm::Goal naiveGoal;
    // No group consideration for the moment
    utils::Vector2d v = ag.position + naive_goal_time_ * ag.velocity;
    naiveGoal.center.set(v.getX(), v.getY());
    naiveGoal.radius = person_radius_;
    ag.goals.push_back(naiveGoal);
    ag.desiredVelocity = people_velocity_;
    ag.groupId = -1;
    agents.push_back(ag);
  }

  // Fill the obstacles of the agents
  std::vector<utils::Vector2d> obs_points = obstacles_;
  for (unsigned int i = 0; i < agents.size(); i++) {
    agents[i].obstacles1.clear();
    agents[i].obstacles1 = obs_points;
  }

  agents_mutex_.lock();
  agents_.resize(obs->obstacles.size() + 1);
  agents_[0].obstacles1 = obs_points;
  for (unsigned int i = 1; i < agents_.size(); i++)
    agents_[i] = agents[i - 1];
  agents_mutex_.unlock();
}

/**
 * @brief  Callback to process the people detected in the robot vecinity.
 * @param people message with the people to be processed
 */
void SFMSensorInterface::peopleCb(const people_msgs::People::ConstPtr &people) {
  ROS_INFO_ONCE("People received");

  // people_mutex_.lock();
  people_ = *people;
  // people_mutex_.unlock();

  std::vector<sfm::Agent> agents;

  // check if people are not in odom frame
  geometry_msgs::PointStamped ps;
  for (unsigned i = 0; i < people->people.size(); i++) {
    sfm::Agent ag;
    ps.header.frame_id = people->header.frame_id;
    ps.header.stamp = ros::Time(0);
    ps.point = people->people[i].position;
    if (people->header.frame_id != odom_frame_) {
      geometry_msgs::PointStamped p;
      try {
        p = tf_buffer_->transform(ps, odom_frame_);
        ps = p;
      } catch (tf2::TransformException &ex) {
        ROS_WARN("No transform %s", ex.what());
      }
    }
    ag.position.set(ps.point.x, ps.point.y);

    geometry_msgs::Vector3 velocity;
    velocity.x = people->people[i].velocity.x;
    velocity.y = people->people[i].velocity.y;

    geometry_msgs::Vector3 localV = SFMSensorInterface::transformVector(
        velocity, people->header.frame_id, odom_frame_);

    ag.yaw = utils::Angle::fromRadian(atan2(localV.y, localV.x));
    ag.velocity.set(localV.x, localV.y);
    ag.linearVelocity = agents_[i + 1].velocity.norm();
    ag.radius = person_radius_;
    ag.teleoperated = false;
    // if (fabs(people->people[i].vel) < 0.05) {
    //	agents[i+1].velocity.set(0,0);
    //}

    // The SFM requires a local goal for each agent. We will assume that the
    // goal for people depends on its current velocity
    ag.goals.clear();
    sfm::Goal naiveGoal;
    // No group consideration for the moment
    // naiveGoal.center =
    //    agents_[i + 1].position + naive_goal_time_ * agents_[i + 1].velocity;
    utils::Vector2d v = ag.position + naive_goal_time_ * ag.velocity;
    naiveGoal.center.set(v.getX(), v.getY());
    naiveGoal.radius = person_radius_;
    ag.goals.push_back(naiveGoal);
    ag.desiredVelocity = people_velocity_;
    ag.groupId = -1;
    agents.push_back(ag);
  }

  // Fill the obstacles of the agents
  std::vector<utils::Vector2d> obs_points = obstacles_;
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
void SFMSensorInterface::odomCb(const nav_msgs::Odometry::ConstPtr &odom) {
  ROS_INFO_ONCE("Odom received");

  // if (odom->header.frame_id != odom_frame_) {
  //   ROS_INFO("Odometry frame is %s, it should be %s",
  //            odom->header.frame_id.c_str(), odom_frame_.c_str());
  //   // return;
  // }
  agents_mutex_.lock();
  sfm::Agent agent = agents_[0];
  agents_mutex_.unlock();

  agent.position.set(odom->pose.pose.position.x, odom->pose.pose.position.y);
  agent.yaw = utils::Angle::fromRadian(tf::getYaw(odom->pose.pose.orientation));

  agent.linearVelocity =
      std::sqrt(odom->twist.twist.linear.x * odom->twist.twist.linear.x +
                odom->twist.twist.linear.y * odom->twist.twist.linear.y);
  agent.angularVelocity = odom->twist.twist.angular.z;

  // The velocity in the odom messages is in the robot local frame!!!
  geometry_msgs::Vector3 velocity;
  velocity.x = odom->twist.twist.linear.x;
  velocity.y = odom->twist.twist.linear.y;

  geometry_msgs::Vector3 localV =
      SFMSensorInterface::transformVector(velocity, robot_frame_, odom_frame_);
  agent.velocity.set(localV.x, localV.y);

  // Update agent[0] (the robot) with odom.
  agents_mutex_.lock();
  agents_[0] = agent;
  agents_mutex_.unlock();
}

/**
 * @brief  returns the vector of sfm agents
 * @return agents vector
 */
std::vector<sfm::Agent> SFMSensorInterface::getAgents() {
  agents_mutex_.lock();
  std::vector<sfm::Agent> agents = agents_;
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
geometry_msgs::Vector3
SFMSensorInterface::transformVector(geometry_msgs::Vector3 &vector,
                                    std::string from, std::string to) {
  geometry_msgs::Vector3 nextVector;

  geometry_msgs::Vector3Stamped v;
  geometry_msgs::Vector3Stamped nv;

  v.header.frame_id = from;

  // we'll just use the most recent transform available for our simple example
  v.header.stamp = ros::Time(0);

  // just an arbitrary point in space
  v.vector.x = vector.x;
  v.vector.y = vector.y;
  v.vector.z = 0.0;

  try {
    nv = tf_buffer_->transform(v, to);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("No transform %s", ex.what());
  }

  nextVector.x = nv.vector.x;
  nextVector.y = nv.vector.y;
  nextVector.z = 0.0;

  return nextVector;
}

} // namespace sfw_planner
