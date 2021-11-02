# social_force_window_planner 
A local controller based on Dinamic Window Approach (DWA) and Social Force Model (SFM) has been developed to command a differential robot in a socially-compliant way.

This planner projects a set of possible trajectories in a small lookahead time (*sim_time* parameter). Then, the SFM is employed as a predictor of the future state of the surrounding people along the trajectories.

Besides the basic scoring function to follow the global path, a new social cost has been added. It is based on the concept of the "**social work**" performed by the robot (<img src="https://render.githubusercontent.com/render/math?math=W_{r}">), and the social work provoked by the robot in the surrounding pedestrians (<img src="https://render.githubusercontent.com/render/math?math=W_{p}">)

<img src="https://render.githubusercontent.com/render/math?math=W_{social} = W_{r} %2B \sum W_{p_{i}}">

With:

- <img src="https://render.githubusercontent.com/render/math?math=W_{r}+"> The summatory of the modulus of the robot social force and the robot obstacle force along the trajectory. According to the SFM.
- <img src="https://render.githubusercontent.com/render/math?math=W_{p}+"> The summatory of the modulus of the social forces provoked by the robot for each close pedestrian along the trajectory. According to the SFM.

This local planner has been programmed as a BaseLocalPlanner plugin under ROS distro Noetic. So it can be used in the ROS *move_base* architecture. The possible collisions are checked based on the ROS local costmap and the projected people positions.

## Parameters

* **Robot Configuration Parameters**
	- *max_trans_acc*. Maximum acceleration in translation (m/s^2).
  	- *max_rot_acc*. Maximum acceleration in rotation (rad/s^2).
  	- *max_trans_vel*. Maximum linear velocity (m/s).
  	- *min_trans_vel*. Minimum linear velocity (m/s).
  	- *max_rot_vel*. Maximum angular velocity (rad/s).
  	- *min_rot_vel*. Minimum angular velocity (rad/s).
  	- *min_in_place_rot_vel*. Angular velocity of rotations in place (rad/s).

* **Goal Tolerance Parameters**
	- *yaw_goal_tolerance*. Tolerance in angular distance (rad) to consider that the goal has been reached.
	- *xy_goal_tolerance*. Tolerance in euclidean distance (m) to consider that the goal has been reached.
	- *wp_tolerance*. Distance (m) from the robot to look for the point of the global plan to follow.
  
* **Forward Simulation Parameters**
	- *sim_time*. Time (seconds) to expand the robot movement and check for collisions. (default: 0.5).
	- *sim_granularity*. Resolution in meters to split the expanded trayectory and check for collisions (Default: 0.025).

* **Sensor Interface Parameters**
  The sensor interface is in charge of taking the sensory input and update the information of the surrounding social agents in the scene. 

	- *laser_topic*. Topic in which the laser range finder is being published [sensor_msgs/LaserScan].
	- *people_topic*. Topic in which the people detected are being published [people_msgs/People].
	- *dyn_obs_topic*. Topic in which the dynamic obstacles detected are being published [dynamic_obstacle_detector/DynamicObstacles].
	- *odom_topic*. Odometry topic [nav_msgs/Odometry].
	- *max_obstacle_dist*. Maximum distance (m) in which the obstacles are considered for the social force model.
	- *naive_goal_time*. Lookahead time (secs) to predict an approximate goal for the pedestrians.
	- *people_velocity*. Average velocity of the pedestrians (m/s). 

* **Social Force Model Parameters**
  The weights of the SFM forces for people trajectory computation.
	- *sfm_goal_weight*. Weight of the attraction force to the goal.
	- *sfm_obstacle_weight*. Weight of the respulsive force of the obstacles.
	- *sfm_people_weight*. Weight of the respulsive force of the pedestrians.

* **Trajectory-scoring Function Parameters** 
Cost function for trajectory scoring:

	<img src="https://render.githubusercontent.com/render/math?math=C_{traj} = C_{s} * \omega_{s} %2B C_{cm} * \omega_{cm} %2B C_{a} * \omega_{a} %2B C_{d} * \omega_{d}">

	With:

  - *social_weight <img src="https://render.githubusercontent.com/render/math?math=\omega_{s}">*. The weight given to the "social-work" term. (<img src="https://render.githubusercontent.com/render/math?math=C_{s} = W_{social}">)
  - *costmap_weight <img src="https://render.githubusercontent.com/render/math?math=\omega_{cm}">*. The weight given to the normalized "non-lethal" costmap value.
  - *angle_weight <img src="https://render.githubusercontent.com/render/math?math=\omega_{a}">*. The weight given to the angle difference between the robot heading and the path heading.
  - *distance_weight <img src="https://render.githubusercontent.com/render/math?math=\omega_{d}">*. The weight given to the distance between the final point of the projected robot trajectory and the current local goal.

## Dependencies

- dynamic_obstacle_detector (https://github.com/robotics-upo/dynamic_obstacle_detector).

- lightsfm (https://github.com/robotics-upo/lightsfm)
  
- ROS move_base (http://wiki.ros.org/move_base)


## TO DO List:

The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.

- [ ] Improve the approximation to a final goal.
- [ ] Larger *sim_time* and *wp_tolerance* gives more "social" freedom to the planner but it presents problems to overcome some corners and narrow passages. We must try to fix this or to find a good balance. 
- [X] Fix the computation of the robot local goal in the path in case the robot moves farther than *wp_tolerance* distance.
