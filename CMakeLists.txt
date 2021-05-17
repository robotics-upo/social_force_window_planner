cmake_minimum_required(VERSION 2.8.3)
project(social_force_window_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            dynamic_reconfigure
            nav_core
            nav_msgs
            costmap_2d
            geometry_msgs
            pluginlib
            tf2
            tf2_geometry_msgs
            tf2_ros
            angles
            dynamic_obstacle_detector
        )

find_package(Eigen3 REQUIRED)

#PAL
include_directories(include)
include_directories(SYSTEM
    /usr/local/include  #to find lightsfm
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})



# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/SFWPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
        sfw_planner
        sfw_planner_ros
    CATKIN_DEPENDS
        roscpp
        dynamic_reconfigure
        geometry_msgs
        pluginlib
        costmap_2d
        nav_core
        nav_msgs
        angles
        tf2
        tf2_ros
        lightsfm
)

#uncomment for profiling
#set(ROS_COMPILE_FLAGS "-g -pg" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g -pg" ${ROS_LINK_FLAGS})
#set(ROS_COMPILE_FLAGS "-g" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g" ${ROS_LINK_FLAGS})

add_library(sfw_planner
	src/goal_functions.cpp
	src/odometry_helper_ros.cpp
	src/costmap_model.cpp
	src/trajectory.cpp
 )
add_dependencies(sfw_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} sfw_planner_gencfg)
target_link_libraries(sfw_planner
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
    )

add_library(sfw_planner_ros
    src/sensor_interface.cpp
	src/sfw_planner.cpp
	src/sfw_planner_ros.cpp)

target_link_libraries(sfw_planner_ros
     sfw_planner)


install(TARGETS
            sfw_planner
            sfw_planner_ros
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES sfw_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)


