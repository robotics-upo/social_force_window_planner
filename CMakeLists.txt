cmake_minimum_required(VERSION 3.5)
project(social_force_window_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

#set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)


nav2_package()


include_directories(
    /usr/local/include  #to find lightsfm, just in case
    include
)

set(dependencies
  nav2_core
  tf2_ros
  angles
  rclcpp
  geometry_msgs
  people_msgs
  nav2_costmap_2d
  pluginlib
  nav2_msgs
  nav2_util
  nav2_core
  tf2
  tf2_geometry_msgs
  message_filters
)

# The lines below will deal with differences between ROS distributions:
if(TARGET tf2_geometry_msgs::tf2_geometry_msgs)
  get_target_property(_include_dirs tf2_geometry_msgs::tf2_geometry_msgs INTERFACE_INCLUDE_DIRECTORIES)
else()
  set(_include_dirs ${tf2_geometry_msgs_INCLUDE_DIRS})
endif()

find_file(TF2_CPP_HEADERS
  NAMES tf2_geometry_msgs.hpp
  PATHS ${_include_dirs}
  NO_CACHE
  PATH_SUFFIXES tf2_geometry_msgs
)



set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
    src/costmap_model.cpp
    #src/goal_functions.cpp
    #src/odometry_helper_ros.cpp
    src/sensor_interface.cpp
    src/sfw_planner_node.cpp
    src/sfw_planner.cpp
    src/trajectory.cpp
)

# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(${library_name}
  ${dependencies}
)
if(EXISTS ${TF2_CPP_HEADERS})
  target_compile_definitions(${library_name} PUBLIC -DTF2_CPP_HEADERS)
endif()



install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
DESTINATION include)


ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_export_definitions("PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_core sfw_plugin.xml)

ament_package()


