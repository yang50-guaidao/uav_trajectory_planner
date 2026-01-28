cmake_minimum_required(VERSION 3.0.2)
project(trajectory_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  pcl_ros
  tf2_ros
  decomp_ros_msgs
  decomp_ros_utils
  decomp_util
  message_generation
  kr_mav_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OsqpEigen REQUIRED HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../../install/lib/cmake/OsqpEigen)

# Generate messages
add_message_files(
  FILES
  BsplineTrajectory.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_planner
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs pcl_ros decomp_ros_msgs message_runtime kr_mav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../decomp_util/include
)

# GridMap library
add_library(grid_map
  src/plan_env/grid_map.cpp
)
target_link_libraries(grid_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Kinodynamic A* library
add_library(kinodynamic_astar
  src/path_searching/kinodynamic_astar.cpp
)
target_link_libraries(kinodynamic_astar
  ${catkin_LIBRARIES}
  grid_map
)

# Corridor generator library
add_library(corridor_generator
  src/corridor/corridor_generator.cpp
)
target_link_libraries(corridor_generator
  ${catkin_LIBRARIES}
)

# B-spline library
add_library(bspline
  src/bspline/uniform_bspline.cpp
  src/bspline/bspline_optimizer.cpp
)
target_link_libraries(bspline
  ${catkin_LIBRARIES}
  OsqpEigen::OsqpEigen
)

# Planner manager library
add_library(planner_manager
  src/plan_manage/planner_manager.cpp
)
target_link_libraries(planner_manager
  ${catkin_LIBRARIES}
  grid_map
  kinodynamic_astar
  corridor_generator
  bspline
)

# Main planner node
add_executable(planner_node
  src/planner_node.cpp
)
target_link_libraries(planner_node
  ${catkin_LIBRARIES}
  planner_manager
)
add_dependencies(planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Trajectory server node
add_executable(traj_server_node
  src/traj_server/traj_server_node.cpp
)
target_link_libraries(traj_server_node
  ${catkin_LIBRARIES}
)
add_dependencies(traj_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Install launch and config files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
