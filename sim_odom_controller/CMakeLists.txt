cmake_minimum_required(VERSION 2.8.3)
project(sim_odom_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
        tf
	frontier_exploration
)

find_package(Eigen3)


catkin_package(
   INCLUDE_DIRS include
  LIBRARIES sim_odom_controller
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf frontier_exploration
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME} src/sim_odom_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(sim_acc_controller src/sim_acc_controller.cpp)
target_link_libraries(sim_acc_controller ${catkin_LIBRARIES} )
