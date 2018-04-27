cmake_minimum_required(VERSION 2.8.3)
project(duet_exploration)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  simple_map_2d
  std_msgs
  visualization_msgs
)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3)

add_message_files(
        FILES
        Frontier.msg
        FrontierCell.msg
        MultiArrayWithHeader.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs std_msgs  actionlib_msgs visualization_msgs
)

link_directories( /home/gf/catkin_ws/src/traj_opt/fm_planer/lib/mosek8 )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES duet_exploration
  CATKIN_DEPENDS geometry_msgs message_generation nav_msgs roscpp simple_map_2d std_msgs visualization_msgs
  DEPENDS PCL Boost
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(harmonic_duet src/harmonic_duet.cpp src/frontier_search.cpp src/trajectory_generator_waypoint.cpp src/bezier_base.cpp src/trajectory_generator_lite.cpp src/path_search.cpp)
target_link_libraries(harmonic_duet ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} mosek64)
add_dependencies(harmonic_duet ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(traj_server src/traj_server.cpp src/bezier_base.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
add_dependencies(traj_server ${catkin_EXPORTED_TARGETS})
