cmake_minimum_required(VERSION 2.8.3)
project(frontier_exploration)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  move_base_msgs
  geometry_msgs
  roscpp
  std_msgs
  std_srvs
  tf
  actionlib
  actionlib_msgs
  visualization_msgs
  message_generation
        nav_msgs
        uav_costmap_obstacle_layer
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

add_service_files(
  FILES
  UpdateBoundaryPolygon.srv
  GetNextFrontier.srv
  BlacklistPoint.srv
  GetNextAirGroundFrontier.srv
)

add_action_files(
  FILES
  ExploreTask.action
)

generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs  actionlib_msgs move_base_msgs visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES explore_costmap uav_costmap_obstacle_layer
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    roscpp
    tf
    actionlib
    uav_costmap_obstacle_layer
  DEPENDS
    PCL
    Boost
)
#add_definitions(-DThread1)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})

add_library(explore_costmap plugins/bounded_explore_layer.cpp src/frontier_search.cpp)
target_link_libraries(explore_costmap ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(explore_costmap ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_library(air_ground_sim_explore_costmap plugins/air_ground_sim_explore_layer.cpp src/frontier_search.cpp plugins/path_search.cpp plugins/trajectory_generator_waypoint.cpp)
target_link_libraries(air_ground_sim_explore_costmap ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_INCLUDE_DIR})
add_dependencies(air_ground_sim_explore_costmap ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_library(astar_planner src/astar_costmap.cpp)
target_link_libraries(astar_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(astar_planner ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_library(path_search plugins/path_search.cpp)
target_link_libraries(path_search ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIR} ${Boost_LIBRARIES})
add_dependencies(path_search ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_library(trajectory_generator plugins/trajectory_generator_waypoint.cpp)
target_link_libraries(trajectory_generator ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIR} ${Boost_LIBRARIES})
add_dependencies(trajectory_generator ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(explore_server src/explore_server.cpp)
target_link_libraries(explore_server ${catkin_LIBRARIES})
add_dependencies(explore_server ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(explore_client src/explore_client.cpp)
target_link_libraries(explore_client ${catkin_LIBRARIES})
add_dependencies(explore_client ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(explore_starter src/explore_starter.cpp)
target_link_libraries(explore_starter ${catkin_LIBRARIES})
add_dependencies(explore_starter ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(make_plan_test src/make_plan_test.cpp)
target_link_libraries(make_plan_test ${catkin_LIBRARIES})
add_dependencies(make_plan_test ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(path_search_test src/path_search_test.cpp)
target_link_libraries(path_search_test ${catkin_LIBRARIES})
add_dependencies(path_search_test ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  # Find package test dependencies
  find_package(rostest REQUIRED)
  find_package(gtest)


  # Add the test folder to the include directories
  include_directories(src/test)

  include_directories(${GTEST_INCLUDE_DIRS})
  link_directories(${GTEST_LIBRARY_DIRS})

  catkin_add_gtest(frontier_search_tests src/test/frontier_search_tests.cpp)
  target_link_libraries(frontier_search_tests explore_costmap)

  catkin_add_gtest(geometry_tools_tests src/test/geometry_tools_tests.cpp)
  target_link_libraries(geometry_tools_tests explore_costmap)

  catkin_add_gtest(costmap_tools_tests src/test/costmap_tools_tests.cpp)
  target_link_libraries(costmap_tools_tests explore_costmap)

endif()

install(TARGETS explore_server explore_client
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS explore_costmap
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
