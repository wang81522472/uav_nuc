# Install script for directory: /home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/catkin_generated/installspace/_setup_util.py")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/catkin_generated/installspace/env.sh")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/catkin_generated/installspace/setup.bash")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/catkin_generated/installspace/setup.sh")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/catkin_generated/installspace/setup.zsh")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/catkin_generated/installspace/.rosinstall")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/frontier_exploration/msg" TYPE FILE FILES
    "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/msg/Frontier.msg"
    "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/msg/FrontierCell.msg"
    "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/msg/MultiArrayWithHeader.msg"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/frontier_exploration/srv" TYPE FILE FILES
    "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/srv/UpdateBoundaryPolygon.srv"
    "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/srv/GetNextFrontier.srv"
    "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/srv/BlacklistPoint.srv"
    "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/srv/GetNextAirGroundFrontier.srv"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/frontier_exploration/action" TYPE FILE FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/action/ExploreTask.action")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/frontier_exploration/msg" TYPE FILE FILES
    "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/devel/share/frontier_exploration/msg/ExploreTaskAction.msg"
    "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/devel/share/frontier_exploration/msg/ExploreTaskActionGoal.msg"
    "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/devel/share/frontier_exploration/msg/ExploreTaskActionResult.msg"
    "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/devel/share/frontier_exploration/msg/ExploreTaskActionFeedback.msg"
    "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/devel/share/frontier_exploration/msg/ExploreTaskGoal.msg"
    "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/devel/share/frontier_exploration/msg/ExploreTaskResult.msg"
    "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/devel/share/frontier_exploration/msg/ExploreTaskFeedback.msg"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/frontier_exploration/cmake" TYPE FILE FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/catkin_generated/installspace/frontier_exploration-msg-paths.cmake")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/devel/include/frontier_exploration")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/devel/share/roseus/ros/frontier_exploration")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/devel/share/common-lisp/ros/frontier_exploration")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/devel/share/gennodejs/ros/frontier_exploration")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/devel/lib/python2.7/dist-packages/frontier_exploration")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/devel/lib/python2.7/dist-packages/frontier_exploration")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/catkin_generated/installspace/frontier_exploration.pc")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/frontier_exploration/cmake" TYPE FILE FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/catkin_generated/installspace/frontier_exploration-msg-extras.cmake")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/frontier_exploration/cmake" TYPE FILE FILES
    "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/catkin_generated/installspace/frontier_explorationConfig.cmake"
    "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/catkin_generated/installspace/frontier_explorationConfig-version.cmake"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/frontier_exploration" TYPE FILE FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/package.xml")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/frontier_exploration/explore_server" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/frontier_exploration/explore_server")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/frontier_exploration/explore_server"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/frontier_exploration" TYPE EXECUTABLE FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/devel/lib/frontier_exploration/explore_server")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/frontier_exploration/explore_server" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/frontier_exploration/explore_server")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/frontier_exploration/explore_server"
         OLD_RPATH "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/devel/lib:/opt/ros/kinetic/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/frontier_exploration/explore_server")
    endif()
  endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/frontier_exploration/explore_client" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/frontier_exploration/explore_client")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/frontier_exploration/explore_client"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/frontier_exploration" TYPE EXECUTABLE FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/devel/lib/frontier_exploration/explore_client")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/frontier_exploration/explore_client" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/frontier_exploration/explore_client")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/frontier_exploration/explore_client"
         OLD_RPATH "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/devel/lib:/opt/ros/kinetic/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/frontier_exploration/explore_client")
    endif()
  endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libexplore_costmap.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libexplore_costmap.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libexplore_costmap.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/devel/lib/libexplore_costmap.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libexplore_costmap.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libexplore_costmap.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libexplore_costmap.so"
         OLD_RPATH "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/devel/lib:/opt/ros/kinetic/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libexplore_costmap.so")
    endif()
  endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/frontier_exploration" TYPE DIRECTORY FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/include/frontier_exploration/" REGEX "/\\.svn$" EXCLUDE)
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/frontier_exploration/launch" TYPE DIRECTORY FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/launch/" REGEX "/\\.svn$" EXCLUDE)
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/frontier_exploration" TYPE FILE FILES "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/costmap_plugins.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/chengdaqian/UAV_UGV_Exploration/explor_catkin_ws/src/frontier_exploration/cmake-build-debug/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
