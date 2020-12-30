# Install script for directory: /home/piotr/Swarm_robots/ros_workspace/src/ros_argos3/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/piotr/Swarm_robots/ros_workspace/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/argos_bridge/msg" TYPE FILE FILES
    "/home/piotr/Swarm_robots/ros_workspace/src/ros_argos3/src/msg/Puck.msg"
    "/home/piotr/Swarm_robots/ros_workspace/src/ros_argos3/src/msg/PuckList.msg"
    "/home/piotr/Swarm_robots/ros_workspace/src/ros_argos3/src/msg/Proximity.msg"
    "/home/piotr/Swarm_robots/ros_workspace/src/ros_argos3/src/msg/ProximityList.msg"
    "/home/piotr/Swarm_robots/ros_workspace/src/ros_argos3/src/msg/LedsColor.msg"
    "/home/piotr/Swarm_robots/ros_workspace/src/ros_argos3/src/msg/DistScan.msg"
    "/home/piotr/Swarm_robots/ros_workspace/src/ros_argos3/src/msg/DistScanList.msg"
    "/home/piotr/Swarm_robots/ros_workspace/src/ros_argos3/src/msg/MotoGround.msg"
    "/home/piotr/Swarm_robots/ros_workspace/src/ros_argos3/src/msg/MotoGroundList.msg"
    "/home/piotr/Swarm_robots/ros_workspace/src/ros_argos3/src/msg/BaseGround.msg"
    "/home/piotr/Swarm_robots/ros_workspace/src/ros_argos3/src/msg/BaseGroundList.msg"
    "/home/piotr/Swarm_robots/ros_workspace/src/ros_argos3/src/msg/Vector3.msg"
    "/home/piotr/Swarm_robots/ros_workspace/src/ros_argos3/src/msg/Position.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/argos_bridge/cmake" TYPE FILE FILES "/home/piotr/Swarm_robots/ros_workspace/build/ros_argos3/src/catkin_generated/installspace/argos_bridge-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/piotr/Swarm_robots/ros_workspace/devel/include/argos_bridge")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/piotr/Swarm_robots/ros_workspace/devel/share/roseus/ros/argos_bridge")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/piotr/Swarm_robots/ros_workspace/devel/share/common-lisp/ros/argos_bridge")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/piotr/Swarm_robots/ros_workspace/devel/share/gennodejs/ros/argos_bridge")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/piotr/Swarm_robots/ros_workspace/devel/lib/python2.7/dist-packages/argos_bridge")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/piotr/Swarm_robots/ros_workspace/devel/lib/python2.7/dist-packages/argos_bridge")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/piotr/Swarm_robots/ros_workspace/build/ros_argos3/src/catkin_generated/installspace/argos_bridge.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/argos_bridge/cmake" TYPE FILE FILES "/home/piotr/Swarm_robots/ros_workspace/build/ros_argos3/src/catkin_generated/installspace/argos_bridge-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/argos_bridge/cmake" TYPE FILE FILES
    "/home/piotr/Swarm_robots/ros_workspace/build/ros_argos3/src/catkin_generated/installspace/argos_bridgeConfig.cmake"
    "/home/piotr/Swarm_robots/ros_workspace/build/ros_argos3/src/catkin_generated/installspace/argos_bridgeConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/argos_bridge" TYPE FILE FILES "/home/piotr/Swarm_robots/ros_workspace/src/ros_argos3/src/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/piotr/Swarm_robots/ros_workspace/build/ros_argos3/src/plugin/cmake_install.cmake")

endif()

