# Install script for directory: /home/unitree/projects/fetch/src/fetch_b1_bridge

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/unitree/projects/fetch/install")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/unitree/projects/fetch/build/fetch_b1_bridge/catkin_generated/installspace/fetch_b1_bridge.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetch_b1_bridge/cmake" TYPE FILE FILES
    "/home/unitree/projects/fetch/build/fetch_b1_bridge/catkin_generated/installspace/fetch_b1_bridgeConfig.cmake"
    "/home/unitree/projects/fetch/build/fetch_b1_bridge/catkin_generated/installspace/fetch_b1_bridgeConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetch_b1_bridge" TYPE FILE FILES "/home/unitree/projects/fetch/src/fetch_b1_bridge/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_bridge_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_bridge_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_bridge_node"
         RPATH "/home/unitree/UnitreeSLAM/sdk/unitree_legged_sdk-3.8.3/lib/cpp/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge" TYPE EXECUTABLE FILES "/home/unitree/projects/fetch/devel/lib/fetch_b1_bridge/b1_bridge_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_bridge_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_bridge_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_bridge_node"
         OLD_RPATH "/home/unitree/UnitreeSLAM/sdk/unitree_legged_sdk-3.8.3/lib/cpp/arm64:/opt/ros/melodic/lib:"
         NEW_RPATH "/home/unitree/UnitreeSLAM/sdk/unitree_legged_sdk-3.8.3/lib/cpp/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_bridge_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetch_b1_bridge" TYPE DIRECTORY FILES "/home/unitree/projects/fetch/src/fetch_b1_bridge/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_stand_down" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_stand_down")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_stand_down"
         RPATH "/home/unitree/UnitreeSLAM/sdk/unitree_legged_sdk-3.8.3/lib/cpp/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge" TYPE EXECUTABLE FILES "/home/unitree/projects/fetch/devel/lib/fetch_b1_bridge/b1_stand_down")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_stand_down" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_stand_down")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_stand_down"
         OLD_RPATH "/home/unitree/UnitreeSLAM/sdk/unitree_legged_sdk-3.8.3/lib/cpp/arm64:"
         NEW_RPATH "/home/unitree/UnitreeSLAM/sdk/unitree_legged_sdk-3.8.3/lib/cpp/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_stand_down")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_stand_up" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_stand_up")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_stand_up"
         RPATH "/home/unitree/UnitreeSLAM/sdk/unitree_legged_sdk-3.8.3/lib/cpp/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge" TYPE EXECUTABLE FILES "/home/unitree/projects/fetch/devel/lib/fetch_b1_bridge/b1_stand_up")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_stand_up" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_stand_up")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_stand_up"
         OLD_RPATH "/home/unitree/UnitreeSLAM/sdk/unitree_legged_sdk-3.8.3/lib/cpp/arm64:"
         NEW_RPATH "/home/unitree/UnitreeSLAM/sdk/unitree_legged_sdk-3.8.3/lib/cpp/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fetch_b1_bridge/b1_stand_up")
    endif()
  endif()
endif()

