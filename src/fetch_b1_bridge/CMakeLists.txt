cmake_minimum_required(VERSION 3.0.2)
project(fetch_b1_bridge)

add_compile_options(-std=c++14 -O2)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_msgs)
catkin_package()

# Unitree's SDK is used in place (not copied): the same 3.8.3 build that
# a2_ros2udp links against.
set(UNITREE_SDK_DIR "$ENV{HOME}/UnitreeSLAM/sdk/unitree_legged_sdk-3.8.3"
    CACHE PATH "unitree_legged_sdk 3.8.3 root")
set(UNITREE_SDK_LIB "${UNITREE_SDK_DIR}/lib/cpp/arm64/libunitree_legged_sdk.so")
if(NOT EXISTS "${UNITREE_SDK_LIB}")
  message(FATAL_ERROR "unitree_legged_sdk not found at ${UNITREE_SDK_LIB} (set -DUNITREE_SDK_DIR=...)")
endif()

include_directories(${catkin_INCLUDE_DIRS} ${UNITREE_SDK_DIR}/include)

add_executable(b1_bridge_node src/b1_bridge_node.cpp)
target_link_libraries(b1_bridge_node ${catkin_LIBRARIES} ${UNITREE_SDK_LIB} -pthread)
set_target_properties(b1_bridge_node PROPERTIES
  BUILD_RPATH "${UNITREE_SDK_DIR}/lib/cpp/arm64"
  INSTALL_RPATH "${UNITREE_SDK_DIR}/lib/cpp/arm64")

install(TARGETS b1_bridge_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# One-shot stand-down tool (not a ROS node)
add_executable(b1_stand_down src/b1_stand_down.cpp)
target_link_libraries(b1_stand_down ${UNITREE_SDK_LIB} -pthread)
set_target_properties(b1_stand_down PROPERTIES
  BUILD_RPATH "${UNITREE_SDK_DIR}/lib/cpp/arm64"
  INSTALL_RPATH "${UNITREE_SDK_DIR}/lib/cpp/arm64")
install(TARGETS b1_stand_down RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# One-shot stand-up tool (not a ROS node)
add_executable(b1_stand_up src/b1_stand_up.cpp)
target_link_libraries(b1_stand_up ${UNITREE_SDK_LIB} -pthread)
set_target_properties(b1_stand_up PROPERTIES
  BUILD_RPATH "${UNITREE_SDK_DIR}/lib/cpp/arm64"
  INSTALL_RPATH "${UNITREE_SDK_DIR}/lib/cpp/arm64")
install(TARGETS b1_stand_up RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
