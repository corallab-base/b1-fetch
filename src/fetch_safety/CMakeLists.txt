cmake_minimum_required(VERSION 3.0.2)
project(fetch_safety)

add_compile_options(-std=c++14 -O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2_ros
  tf2_eigen
  message_generation
)
find_package(Eigen3 REQUIRED)

add_message_files(FILES ZoneCounts.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(safety_filter_node src/safety_filter_node.cpp)
target_link_libraries(safety_filter_node ${catkin_LIBRARIES})
add_dependencies(safety_filter_node ${PROJECT_NAME}_generate_messages_cpp)

# Runs on each camera board: cloud -> per-zone counts for the filter
add_executable(zone_checker_node src/zone_checker_node.cpp)
target_link_libraries(zone_checker_node ${catkin_LIBRARIES})
add_dependencies(zone_checker_node ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS safety_filter_node zone_checker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/test_safety_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
