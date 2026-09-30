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
)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(safety_filter_node src/safety_filter_node.cpp)
target_link_libraries(safety_filter_node ${catkin_LIBRARIES})

install(TARGETS safety_filter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/test_safety_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
