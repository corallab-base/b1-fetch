cmake_minimum_required(VERSION 3.0.2)
project(fetch_nav)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/teleop_keys.py scripts/ir_filter.py scripts/goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
