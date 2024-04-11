cmake_minimum_required(VERSION 3.16)
project(kas_utils)

find_package(catkin REQUIRED)

find_package(kas_utils_cpp REQUIRED)
set(ENV{kas_utils_cpp_LIBRARIES_DIR} ${kas_utils_cpp_LIBRARIES_DIR})

catkin_python_setup()

catkin_package()

# Install package.xml for catkin (does not work because of catkin_python_setup() for some reason)
install(FILES package.xml DESTINATION share/kas_utils/)

