cmake_minimum_required(VERSION 3.1.2)
project(gazebo_rad_source)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mrs_msgs 
  mrs_lib
  gazebo_ros
  gazebo_plugins
  gazebo_rad_msgs
  )

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS gazebo_ros gazebo_rad_msgs
  CATKIN_DEPENDS roscpp std_msgs mrs_msgs mrs_lib
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  )

add_library(
  gazebo_rad_source SHARED
  src/gazebo_rad_source.cpp
  )

target_link_libraries(
  gazebo_rad_source
  ${catkin_LIBRARIES}
  )

add_dependencies(
  gazebo_rad_source
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

install(DIRECTORY gazebo_files/models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
