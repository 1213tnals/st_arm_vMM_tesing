cmake_minimum_required(VERSION 3.0.2)
project(st_arm_libs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
)
generate_messages(DEPENDENCIES std_msgs)

find_package(gazebo REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (RBDL REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${PCL_LIBRARY_DIRS} ${EIGEN3_INCLUDE_DIR} ${RBDL_INCLUDE_DIR})

add_library(st_arm_plugin src/st_arm_plugin.cc 
                          # src/rbdl/Constraints.cc 
                          # src/rbdl/Dynamics.cc 
                          # src/rbdl/Joint.cc 
                          # src/rbdl/Kinematics.cc 
                          # src/rbdl/Logging.cc 
                          # src/rbdl/Model.cc 
                          # src/rbdl/rbdl_mathutils.cc 
                          # src/rbdl/rbdl_utils.cc 
                          # src/rbdl/rbdl_version.cc
                          )
target_link_libraries(st_arm_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${RBDL_LIBRARY}) # ${EIGEN3_INCLUDE_DIR} ${RBDL_LIBRARIES}

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME} rbdl
 DEPENDS gazebo_ros roscpp rospy std_msgs
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")  
