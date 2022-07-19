################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(rmd_motor_control)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Eigen COMPONENTS Core REQUIRED)
find_package(Eigen COMPONENTS Dense REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jinu_manipulator_controller
  CATKIN_DEPENDS 
    cmake_modules 
    geometry_msgs 
    roscpp 
    sensor_msgs 
    std_msgs 
    trajectory_msgs
  DEPENDS 
    Boost
    Eigen
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

#add_library(rmd_motor src/rmd_motor.cpp)

add_executable(
  rmd_motor_controller 
  src/main.cpp 
  src/rmd_utils.cpp 
  src/rmd_can.cpp
  src/spi2can.cpp
  src/rmd_motor.cpp
  src/motor_controller.cpp
  src/dynamics.cpp)
add_dependencies(rmd_motor_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rmd_motor_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} Qt5::Core)

################################################################################
# Install
################################################################################
install(TARGETS rmd_motor_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/update_spi_rules
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)