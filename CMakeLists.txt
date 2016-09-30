cmake_minimum_required(VERSION 2.8.3)
project(time_calibration)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  ublox_node
  gnss_node
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs rospy sensor_msgs cv_bridge ublox_node gnss_node
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} 
  include
)

## Declare a C++ library
# add_library(time_calibration
#   src/${PROJECT_NAME}/time_calibration.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(time_calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
 add_executable(time_calibration src/time_calibration.cpp src/utilities.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(time_calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(time_calibration
   ${catkin_LIBRARIES}
 )


