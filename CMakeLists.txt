cmake_minimum_required(VERSION 2.8.3)
project(multisensor_filters)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  roscpp
  rospy
  std_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY msg
	FILES
	PointWithCovariance.msg
	PointWithCovarianceStamped.msg
)

generate_messages(
	DEPENDENCIES 
	std_msgs 
	geometry_msgs
)



###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

###########
## Build ##
###########

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
