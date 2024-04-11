cmake_minimum_required(VERSION 2.8.3)
project(rtabmap_ros_msgs)

set(CMAKE_CXX_STANDARD 17)

# Policy CMP0043 introduced in cmake version 3.0 IGNORES the use of COMPILE_DEFINITIONS suffixed variables, e.g. COMPILE_DEFINITIONS_DEBUG
# Set to OLD behavior until minimum cmake version >= 2.8.10 (version that COMPILE_DEFINITIONS can be set by generator expressions instead)
if (POLICY CMP0043)
    cmake_policy(SET CMP0043 OLD)
endif (POLICY CMP0043)
# Policy CMP0042 introduced in cmake version 3.0 enables the use of @rpath in an install name via MACOSX_RPATH by default
# Set to OLD behavior so that all versions use the same behavior, or until minimum cmake version >= 2.8.12 (version where @rpath is available)
if (POLICY CMP0042)
    cmake_policy(SET CMP0042 OLD)
endif (POLICY CMP0042)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs
             rosgraph_msgs genmsg stereo_msgs move_base_msgs
)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Info.msg
   KeyPoint.msg
   GlobalDescriptor.msg
   ScanDescriptor.msg
   MapData.msg
   MapGraph.msg
   NodeData.msg
   Link.msg
   OdomInfo.msg
   Point2f.msg
   Point3f.msg
   Goal.msg
   RGBDImage.msg
   RGBDImages.msg
   UserData.msg
   GPS.msg
   Path.msg
   EnvSensor.msg
   DoorCorners.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   GetMap.srv
   GetMap2.srv
   ListLabels.srv
   PublishMap.srv
   ResetPose.srv
   SetGoal.srv
   SetLabel.srv
   GetPlan.srv
   AddLink.srv
   GetNodeData.srv
   GetNodesInRadius.srv
   LoadDatabase.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
   nav_msgs
)

catkin_package(
  CATKIN_DEPENDS sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs
                 rosgraph_msgs stereo_msgs move_base_msgs
)
