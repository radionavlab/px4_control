cmake_minimum_required(VERSION 2.8.3)
project(px4_control)

option(USE_KINETIC "Use kinetic version instead of indigo version" TRUE) # TRUE/FALSE

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs
  message_generation
  tf
  cmake_modules
  mg_msgs
)

find_package(Eigen REQUIRED)

catkin_package(
	DEPENDS Eigen
  CATKIN_DEPENDS mg_msgs
#  INCLUDE_DIRS include
#  LIBRARIES px4_control
#  CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs roscpp std_msgs visualization_msgs
#  DEPENDS system_lib
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

if(USE_KINETIC)
    add_executable(px4_control_node 
                   src/px4_control.cpp 
                   src/HelperFunctions/helper.cpp
                   src/HelperFunctions/QuatRotEuler.cpp
                   src/Callbacks/callbacks.cpp
                   src/Threads/FSMTaskKinetic.cpp
                   src/Threads/joyTask.cpp
                   src/Threads/commPubKinetic.cpp
                   src/Threads/watchdogs.cpp
                   src/PosControl/PosControlKinetic.cpp
                   src/JoyDrivers/joyDrivers.cpp
                   src/pevents/pevents.cpp
                   src/Services/services.cpp
                   src/structs.cpp)
          target_link_libraries(px4_control_node ${catkin_LIBRARIES})

    #This makes sure that messages and services are compiled before the rest
    add_dependencies(px4_control_node px4_control_generate_messages_cpp)
    add_dependencies(px4_control_node ${catkin_EXPORTED_TARGETS})
    add_dependencies(px4_control_node mg_msgs_gencpp)
else()
    add_executable(px4_control_node 
                   src/px4_control.cpp 
                   src/HelperFunctions/helper.cpp
                   src/HelperFunctions/QuatRotEuler.cpp
                   src/Callbacks/callbacks.cpp
                   src/Threads/FSMTask.cpp
                   src/Threads/joyTask.cpp
                   src/Threads/commPub.cpp
                   src/Threads/watchdogs.cpp
                   src/JoyDrivers/joyDrivers.cpp
                   src/PosControl/PosControl.cpp
                   src/pevents/pevents.cpp
                   src/Services/services.cpp
                   src/structs.cpp)
          target_link_libraries(px4_control_node ${catkin_LIBRARIES})

    #This makes sure that messages and services are compiled before the rest
    add_dependencies(px4_control_node px4_control_generate_messages_cpp)
    add_dependencies(px4_control_node ${catkin_EXPORTED_TARGETS})
    add_dependencies(px4_control_node mg_msgs_gencpp)
endif(USE_KINETIC)
unset(USE_KINETIC CACHE)



