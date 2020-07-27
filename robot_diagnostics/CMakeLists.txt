cmake_minimum_required(VERSION 2.8.3)
project(robot_diagnostics)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  
  geometry_msgs
  roscpp
  rospy
  std_msgs
  monitoring_core
  roslib
  robot_activity
)



#generate_dynamic_reconfigure_options(cfg/controller.cfg)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS	
    geometry_msgs
    roscpp
    rospy
    std_msgs
    monitoring_core
    robot_activity
)
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)



add_executable(node_monitoring
  src/node_monitor.cpp
  src/node_statistics.cpp

)

target_link_libraries(node_monitoring
  ${catkin_LIBRARIES} 
)

add_executable(system_monitoring
  src/system_monitor.cpp
  src/system_statistics.cpp
)

target_link_libraries(system_monitoring
  ${catkin_LIBRARIES}
)


add_executable(topic_monitoring
  src/topic_monitor.cpp
  src/topic_statistics.cpp
)

target_link_libraries(topic_monitoring
  ${catkin_LIBRARIES}
)


add_executable(sensor_monitoring
  src/sensor_monitor.cpp
  src/sensor_statistics.cpp
)

target_link_libraries(sensor_monitoring
  ${catkin_LIBRARIES}
)


add_executable(diagnostics_server
  src/diagnostics_server.cpp
)

target_link_libraries(diagnostics_server
  ${catkin_LIBRARIES}
)

add_executable(xmlrpc
  src/xmlrpc.cpp
)

target_link_libraries(xmlrpc
  ${catkin_LIBRARIES}
)



add_executable(dummyPublisher
  src/dummyPublisher.cpp
)

target_link_libraries(dummyPublisher
  ${catkin_LIBRARIES}
)

#add_dependencies(trajectory_controller ${PROJECT_NAME}_gencfg)

#find_package(PythonLibs REQUIRED)
#include_directories(${PYTHON_INCLUDE_DIRS})
#target_link_libraries(trajectory_controller ${PYTHON_LIBRARIES})

#add_executable(test_trajectory_generator
#  src/test_trajectory_generator.cpp
#  src/utilities.cpp

#)

#target_link_libraries(test_trajectory_generator
#  ${catkin_LIBRARIES}
#)
