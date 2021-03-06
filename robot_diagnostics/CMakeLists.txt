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
  visualization_msgs
  message_filters
  bag_recorder
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
    visualization_msgs
    message_filters
    bag_recorder
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
  src/node_processing.cpp

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




add_executable(diagnostics_aggregator
  src/diagnostics_aggregator.cpp
)

target_link_libraries(diagnostics_aggregator
  ${catkin_LIBRARIES}
)


add_executable(diagnostics_recovery
  src/diagnostics_recovery.cpp
)

target_link_libraries(diagnostics_recovery
  ${catkin_LIBRARIES}
)


add_executable(dummyBattery
  src/dummyBattery.cpp
)

target_link_libraries(dummyBattery
  ${catkin_LIBRARIES}
)


add_executable(dummyImu
  src/dummyImu.cpp
)

target_link_libraries(dummyImu
  ${catkin_LIBRARIES}
)



add_executable(rosbag_recorder
  src/rosbag_recorder.cpp
)

target_link_libraries(rosbag_recorder
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
