cmake_minimum_required (VERSION 3.12 FATAL_ERROR)
project(anas_thesis)

file(GLOB_RECURSE AUTOPILOT_HDR "include/*.h" "include/*.inl")
file(GLOB_RECURSE AUTOPILOT_SRC "src/*.cpp")

add_library(autopilot STATIC ${AUTOPILOT_HDR} ${AUTOPILOT_SRC})
target_include_directories(autopilot PUBLIC "include")

# OpenCV
find_package(OpenCV REQUIRED)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)

target_include_directories(autopilot PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(autopilot LINK_PUBLIC ${OpenCV_LIBS})

# ROS for UAV
find_package(catkin REQUIRED COMPONENTS  roscpp sensor_msgs std_msgs geometry_msgs cv_bridge
                                mavros  mavros_msgs
                                uav_abstraction_layer ual_backend_gazebo_light)

target_include_directories(autopilot PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(autopilot LINK_PUBLIC ${catkin_LIBRARIES})