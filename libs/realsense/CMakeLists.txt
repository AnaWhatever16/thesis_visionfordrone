cmake_minimum_required(VERSION 3.12)
project(anas_thesis)

file(GLOB_RECURSE REALSENSE_HDR "include/*.h")
file(GLOB_RECURSE REALSENSE_SRC "src/*.cpp")

add_library(realsense STATIC ${REALSENSE_HDR} ${REALSENSE_SRC})
target_include_directories(realsense PUBLIC "include")

# OpenCV
find_package(OpenCV REQUIRED)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)

target_include_directories(realsense PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(realsense LINK_PUBLIC ${OpenCV_LIBS})

# Eigen3
find_package(Eigen3 REQUIRED)
target_link_libraries(realsense LINK_PUBLIC Eigen3::Eigen)

# PCL
find_package(PCL REQUIRED)
target_include_directories(realsense PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(realsense LINK_PUBLIC ${PCL_LIBRARIES})

# Realsense
find_package(realsense2 REQUIRED HINTS "/usr/local/lib/cmake/realsense2")
MESSAGE(STATUS "Found librealsense v2, compiling RealSense devices wrapper")
target_compile_definitions(realsense PUBLIC "ENABLE_LIBREALSENSE_V2")
target_include_directories(realsense PUBLIC ${realsense2_INCLUDE_DIRS})
target_link_libraries(realsense LINK_PUBLIC ${realsense2_LIBRARY})
