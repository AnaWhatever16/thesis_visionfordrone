cmake_minimum_required(VERSION 3.12)
project(anas_thesis)

file(GLOB_RECURSE VISION_HDR "include/*.h")
file(GLOB_RECURSE VISION_SRC "src/*.cpp")

add_library(vision STATIC ${VISION_HDR} ${VISION_SRC})
target_include_directories(vision PUBLIC "include")

# OpenCV
find_package(OpenCV REQUIRED)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)

target_include_directories(vision PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vision LINK_PUBLIC ${OpenCV_LIBS})

# Eigen3
find_package(Eigen3 REQUIRED)
target_link_libraries(vision LINK_PUBLIC Eigen3::Eigen)