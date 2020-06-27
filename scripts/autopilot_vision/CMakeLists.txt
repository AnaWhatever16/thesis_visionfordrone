# Circulant Matrix example
add_executable(auto_kcf autopilot_KCF.cpp)
target_link_libraries(auto_kcf LINK_PRIVATE vision)
target_link_libraries(auto_kcf LINK_PRIVATE realsense)
target_link_libraries(auto_kcf LINK_PRIVATE autopilot)