# KCF
add_executable(auto_kcf autopilot_KCF.cpp)
target_link_libraries(auto_kcf LINK_PRIVATE vision)
target_link_libraries(auto_kcf LINK_PRIVATE realsense)
target_link_libraries(auto_kcf LINK_PRIVATE autopilot)

# TM
add_executable(auto_TM autopilot_TM.cpp)
target_link_libraries(auto_TM LINK_PRIVATE vision)
target_link_libraries(auto_TM LINK_PRIVATE realsense)
target_link_libraries(auto_TM LINK_PRIVATE autopilot)

# FM
add_executable(auto_FM autopilot_FM.cpp)
target_link_libraries(auto_FM LINK_PRIVATE vision)
target_link_libraries(auto_FM LINK_PRIVATE realsense)
target_link_libraries(auto_FM LINK_PRIVATE autopilot)