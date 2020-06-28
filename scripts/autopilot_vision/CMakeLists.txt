# KCF
add_executable(auto_KCF autopilot_KCF.cpp)
target_link_libraries(auto_KCF LINK_PRIVATE vision)
target_link_libraries(auto_KCF LINK_PRIVATE realsense)
target_link_libraries(auto_KCF LINK_PRIVATE autopilot)

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

# LK
add_executable(auto_LK autopilot_LK.cpp)
target_link_libraries(auto_LK LINK_PRIVATE vision)
target_link_libraries(auto_LK LINK_PRIVATE realsense)
target_link_libraries(auto_LK LINK_PRIVATE autopilot)

# TM + KCF
add_executable(auto_cool autopilot_TMKCF.cpp)
target_link_libraries(auto_cool LINK_PRIVATE vision)
target_link_libraries(auto_cool LINK_PRIVATE realsense)
target_link_libraries(auto_cool LINK_PRIVATE autopilot)