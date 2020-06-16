# Circulant Matrix example
add_executable(permut mainCirculantMatrix.cpp)
target_link_libraries(permut LINK_PRIVATE vision)
target_link_libraries(permut LINK_PRIVATE realsense)

# Example Features
add_executable(exfeat mainExampleFeatures.cpp)
target_link_libraries(exfeat LINK_PRIVATE realsense)

# Template Matching
add_executable(template_matching mainTemplMatch.cpp)
target_link_libraries(template_matching LINK_PRIVATE vision)
target_link_libraries(template_matching LINK_PRIVATE realsense)

# Template Matching with threads
add_executable(template_thread mainTemplMatchThread.cpp)
target_link_libraries(template_thread LINK_PRIVATE vision)
target_link_libraries(template_thread LINK_PRIVATE realsense)

# Feature Matching
add_executable(features mainFeature.cpp)
target_link_libraries(features LINK_PRIVATE vision)
target_link_libraries(features LINK_PRIVATE realsense)

# KCF 
add_executable(KCF mainKCF.cpp)
target_link_libraries(KCF LINK_PRIVATE vision)
target_link_libraries(KCF LINK_PRIVATE realsense)

# Optical Flow Lucas-Kanade
add_executable(lucas mainLucas.cpp)
target_link_libraries(lucas LINK_PRIVATE vision)
target_link_libraries(lucas LINK_PRIVATE realsense)