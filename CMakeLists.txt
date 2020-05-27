cmake_minimum_required (VERSION 3.12 FATAL_ERROR)
project(anas_thesis)

add_subdirectory(libs/autopilot)
add_subdirectory(libs/vision)
add_subdirectory(libs/realsense)
#add_subdirectory(scripts)

# #########################################
# ######			Documentation	   ######
# #########################################
find_package(Doxygen)
if(DOXYGEN_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
  add_custom_target(	doc_ana_thesis
            ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen" VERBATIM )
endif(DOXYGEN_FOUND)