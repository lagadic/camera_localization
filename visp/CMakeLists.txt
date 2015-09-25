project(camera_localization_visp)
cmake_minimum_required(VERSION 2.6)

#----------------------------------------------------------------------
# Try to find ViSP third party library
#----------------------------------------------------------------------
find_package(VISP)

#----------------------------------------------------------------------
# Propagate in sub directories
#----------------------------------------------------------------------
add_subdirectory(homography-basis)
add_subdirectory(pose-basis)
add_subdirectory(pose-mbt)
add_subdirectory(template-matching)

