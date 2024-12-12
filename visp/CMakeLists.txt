cmake_minimum_required(VERSION 3.10)
project(camera_localization_visp)

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

