project(camera_localization_opencv)
cmake_minimum_required(VERSION 2.6)

#----------------------------------------------------------------------
# Try to find ViSP third party library
#----------------------------------------------------------------------
find_package(OpenCV)

#----------------------------------------------------------------------
# Propagate in sub directories
#----------------------------------------------------------------------
add_subdirectory(homography-basis)
add_subdirectory(pose-basis)
