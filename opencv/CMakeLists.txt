cmake_minimum_required(VERSION 3.10)
project(camera_localization_opencv)

#----------------------------------------------------------------------
# Try to find ViSP third party library
#----------------------------------------------------------------------
find_package(OpenCV)

#----------------------------------------------------------------------
# Propagate in sub directories
#----------------------------------------------------------------------
add_subdirectory(homography-basis)
add_subdirectory(pose-basis)
