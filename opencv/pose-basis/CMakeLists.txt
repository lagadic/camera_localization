cmake_minimum_required(VERSION 3.10)
project(pose-basis-opencv)

find_package(OpenCV REQUIRED)

set (CMAKE_CXX_STANDARD 11)

include_directories(${OpenCV_INCLUDE_DIRS})

set(tutorial_cpp
  pose-dlt-opencv.cpp
  pose-dementhon-opencv.cpp
  pose-gauss-newton-opencv.cpp)

foreach(cpp ${tutorial_cpp})
  # Compute the name of the binary to create
  get_filename_component(binary ${cpp} NAME_WE)

  # From source compile the binary and add link rules
  add_executable(${binary} ${cpp})
  target_link_libraries(${binary} ${OpenCV_LIBRARIES})
endforeach()

