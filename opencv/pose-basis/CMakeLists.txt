project(pose-basis-opencv)
cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED)

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

