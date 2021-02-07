project(homography-basis-visp)
cmake_minimum_required(VERSION 3.0)

find_package(VISP REQUIRED)

include_directories(${VISP_INCLUDE_DIRS})

set(tutorial_cpp
  homography-dlt-visp.cpp
  pose-from-homography-dlt-visp.cpp)

foreach(cpp ${tutorial_cpp})
  # Compute the name of the binary to create
  get_filename_component(binary ${cpp} NAME_WE)

  # From source compile the binary and add link rules
  add_executable(${binary} ${cpp})
  target_link_libraries(${binary} ${VISP_LIBRARIES})
endforeach()

