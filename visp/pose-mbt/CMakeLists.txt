project(pose-mbt-visp)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)

# set the list of source files
set(tutorial_cpp
  pose-mbt-visp.cpp)

list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.mpg")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.xml")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.cao")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.wrl")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.init")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.ppm")

include_directories(${VISP_INCLUDE_DIRS})

foreach(cpp ${tutorial_cpp})
  # Compute the name of the binary to create
  get_filename_component(binary ${cpp} NAME_WE)

  # From source compile the binary and add link rules
  add_executable(${binary} ${cpp})
  target_link_libraries(${binary} ${VISP_LIBRARIES})
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(pose-mbt-visp.cpp ${data})
endforeach()
