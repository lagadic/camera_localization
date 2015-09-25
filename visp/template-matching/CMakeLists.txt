project(template-matching-visp)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)

# set the list of source files
set(tutorial_cpp
  template-matching-visp.cpp)

list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/bruegel.mpg")

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
  visp_copy_data(template-matching-visp.cpp ${data})
endforeach()
