project(camera-localization)
cmake_minimum_required(VERSION 2.6)

option(WITH_OPENCV "Implementation using OpenCV structures" ON)
option(WITH_VISP "Implementation using OpenCV structures" ON)

#----------------------------------------------------------------------
# Try to find doxygen for documentation generation
# Use "make doc" target to generate the documentation
#----------------------------------------------------------------------
find_package(Doxygen)
if(DOXYGEN_FOUND)
  configure_file(doc/Doxyfile.cmake
    ${PROJECT_BINARY_DIR}/Doxyfile
    @ONLY )

  configure_file(doc/mainpage.doc.cmake
    ${PROJECT_BINARY_DIR}/doc/mainpage.doc
    @ONLY )

  add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile)
endif()

#----------------------------------------------------------------------
# Propagate in sub directories
#----------------------------------------------------------------------
if(WITH_VISP)
  add_subdirectory(visp)
endif()

if(WITH_OPENCV)
  add_subdirectory(opencv)
endif()
