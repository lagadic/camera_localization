project(camera_localization)
cmake_minimum_required(VERSION 2.6)

#----------------------------------------------------------------------
# Try to find ViSP third party library
#----------------------------------------------------------------------
find_package(VISP REQUIRED)

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
subdirs(homography-basis)
subdirs(pose-basis)
subdirs(pose-mbt)
subdirs(template-matching)

