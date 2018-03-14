cmake_minimum_required(VERSION 3.5)

# DESCRIPTION "A small library for outputting CAD models from C++"
project(libcppcad
    VERSION 0.1.0)

# Define variables for `cpack`
SET(MAJOR_VERSION 0)
SET(MINOR_VERSION 1)
SET(PATCH_VERSION 0)

include(GNUInstallDirs)

# From <https://cmake.org/Wiki/CMake/Tutorials/How_to_create_a_ProjectConfig.cmake_file>:
# Offer the user the choice of overriding the installation directories:
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
    "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
    set(DEF_INSTALL_CMAKE_DIR CMake)
else()
    set(DEF_INSTALL_CMAKE_DIR lib/cmake/CppCad)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
    "Installation directory for CMake files")
 
# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
    set(var INSTALL_${p}_DIR)
    if(NOT IS_ABSOLUTE "${${var}}")
        set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
    endif()
endforeach()

# Set language version:
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11" )

add_library(cppcad INTERFACE)

target_include_directories(cppcad INTERFACE
    $<BUILD_INTERFACE:"${PROJECT_SOURCE_DIR}/include">
    $<INSTALL_INTERFACE:"include">)

# This is a headers-only library for now.  If binaries are distributed, the
# following lines would be uncommented.
#
# Set SOVERSION to major number of VERSION, so, e.g., thelib.so.1 will be a
# symlink to libthelib.so.1.0.0.
#set_target_properties(cppcad PROPERTIES
#    VERSION ${PROJECT_VERSION}
#    SOVERSION 0
#    PUBLIC_HEADER include/cpp_cad.h)


# When expanding the pkg-config file, don't expand ${VAR}s:
configure_file(cppcad.pc.in cppcad.pc @ONLY)

# Create CMake package file:
# Add all targets to the build-tree export set
export(TARGETS cppcad
    FILE "${PROJECT_BINARY_DIR}/CppCadTargets.cmake")
configure_file(CppCadConfig.cmake.in
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CppCadConfig.cmake" @ONLY)
configure_file(CppCadConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/CppCadConfigVersion.cmake" @ONLY)

# Declare files to install:
install(TARGETS cppcad
    EXPORT CppCadTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES "${PROJECT_SOURCE_DIR}/include/cpp_cad.h"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT headers)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/include/cpp_cad"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT headers)

# Install the pkg-config file:
install(FILES ${CMAKE_BINARY_DIR}/cppcad.pc DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)

# Install the CppCadConfig.cmake and CppCadConfigVersion.cmake file:
install(FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CppCadConfig.cmake"
    "${PROJECT_BINARY_DIR}/CppCadConfigVersion.cmake"
    DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)
# Install the export set for use with the install-tree
install(EXPORT CppCadTargets DESTINATION
    "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# CPack variables for building a Debian package:
IF(EXISTS "${CMAKE_ROOT}/Modules/CPack.cmake")
    INCLUDE(InstallRequiredSystemLibraries)

    SET(CPACK_SET_DESTDIR "on")
    SET(CPACK_PACKAGING_INSTALL_PREFIX "/tmp")
    SET(CPACK_GENERATOR "DEB")

    SET(CPACK_PACKAGE_DESCRIPTION "A small library for outputting CAD models from C++")
    # Multi-line Deb fields have to have subsequent lines indented by 1 space,
    # and empty lines converted to '.':
    SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY
"It implements tesselatons for a few common primitives.  They can be
 used in operations supported by the CGAL library, such as
 constructive solid geometry unions, differences, conves hulls, and
 Minkowski sums.
 .
 C++ code is compiled to a binary, which is run to output the CAD
 model.  CGAL comes with support for outputting and parsing OFF,
 Wavefront OBJ, VRML, OpenInventor.")
    SET(CPACK_PACKAGE_VENDOR "Pavel Penev")
    SET(CPACK_PACKAGE_CONTACT "Pavel Penev <pavpen@gmail.com>")
    SET(CPACK_PACKAGE_VERSION_MAJOR "${MAJOR_VERSION}")
    SET(CPACK_PACKAGE_VERSION_MINOR "${MINOR_VERSION}")
    SET(CPACK_PACKAGE_VERSION_PATCH "${PATCH_VERSION}")
    SET(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}_${MAJOR_VERSION}.${MINOR_VERSION}.${CPACK_PACKAGE_VERSION_PATCH}")
    SET(CPACK_SOURCE_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}_${MAJOR_VERSION}.${MINOR_VERSION}.${CPACK_PACKAGE_VERSION_PATCH}")

    SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.23), libgcc1 (>= 1:6.0.1), libstdc++6 (>= 5.3.1), libcgal-dev (>= 4.7)")

    SET(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
    SET(CPACK_DEBIAN_PACKAGE_SECTION "devel")
    SET(CPACK_DEBIAN_ARCHITECTURE ${CMAKE_SYSTEM_PROCESSOR})

    SET(CPACK_COMPONENTS_ALL Libraries ApplicationData)
    INCLUDE(CPack)
ENDIF(EXISTS "${CMAKE_ROOT}/Modules/CPack.cmake")
