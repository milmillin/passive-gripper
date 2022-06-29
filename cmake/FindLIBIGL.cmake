# - Try to find the LIBIGL library
# Once done this will define
#
#  LIBIGL_FOUND - system has LIBIGL
#  LIBIGL_INCLUDE_DIR - **the** LIBIGL include directory
if(LIBIGL_FOUND)
    return()
endif()

set(EXPECTED_LIBIGL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/external/libigl/include)
if(EXISTS ${EXPECTED_LIBIGL_INCLUDE_DIR})
    set(LIBIGL_INCLUDE_DIR ${EXPECTED_LIBIGL_INCLUDE_DIR})
    message("-- Found libigl:" ${LIBIGL_INCLUDE_DIR})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBIGL
    "\nlibigl not found --- You can download it using:\n\tgit clone https://github.com/libigl/libigl.git ${CMAKE_SOURCE_DIR}/external/libigl"
    LIBIGL_INCLUDE_DIR)
mark_as_advanced(LIBIGL_INCLUDE_DIR)

list(APPEND CMAKE_MODULE_PATH "${LIBIGL_INCLUDE_DIR}/../cmake")
include(libigl)
