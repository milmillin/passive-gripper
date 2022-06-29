cmake_minimum_required(VERSION 3.14)
project(passive-gripper)

option(CLUSTER_RELEASE_BUILD "Release build that should run on cluster" OFF)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Set build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if (CLUSTER_RELEASE_BUILD)
  # Enable link-time optimization
  set (CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
  message(STATUS "Link time optimization enabled")
endif()

# Add your project files
file(GLOB_RECURSE CORE_SRCFILES "include/passive-gripper/*.cpp")
file(GLOB_RECURSE UI_SRCFILES "src-ui/*.cpp")
file(GLOB_RECURSE BATCH_SRCFILES "src-batch/*.cpp")

include_directories("include")

if (MSVC)
  # For Microsoft compiler
  set_source_files_properties(
    ${CORE_SRCFILES}
    PROPERTIES
    COMPILE_FLAGS "/W4"
  )
  set_source_files_properties(
    ${UI_SRCFILES}
    PROPERTIES
    COMPILE_FLAGS "/W4"
  )
  set_source_files_properties(
    ${BATCH_SRCFILES}
    PROPERTIES
    COMPILE_FLAGS "/W4"
  )

  # Find LAPACK
  set(LAPACK_DIR "external/lapack")
  find_package(LAPACK CONFIG REQUIRED)

  # Copy shared libraries to binary directory
  file(GLOB LAPACK_DLLS ${LAPACK_DIR}/*.dll)
  foreach (lapack_lib ${LAPACK_DLLS})
    message(STATUS "Copying ${lapack_lib} to build directory")
    file(COPY ${lapack_lib} DESTINATION ${PROJECT_BINARY_DIR})
  endforeach()
else()
  # For other compilers
  set_source_files_properties(
    ${CORE_SRCFILES}
    PROPERTIES
    COMPILE_FLAGS "-Wall -Wextra -pedantic"
  )
  set_source_files_properties(
    ${UI_SRCFILES}
    PROPERTIES
    COMPILE_FLAGS "-Wall -Wextra -pedantic"
  )
  set_source_files_properties(
    ${BATCH_SRCFILES}
    PROPERTIES
    COMPILE_FLAGS "-Wall -Wextra -pedantic"
  )
  set(CMAKE_CXX_FLAGS_RELEASE "-O3")

  find_package(LAPACK)
endif()

message(STATUS "Copying assets")
file(COPY assets DESTINATION ${PROJECT_BINARY_DIR})

# Link OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Qhull
find_package(Qhull CONFIG REQUIRED)

# NLOPT
find_package(NLopt CONFIG REQUIRED)

# CGAL
find_package(CGAL CONFIG REQUIRED)

# Boost
find_package(Boost COMPONENTS system filesystem REQUIRED)

# Autodiff (not the best way)
include_directories("external/autodiff")


# Link libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_PNG               "Use PNG"            ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use IMGUI"          ON)
option(LIBIGL_WITH_EMBREE            "Use Embree"         ON)
option(LIBIGL_WITH_CGAL              "Use CGAL"           ON)

find_package(LIBIGL REQUIRED QUIET)

# Swept Volume
file(GLOB_RECURSE SWEPT_VOLUME_SRCFILES "external/swept-volumes/include/*.cpp")
add_library(swept-volume ${SWEPT_VOLUME_SRCFILES})
target_link_libraries(swept-volume igl::core igl::opengl igl::cgal)
include_directories("external/swept-volumes/include")

# ikfast
add_library(ikfast "external/ikfast/ikfast61.cpp")
include_directories("external/ikfast")

# Linking
set(CORE_LINK_LIBS igl::core igl::embree igl::png igl::cgal lapack Qhull::qhullcpp NLopt::nlopt CGAL::CGAL swept-volume ikfast)
if (NOT MSVC)
    list(APPEND CORE_LINK_LIBS stdc++fs Qhull::qhullstatic_r)
else()
    list(APPEND CORE_LINK_LIBS Qhull::qhull_r)
endif()



set(UI_LINK_LIBS psg-core igl::core igl::opengl_glfw igl::opengl_glfw_imgui Boost::filesystem Boost::system)

set(BATCH_LINK_LIBS psg-core Boost::filesystem Boost::system)
if (NOT MSVC)
    list(APPEND BATCH_LINK_LIBS stdc++fs)
endif()

# Enable easy profiler
if (ENABLE_EASY_PROFILER)
  message(STATUS "Using easy profiler in path:" ${EASY_PROFILER_LIB_PATH})
  list(APPEND CMAKE_PREFIX_PATH ${EASY_PROFILER_LIB_PATH})
  find_package(easy_profiler REQUIRED)
  list(APPEND CORE_LINK_LIBS easy_profiler)
  add_compile_definitions(COMPILE_WITH_EASY_PROFILER)
endif()

add_library(psg-core ${CORE_SRCFILES})
target_link_libraries(psg-core ${CORE_LINK_LIBS})

add_executable(${PROJECT_NAME} ${UI_SRCFILES})
target_link_libraries(${PROJECT_NAME} ${UI_LINK_LIBS})

add_executable(psg-batch ${BATCH_SRCFILES})
target_link_libraries(psg-batch ${BATCH_LINK_LIBS})
