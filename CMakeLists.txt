cmake_minimum_required(VERSION 3.1)
project(passive-gripper)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Set build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")


# Add your project files
file(GLOB_RECURSE SRCFILES "src/*.cpp")

if (MSVC)
  # For Microsoft compiler
  # Enable OpenMP
  # Enable Warning
  set_source_files_properties(
    ${SRCFILES}
    PROPERTIES
    COMPILE_FLAGS "/W4 /openmp:experimental"
  )
else()
  # For other compilers
  # Enable Warning
  set_source_files_properties(
    ${SRCFILES}
    PROPERTIES
    COMPILE_FLAGS "-Wall -Wextra -pedantic"
  )
endif()

# Link libraries
set(LINK_LIBS igl::core igl::opengl_glfw igl::opengl_glfw_imgui igl::embree embree)
if (NOT MSVC)
    list(APPEND LINK_LIBS stdc++fs)
endif()

# Link libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use IMGUI"          ON)
option(LIBIGL_WITH_EMBREE            "Use Embree"         ON)

find_package(LIBIGL REQUIRED QUIET)

# Link Embree
find_package(embree 3 CONFIG REQUIRED)

# Linking
add_executable(${PROJECT_NAME} ${SRCFILES})
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
