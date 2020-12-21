cmake_minimum_required(VERSION 3.1)
project(passive-gripper)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Add your project files
file(GLOB_RECURSE SRCFILES "src/*.cpp")

# Enable OpenMP
if (MSVC)
  # For Microsoft compilers
  set_source_files_properties(
    ${SRCFILES}
    PROPERTIES
    COMPILE_FLAGS "/openmp:experimental"
  )
endif()

# Link libraries
set(LINK_LIBS igl::core igl::opengl_glfw igl::opengl_glfw_imgui igl::embree embree)
if (NOT MSVC)
    list(APPEND LINK_LIBS stdc++fs)
endif()

# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use IMGUI"          ON)
option(LIBIGL_WITH_EMBREE            "Use Embree"         ON)

find_package(LIBIGL REQUIRED QUIET)

find_package(embree 3 CONFIG REQUIRED)


add_executable(${PROJECT_NAME} ${SRCFILES})
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
