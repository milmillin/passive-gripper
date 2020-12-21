cmake_minimum_required(VERSION 3.1)
project(passive-gripper)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Link libraries
set(LINK_LIBS igl::core igl::opengl_glfw igl::opengl_glfw_imgui)
if (NOT MSVC)
    list(APPEND LINK_LIBS stdc++fs)
endif()

# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use IMGUI"          ON)

find_package(LIBIGL REQUIRED QUIET)

# Add your project files
file(GLOB_RECURSE SRCFILES "src/*.cpp")

add_executable(${PROJECT_NAME} ${SRCFILES})
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
