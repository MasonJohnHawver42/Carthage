cmake_minimum_required(VERSION "3.1")

include(CMakePrintHelpers)

project("aaa")

# Find OpenGL, set link library names and include paths
find_package(OpenGL REQUIRED)
set(OPENGL_LIBRARIES ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
set(OPENGL_INCLUDE_DIRS ${OPENGL_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIRS})

# find_package(Boost REQUIRED)
# find_package(CGAL REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Disable building some of the extra things GLFW has (examples, tests, docs)
set(GLFW_BUILD_EXAMPLES  OFF CACHE BOOL " " FORCE)
set(GLFW_BUILD_TESTS     OFF CACHE BOOL " " FORCE)
set(GLFW_BUILD_DOCS      OFF CACHE BOOL " " FORCE)

add_definitions( -DMY_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src/" )
add_definitions( -DMY_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data/" )

# Run cmake on the CMakeLists.txt file found inside of the GLFW directory
add_subdirectory(ext/glfw)

# Source files
file(GLOB_RECURSE MY_SOURCES "src/*.cpp")

set(
    MAIN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/test.cpp
)

list(REMOVE_ITEM MY_SOURCES ${MAIN_SOURCES})

file(GLOB IMGUI_SOURCE_FILES "ext/imgui/*.cpp")

list(APPEND MY_SOURCES ${IMGUI_SOURCE_FILES})
list(APPEND MY_SOURCES "ext/glad/src/glad.c")
list(APPEND MY_SOURCES "ext/imgui/backends/imgui_impl_glfw.cpp")
list(APPEND MY_SOURCES "ext/imgui/backends/imgui_impl_opengl3.cpp")

set(
    INCLUDE_DIRS
    src/
    ext/
    ext/glfw/include
    ext/glad/include
    ext/stb
    ext/tinyobjloader
    ext/imgui
    ext/imgui/backends
)

set(
    LIBS
    glfw
    ${OPENGL_LIBRARIES}
)

foreach(MAIN_SOURCE ${MAIN_SOURCES})

    get_filename_component(MAIN_NAME ${MAIN_SOURCE} NAME_WE)

    add_executable(${PROJECT_NAME}_${MAIN_NAME} ${MY_SOURCES} ${MAIN_SOURCE})

    include_directories(${INCLUDE_DIRS})

    target_link_libraries(${PROJECT_NAME}_${MAIN_NAME} PRIVATE ${LIBS})

    # For MS Visual Code
    if (MSVC)
        target_compile_options(${PROJECT_NAME}_${MAIN_NAME} PRIVATE "/MP")
        set_property(TARGET ${PROJECT_NAME}_${MAIN_NAME} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME}_${MAIN_NAME})
    endif()
endforeach()