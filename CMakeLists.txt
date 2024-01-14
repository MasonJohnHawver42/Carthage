cmake_minimum_required(VERSION "3.1")

project("aaa")

# Find OpenGL, set link library names and include paths
find_package(OpenGL REQUIRED)
set(OPENGL_LIBRARIES ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
set(OPENGL_INCLUDE_DIRS ${OPENGL_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIRS})

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

set(
    SOURCES
    ext/glad/src/glad.c
)

set(
    INCLUDE_DIRS
    src/
    ext/
    ext/glfw/include
    ext/glad/include
    ext/stb
    ext/tinyobjloader
)

set(
    LIBS
    glfw
    ${OPENGL_LIBRARIES}
)

foreach(MAIN_SOURCE ${MAIN_SOURCES})

    get_filename_component(MAIN_NAME ${MAIN_SOURCE} NAME_WE)

    add_executable(${PROJECT_NAME}_${MAIN_NAME} ${MY_SOURCES} ${SOURCES} ${MAIN_SOURCE})

    include_directories(${INCLUDE_DIRS})

    target_link_libraries(${PROJECT_NAME}_${MAIN_NAME} PRIVATE ${LIBS})

    # For MS Visual Code
    if (MSVC)
        target_compile_options(${PROJECT_NAME}_${MAIN_NAME} PRIVATE "/MP")
        set_property(TARGET ${PROJECT_NAME}_${MAIN_NAME} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME}_${MAIN_NAME})
    endif()
endforeach()