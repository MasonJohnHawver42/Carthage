cmake_minimum_required(VERSION 3.5)

include(CMakePrintHelpers)

project(carthage)

# Find OpenGL, set link library names and include paths
set(OpenGL_GL_PREFERENCE "LEGACY")
find_package(OpenGL REQUIRED)
set(OPENGL_LIBRARIES ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})

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
file(GLOB MAIN_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
list(REMOVE_ITEM MY_SOURCES ${MAIN_SOURCES})

# Include imgui
add_library(glad STATIC
    ext/glad/src/glad.c
)

# Include imgui
add_library(imgui STATIC
    ext/imgui/imgui.cpp
    ext/imgui/imgui_demo.cpp
    ext/imgui/imgui_draw.cpp
    ext/imgui/imgui_tables.cpp
    ext/imgui/imgui_widgets.cpp
)

add_library(carthage STATIC
    ext/imgui/backends/imgui_impl_glfw.cpp
    ext/imgui/backends/imgui_impl_opengl3.cpp
    ${MY_SOURCES}
)

target_link_libraries(carthage PRIVATE
    glad
    glfw
    imgui
    ${OPENGL_gl_LIBRARY} 
    ${OPENGL_glu_LIBRARY}
)

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
    ${OPENGL_INCLUDE_DIR}
)

include_directories(${INCLUDE_DIRS})

set(
    LIBS
    glad
    glfw
    imgui
    carthage
    ${OPENGL_gl_LIBRARY} 
    ${OPENGL_glu_LIBRARY}
)

foreach(MAIN_SOURCE ${MAIN_SOURCES})
    get_filename_component(MAIN_NAME ${MAIN_SOURCE} NAME_WE)
    
    add_executable(${MAIN_NAME} ${MAIN_SOURCE})
    target_link_libraries(${MAIN_NAME} PRIVATE ${LIBS})
    
    if (MSVC) # For MS Visual Code
        target_compile_options(${PROJECT_NAME}_${MAIN_NAME} PRIVATE "/MP")
        set_property(TARGET ${PROJECT_NAME}_${MAIN_NAME} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME}_${MAIN_NAME})
    endif()
endforeach()
