cmake_minimum_required(VERSION 3.2)
project(earcut_hpp)

option(EARCUT_BUILD_TESTS "Build the earcut test program" ON)
option(EARCUT_BUILD_BENCH "Build the earcut benchmark program" ON)
option(EARCUT_BUILD_VIZ "Build the earcut visualizer program" ON)
option(EARCUT_WARNING_IS_ERROR "Treat warnings as errors" OFF)

# dependencies
if (EARCUT_BUILD_VIZ)
    # OpenGL
    # linux: xorg-dev libgl1-mesa-glx libgl1-mesa-dev
    # windows: in the windows sdk
    find_package(OpenGL REQUIRED)
    include_directories(SYSTEM ${OPENGL_INCLUDE_DIRS})

    # GLFW3
    if(EXISTS "${PROJECT_SOURCE_DIR}/.gitmodules")
        execute_process(
                COMMAND             git submodule update --init --recursive
                WORKING_DIRECTORY   ${PROJECT_SOURCE_DIR}
                OUTPUT_QUIET
                ERROR_QUIET
        )
    endif()
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "Build the GLFW example programs" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "Build the GLFW test programs" FORCE)
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "Build the GLFW documentation" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "Generate installation target" FORCE)
    add_subdirectory(glfw)
    set(GLFW_LIBS glfw ${GLFW_LIBRARIES})
    include_directories(SYSTEM "glfw/include")
endif()

# setup compiler flags for earcut
set(CMAKE_CXX_STANDARD 11)
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type specified. Setting to 'Release'")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "The type of build." FORCE)
endif()
if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang$" OR CMAKE_COMPILER_IS_GNUCXX)
    if ("${CMAKE_CXX_FLAGS}" MATCHES "--coverage")
        add_definitions(-DNDEBUG)
    else()
        include(CheckCXXCompilerFlag)
        check_cxx_compiler_flag("-fsanitize=undefined" HAVE_FLAG_SANITIZE_UNDEFINED)
        if(HAVE_FLAG_SANITIZE_UNDEFINED)
            set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=undefined")
        endif()
    endif()
    add_compile_options("-pipe" "-Wall" "-Wextra" "-Wconversion" "-Wpedantic")
    if (EARCUT_WARNING_IS_ERROR)
        add_compile_options("-Werror")
    endif()
    # optional: -march=native (builds with the optimizations available on the build machine (only for local use!))
elseif(MSVC)
    if (EARCUT_WARNING_IS_ERROR)
        add_compile_options("/WX")
    endif()
endif()

# earcut.hpp
include_directories("include")

file(GLOB FIXTURE_SOURCE_FILES test/fixtures/*.cpp test/fixtures/*.hpp)
source_group(fixtures FILES ${FIXTURE_SOURCE_FILES})

if (MSVC)
    set_source_files_properties(${FIXTURE_SOURCE_FILES} PROPERTIES COMPILE_FLAGS "/Od")
endif()

file(GLOB COMPARISON_SOURCE_FILES test/comparison/*.cpp test/comparison/*.hpp)
source_group(comparison FILES ${COMPARISON_SOURCE_FILES})

file(GLOB LIBTESS2_SOURCE_FILES test/comparison/libtess2/*.c test/comparison/libtess2/*.h)
source_group(comparison\\libtess2 FILES ${LIBTESS2_SOURCE_FILES})

if (MSVC)
    set_source_files_properties(${LIBTESS2_SOURCE_FILES} PROPERTIES COMPILE_FLAGS "/wd4244 /wd4267")
else()
    set_source_files_properties(${LIBTESS2_SOURCE_FILES} PROPERTIES COMPILE_FLAGS "-w")
endif()

set(COMMON_SOURCE_FILES ${LIBTESS2_SOURCE_FILES} ${FIXTURE_SOURCE_FILES} ${COMPARISON_SOURCE_FILES})

set(TESTS_SOURCE_FILES ${COMMON_SOURCE_FILES} test/tap.cpp test/tap.hpp test/test.cpp)
set(BENCH_SOURCE_FILES ${COMMON_SOURCE_FILES} test/bench.cpp)
set(VIZ_SOURCE_FILES ${COMMON_SOURCE_FILES} test/viz.cpp)

if (EARCUT_BUILD_TESTS)
    add_executable(tests ${TESTS_SOURCE_FILES})
    target_link_libraries(tests ${Boost_LIBRARIES})
endif()
if (EARCUT_BUILD_BENCH)
    add_executable(bench ${BENCH_SOURCE_FILES})
    target_link_libraries(bench ${Boost_LIBRARIES})
endif()
if (EARCUT_BUILD_VIZ)
    add_executable(viz ${VIZ_SOURCE_FILES})
    target_link_libraries(viz ${Boost_LIBRARIES} ${GLFW_LIBS} ${OPENGL_LIBRARIES})
endif()
