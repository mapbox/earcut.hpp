cmake_minimum_required(VERSION 3.16)
project(earcut_hpp LANGUAGES CXX C)

option(EARCUT_BUILD_TESTS "Build the earcut test program" ON)
option(EARCUT_BUILD_BENCH "Build the earcut benchmark program" ON)
option(EARCUT_BUILD_VIZ "Build the earcut visualizer program" OFF)
option(EARCUT_WARNING_IS_ERROR "Treat warnings as errors" OFF)

include(FetchContent)

if (NOT CMAKE_BUILD_TYPE AND NOT GENERATOR_IS_MULTI_CONFIG)
    message(STATUS "No build type specified. Setting to 'Release'")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "The type of build." FORCE)
endif()


include(GNUInstallDirs)

add_library(earcut_hpp INTERFACE)
add_library(earcut_hpp::earcut_hpp ALIAS earcut_hpp)

target_include_directories(earcut_hpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# The shipping header stays C++14-compatible; the test/bench/viz tree uses C++17.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fixtures and their expectations are read at runtime from the mapbox/earcut JS repo,
# fetched here and pinned by tag (kept in lockstep with the port — bump on version updates).
FetchContent_Declare(earcut_js
    GIT_REPOSITORY https://github.com/mapbox/earcut.git
    GIT_TAG v3.0.2
    GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(earcut_js)

# picojson: lightweight single-header JSON parser for reading the fixtures at runtime.
FetchContent_Declare(picojson
    GIT_REPOSITORY https://github.com/kazuho/picojson.git
    GIT_TAG v1.3.0
    GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(picojson)

add_library(fixtures OBJECT test/fixtures/geometries.cpp test/fixtures/geometries.hpp)
target_compile_options(fixtures PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/Od>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-ffp-contract=off>
)
target_link_libraries(fixtures PUBLIC earcut_hpp)
target_include_directories(fixtures PRIVATE ${picojson_SOURCE_DIR})
target_compile_definitions(fixtures PRIVATE
    EARCUT_FIXTURE_DIR="${earcut_js_SOURCE_DIR}/test/fixtures"
    EARCUT_EXPECTED_JSON="${earcut_js_SOURCE_DIR}/test/expected.json")


file(GLOB COMPARISON_SOURCE_FILES test/comparison/*.cpp test/comparison/*.hpp)
source_group(comparison FILES ${COMPARISON_SOURCE_FILES})
# this is interface since there is no cpp files in the comparison directory
add_library(comparison INTERFACE)


add_library(common INTERFACE)
target_link_libraries(common INTERFACE comparison)

# optional: -march=native (builds with the optimizations available on the build machine (only for local use!))
target_compile_options(common INTERFACE
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-pipe -Wall -Wextra -Wconversion -Wpedantic>
)

if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang$" OR CMAKE_COMPILER_IS_GNUCXX)
    if ("${CMAKE_CXX_FLAGS}" MATCHES "--coverage")
        # We disable debug code for the coverage so it won't see assertion and other things only enabled for debugging
        target_compile_definitions(common INTERFACE NDEBUG)
    else()
        # Here we enable the undefined behavior sanitizer for the tests, benchmarks and the viz
        include(CheckCXXCompilerFlag)
        check_cxx_compiler_flag("-fsanitize=undefined" HAVE_FLAG_SANITIZE_UNDEFINED)
        if(HAVE_FLAG_SANITIZE_UNDEFINED)
            target_compile_options(common INTERFACE $<$<CONFIG:Debug>:-fsanitize=undefined>)
            target_link_options(common INTERFACE $<$<CONFIG:Debug>:-fsanitize=undefined>)
        endif()
    endif()
endif()

if (EARCUT_WARNING_IS_ERROR)
    target_compile_options(common INTERFACE
        $<$<CXX_COMPILER_ID:MSVC>:/WX>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Werror>
    )
endif()

if (EARCUT_BUILD_TESTS)
    enable_testing()

    # GoogleTest setup
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    set(BUILD_GTEST ON CACHE BOOL "" FORCE)
    FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.15.2
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(googletest)

    add_executable(tests test/test.cpp $<TARGET_OBJECTS:fixtures>)
    target_link_libraries(tests PRIVATE earcut_hpp common gtest gtest_main)

    # Fix floating-point precision issues on different architectures (see issues #97 and #103)
    target_compile_options(tests PRIVATE
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-ffp-contract=off>
    )

    add_test(NAME earcut_tests COMMAND tests)
endif()
if (EARCUT_BUILD_BENCH)
    # Google Benchmark setup
    set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
    set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(benchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG v1.9.0
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(benchmark)

    # libtess2: fetched download-only (no upstream CMake, so MakeAvailable just populates) and built
    # here as a static lib linked into the benchmark alone — purely a performance comparison point
    # (A4). It provides no correctness coverage, so tests/viz never see it.
    FetchContent_Declare(libtess2
        GIT_REPOSITORY https://github.com/memononen/libtess2.git
        GIT_TAG v1.0.2
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(libtess2)

    file(GLOB LIBTESS2_SOURCE_FILES ${libtess2_SOURCE_DIR}/Source/*.c)
    add_library(libtess2 STATIC ${LIBTESS2_SOURCE_FILES})
    target_include_directories(libtess2 PUBLIC ${libtess2_SOURCE_DIR}/Include)
    target_compile_options(libtess2 PRIVATE
        $<$<CXX_COMPILER_ID:MSVC>:/wd4244 /wd4267>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-w>
    )

    add_executable(bench test/bench.cpp $<TARGET_OBJECTS:fixtures>)
    target_link_libraries(bench PRIVATE earcut_hpp common benchmark::benchmark libtess2)
endif()
if (EARCUT_BUILD_VIZ)
    add_executable(viz test/viz.cpp $<TARGET_OBJECTS:fixtures>)

    # Setup viz target
    # OpenGL
    # linux: xorg-dev libgl1-mesa-glx libgl1-mesa-dev
    # windows: in the windows sdk
    find_package(OpenGL REQUIRED)

    # GLFW3
    find_package(glfw3 QUIET) # try to use the system default
    if (NOT glfw3_FOUND)
        set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "Build the GLFW example programs" FORCE)
        set(GLFW_BUILD_TESTS OFF CACHE BOOL "Build the GLFW test programs" FORCE)
        set(GLFW_BUILD_DOCS OFF CACHE BOOL "Build the GLFW documentation" FORCE)
        set(GLFW_INSTALL OFF CACHE BOOL "Generate installation target" FORCE)
        # GLFW 3.4 builds X11 + Wayland by default; the viz only needs one backend, so skip
        # Wayland to avoid the wayland-scanner/protocol build dependency (X11 via xorg-dev suffices).
        set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "Build support for Wayland" FORCE)
        FetchContent_Declare(glfw
            GIT_REPOSITORY https://github.com/glfw/glfw.git
            GIT_TAG 3.4
            GIT_SHALLOW TRUE)
        FetchContent_MakeAvailable(glfw)
    endif()

    target_compile_definitions(viz PRIVATE GL_SILENCE_DEPRECATION)

    target_link_libraries(viz PRIVATE earcut_hpp common glfw OpenGL::GL)
endif()

install(
  DIRECTORY include/mapbox
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS earcut_hpp EXPORT earcut_hpp-config)

# Since there is two projects, we need to export into the parent directory
export(
  TARGETS earcut_hpp
  NAMESPACE earcut_hpp::
  FILE "${PROJECT_BINARY_DIR}/earcut_hpp-config.cmake"
)

install(EXPORT earcut_hpp-config
  DESTINATION "${CMAKE_INSTALL_DATADIR}/cmake/earcut_hpp"
  NAMESPACE earcut_hpp::
)
