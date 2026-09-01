cmake_minimum_required(VERSION 3.20)
project(TerraNav CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Sanitizer support: -DSANITIZE=address|undefined|both (default: off)
option(SANITIZE "Enable sanitizers" OFF)
if(SANITIZE)
    # GCC sanitizer libraries live in a non-standard path
    execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libasan.so
        OUTPUT_VARIABLE GCC_LIB_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    get_filename_component(GCC_LIB_DIR "${GCC_LIB_DIR}" DIRECTORY)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L${GCC_LIB_DIR}")

    if(SANITIZE STREQUAL "address" OR SANITIZE STREQUAL "both")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer -g")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
    endif()
    if(SANITIZE STREQUAL "undefined" OR SANITIZE STREQUAL "both")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined -fno-omit-frame-pointer -g")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=undefined")
    endif()
endif()

# Default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

# raylib built from submodule
set(PLATFORM Desktop CACHE STRING "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/raylib)

# Common include paths
set(INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/third_party
)

# --- Visualization ---
add_executable(viz src/visualization/main.cpp)
target_include_directories(viz PRIVATE ${INCLUDE_DIRS})
target_link_libraries(viz PRIVATE raylib)

# --- Map Generation ---
add_executable(map_gen src/map_generation/main.cpp)
target_include_directories(map_gen PRIVATE ${INCLUDE_DIRS})

# --- Benchmark ---
add_executable(bench src/benchmark/main.cpp)
target_include_directories(bench PRIVATE ${INCLUDE_DIRS})
target_link_libraries(bench PRIVATE raylib)

# Output binaries in build/
set_target_properties(viz map_gen bench PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
