# Native benchmark of the simulator core (no Python).
#   cmake -S bench -B build-bench -DCMAKE_BUILD_TYPE=Release && cmake --build build-bench -j
cmake_minimum_required(VERSION 3.26)
project(RocketSimBench CXX)

set(ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
file(GLOB_RECURSE BULLET_SOURCES ${ROOT}/libsrc/bullet3-3.24/*.cpp)
file(GLOB_RECURSE SIM_SOURCES ${ROOT}/src/*.cpp)

add_executable(bench bench.cpp ${BULLET_SOURCES} ${SIM_SOURCES})
target_compile_definitions(bench PRIVATE RS_DONT_LOG=1)
target_compile_features(bench PRIVATE cxx_std_20)
target_include_directories(bench PRIVATE ${ROOT}/libsrc ${ROOT}/src)
set_target_properties(bench PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
option(BENCH_NATIVE "build with -march=native" ON)
if(BENCH_NATIVE)
	target_compile_options(bench PRIVATE -march=native)
endif()
