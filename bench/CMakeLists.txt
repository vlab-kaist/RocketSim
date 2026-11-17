# Native benchmark of the simulator core (no Python).
#   cmake -S bench -B build-bench -DCMAKE_BUILD_TYPE=Release && cmake --build build-bench -j
cmake_minimum_required(VERSION 3.26)
project(RocketSimBench CXX)

set(ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
# Same as the library build: results don't depend on -march
add_compile_options(-ffp-contract=off)
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

find_package(OpenMP REQUIRED)
add_executable(bench_par bench_par.cpp ${BULLET_SOURCES} ${SIM_SOURCES})
target_compile_features(bench_par PRIVATE cxx_std_20)
target_compile_definitions(bench_par PRIVATE RS_DONT_LOG=1)
target_include_directories(bench_par PRIVATE ${ROOT}/libsrc ${ROOT}/src)
set_target_properties(bench_par PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
target_link_libraries(bench_par PRIVATE OpenMP::OpenMP_CXX)
if(BENCH_NATIVE)
	target_compile_options(bench_par PRIVATE -march=native)
endif()

add_executable(clone_test clone_test.cpp ${BULLET_SOURCES} ${SIM_SOURCES})
target_compile_features(clone_test PRIVATE cxx_std_20)
target_compile_definitions(clone_test PRIVATE RS_DONT_LOG=1)
target_include_directories(clone_test PRIVATE ${ROOT}/libsrc ${ROOT}/src)

add_executable(snapshot_test snapshot_test.cpp ${BULLET_SOURCES} ${SIM_SOURCES})
target_compile_features(snapshot_test PRIVATE cxx_std_20)
target_compile_definitions(snapshot_test PRIVATE RS_DONT_LOG=1)
target_include_directories(snapshot_test PRIVATE ${ROOT}/libsrc ${ROOT}/src)
