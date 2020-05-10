cmake_minimum_required(VERSION 3.5)
project(AOEmbree)
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE Release)


if (POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

find_package(embree 3.0 REQUIRED)

INCLUDE_DIRECTORIES(${EMBREE_INCLUDE_DIRS})

add_executable(AOEmbree AOEmbree.cpp)

set(CMAKE_CXX_FLAGS_RELEASE "-O2")

target_compile_definitions(AOEmbree PUBLIC NOMINMAX)
TARGET_LINK_LIBRARIES(AOEmbree ${EMBREE_LIBRARY})


