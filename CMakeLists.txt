cmake_minimum_required(VERSION 3.10)
project(AOEmbree)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Release)

if (POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# find_package(embree REQUIRED)
# find_package(tbb REQUIRED)
find_package(OpenMP COMPONENTS C REQUIRED)


add_executable(AOEmbree AOEmbree.cpp)
# TARGET_LINK_LIBRARIES(AOEmbree embree)
# TARGET_LINK_LIBRARIES(AOEmbree tbb)
target_link_libraries(AOEmbree PRIVATE OpenMP::OpenMP_C)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_C_FLAGS_RELEASE "-O3")