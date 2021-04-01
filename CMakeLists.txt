cmake_minimum_required(VERSION 3.10.2)

project(biconvex_mpc VERSION 0.0.1)

# Set C++ 17 as standard 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#Configure packages
find_package(pybind11 CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

# TODO: Create individual directories for each sub-module (i.e. optimizer, etc.)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME}
    src/fista.cpp
    src/problem.cpp
)

add_executable(test_fista src/test_fista.cpp)
add_dependencies(test_fista ${PROJECT_NAME})
target_link_libraries(test_fista PUBLIC ${PROJECT_NAME})

install(TARGETS test_fista DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(biconvex_mpc Eigen3::Eigen)