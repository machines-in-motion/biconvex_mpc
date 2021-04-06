cmake_minimum_required(VERSION 3.10.2)

project(biconvex_mpc VERSION 0.0.1)

# Set C++ 17 as standard 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# prefix path to look for pybind11
list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib/python3.6/dist-packages/")

#Configure packages
find_package(pybind11 CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

# TODO: Create individual directories for each sub-module (i.e. optimizer, etc.)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/solvers/
)

add_library(${PROJECT_NAME}
    src/solvers/fista.cpp
)

add_executable(test_fista src/solvers/test_fista.cpp)
add_dependencies(test_fista ${PROJECT_NAME})
target_link_libraries(test_fista PUBLIC ${PROJECT_NAME})

install(TARGETS test_fista DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})

pybind11_add_module(${PROJECT_NAME}_cpp MODULE
  srcpy/solvers/fista.cpp
  )

# # cmake-format: on
# target_link_libraries(${PROJECT_NAME}_cpp PRIVATE pybind11::module)
# target_link_libraries(${PROJECT_NAME}_cpp PRIVATE ${PROJECT_NAME})
target_link_libraries(biconvex_mpc Eigen3::Eigen)
