cmake_minimum_required(VERSION 3.10.2)

project(biconvex_mpc VERSION 0.0.1)

# Set C++ 17 as standard 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib/python3.6/dist-packages/")
set (python_install_dir "/usr/local/lib/python3.6/dist-packages/")

#Configure packages
find_package(pybind11 CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

# TODO: Create individual directories for each sub-module (i.e. optimizer, etc.)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/solvers/
  ${CMAKE_CURRENT_SOURCE_DIR}/include/motion_planner/
  ${CMAKE_CURRENT_SOURCE_DIR}/include/dynamics/

)

add_library(${PROJECT_NAME} SHARED
    src/solvers/fista.cpp
    src/solvers/problem.cpp
    src/motion_planner/biconvex.cpp
    src/dynamics/centroidal.cpp
)

# add_executable(test_fista src/solvers/test_fista.cpp)
# add_dependencies(test_fista ${PROJECT_NAME})
# target_link_libraries(test_fista PUBLIC ${PROJECT_NAME})
target_link_libraries(biconvex_mpc Eigen3::Eigen)

#Pybind setup
# pybind11_add_module(fista_py MODULE srcpy/solvers/fista.cpp)
# target_include_directories(fista_py PRIVATE ${EIGEN3_INCLUDE_DIR})
# target_link_libraries(fista_py PRIVATE pybind11::module)
# target_link_libraries(fista_py PRIVATE biconvex_mpc)

pybind11_add_module(biconvex_mpc_cpp MODULE srcpy/motion_planner/biconvex.cpp)
target_include_directories(biconvex_mpc_cpp PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(biconvex_mpc_cpp PRIVATE pybind11::module)
target_link_libraries(biconvex_mpc_cpp PRIVATE biconvex_mpc)


#Install Targets

#Install PyBind library
# install(TARGETS fista_py LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
install(TARGETS biconvex_mpc_cpp LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})

# install(TARGETS fista_py LIBRARY DESTINATION ${python_install_dir})

