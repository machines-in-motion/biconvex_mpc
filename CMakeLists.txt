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
)

add_library(${PROJECT_NAME} SHARED
    src/solvers/fista.cpp
    src/solvers/problem.cpp
)

add_executable(test_fista src/solvers/test_fista.cpp)
add_dependencies(test_fista ${PROJECT_NAME})
target_link_libraries(test_fista PUBLIC ${PROJECT_NAME})
target_link_libraries(biconvex_mpc Eigen3::Eigen)

#Pybind setup
pybind11_add_module(fista_py MODULE srcpy/solvers/fista.cpp)
target_include_directories(fista_py PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(fista_py PRIVATE pybind11::module)
target_link_libraries(fista_py PRIVATE biconvex_mpc)

#Install Targets

#Install PyBind library
install(TARGETS fista_py LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
# install(TARGETS fista_py LIBRARY DESTINATION ${python_install_dir})

#Install cpp binary
install(TARGETS test_fista DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})

#install(TARGETS fista_py
#        DIRECTORY python/${PROJECT_NAME}
#        PATTERN "*.pyc" EXCLUDE
#        PATTERN "__pycache__" EXCLUDE
#)

# # cmake-format: on
#target_link_libraries(${PROJECT_NAME}_cpp PRIVATE ${PROJECT_NAME})
#target_link_libraries(${PROJECT_NAME}_cpp PRIVATE pybind11::module)
#install(TARGETS ${PROJECT_NAME}_cpp DESTINATION $python_path)

