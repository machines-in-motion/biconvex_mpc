cmake_minimum_required(VERSION 3.10.2)

project(biconvex_mpc VERSION 0.0.1)

# Set C++ 17 as standard 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib/python3.6/dist-packages/")
set (python_install_dir "/usr/local/lib/python3.6/dist-packages/")

##Configure packages
find_package(pybind11 CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

# robot packages
find_package(pinocchio REQUIRED)
find_package(crocoddyl REQUIRED)

# for benchmarking
if ( OSQP )
    find_package(OsqpEigen REQUIRED)
endif()

# TODO: Create individual directories for each sub-module (i.e. optimizer, etc.)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/solvers/
  ${CMAKE_CURRENT_SOURCE_DIR}/include/motion_planner/
  ${CMAKE_CURRENT_SOURCE_DIR}/include/dynamics/
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ik/

)

add_library(${PROJECT_NAME} SHARED
    src/solvers/fista.cpp
    src/solvers/problem.cpp
    src/motion_planner/biconvex.cpp
    src/dynamics/centroidal.cpp

    src/ik/inverse_kinematics.cpp
    src/ik/action_model.cpp
    src/ik/end_effector_tasks.cpp
    src/ik/com_tasks.cpp
    src/ik/regularization_costs.cpp

    )

target_link_libraries(biconvex_mpc Eigen3::Eigen)
if ( OsqpEigen_FOUND )
    target_link_libraries(biconvex_mpc OsqpEigen::OsqpEigen)
    target_compile_definitions(biconvex_mpc PUBLIC USE_OSQP="True")
endif()

target_link_libraries(biconvex_mpc pinocchio::pinocchio)
target_link_libraries(biconvex_mpc crocoddyl::crocoddyl)

#Pybind setup
pybind11_add_module(biconvex_mpc_cpp MODULE srcpy/motion_planner/biconvex.cpp)
target_include_directories(biconvex_mpc_cpp PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(biconvex_mpc_cpp PRIVATE pybind11::module)
target_link_libraries(biconvex_mpc_cpp PRIVATE biconvex_mpc)

pybind11_add_module(inverse_kinematics_cpp MODULE srcpy/ik/inverse_kinematics.cpp)
target_link_libraries(inverse_kinematics_cpp PRIVATE biconvex_mpc)

#Install Targets

#Adding some test function for c++ testing
add_executable(test_ik src/test_ik.cpp)
add_dependencies(test_ik biconvex_mpc)
target_link_libraries(test_ik PUBLIC biconvex_mpc)
install(TARGETS test_ik DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})

#Install PyBind library
install(TARGETS biconvex_mpc_cpp LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
install(TARGETS inverse_kinematics_cpp LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
