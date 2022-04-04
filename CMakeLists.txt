cmake_minimum_required(VERSION 3.10)

project(biconvex_mpc VERSION 0.0.1)

# Set C++ 17 as standard 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Configure packages

add_subdirectory(extern) # for pybind11
find_package(pybind11 CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

# robot packages
find_package(pinocchio REQUIRED)
find_package(crocoddyl REQUIRED)

# for benchmarking
if ( OSQP )
    message("Building with OSQP")
    find_package(OsqpEigen REQUIRED)
endif()

# TODO: Create individual directories for each sub-module (i.e. optimizer, etc.)
# include_directories(
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

add_library(${PROJECT_NAME} SHARED
    src/solvers/fista.cpp
    src/solvers/problem.cpp
    src/motion_planner/biconvex.cpp
    src/dynamics/centroidal.cpp
    src/gait_planner/gait_planner.cpp

    src/ik/inverse_kinematics.cpp
    src/ik/action_model.cpp
    src/ik/end_effector_tasks.cpp
    src/ik/com_tasks.cpp
    src/ik/regularization_costs.cpp

    src/motion_planner/kino_dyn.cpp
    )

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME} crocoddyl::crocoddyl)

# Includes. Add the include dependencies
target_include_directories(
    ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           )

install(
TARGETS ${PROJECT_NAME}
DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})

if ( OsqpEigen_FOUND )
    target_link_libraries(${PROJECT_NAME} OsqpEigen::OsqpEigen)
    target_compile_definitions(${PROJECT_NAME} PUBLIC USE_OSQP="True")
endif()

#Pybind setup
pybind11_add_module(biconvex_mpc_cpp MODULE srcpy/motion_planner/biconvex.cpp)
target_include_directories(biconvex_mpc_cpp PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(biconvex_mpc_cpp PRIVATE pybind11::module)
target_link_libraries(biconvex_mpc_cpp PRIVATE biconvex_mpc)

pybind11_add_module(inverse_kinematics_cpp MODULE srcpy/ik/inverse_kinematics.cpp)
target_link_libraries(inverse_kinematics_cpp PRIVATE biconvex_mpc)

pybind11_add_module(gait_planner_cpp MODULE srcpy/gait_planner/py_gait_planner.cpp)
target_include_directories(gait_planner_cpp PRIVATE ${EIGEN3_INCLUDE_DIR})
#target_link_directories(gait_planner_cpp PRIVATE pybind11::module)
target_link_libraries(gait_planner_cpp PRIVATE biconvex_mpc)

#Install PyBind library
install(TARGETS biconvex_mpc_cpp DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
install(TARGETS inverse_kinematics_cpp LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
install(TARGETS gait_planner_cpp LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
