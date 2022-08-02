// This file contains python bindings for inverse kinematics


#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <ik/inverse_kinematics.hpp>

using namespace ik;
namespace py = pybind11;



PYBIND11_MODULE(inverse_kinematics_cpp, m)
{
    m.doc() = "Inverse Kinematics based on DDP";

    py::class_<ik::InverseKinematics> ik (m, "InverseKinematics");
    ik.def(py::init<std::string, int>());
    ik.def("setup_costs", &ik::InverseKinematics::setup_costs);
    ik.def("optimize", &ik::InverseKinematics::optimize);
    ik.def("get_xs", &ik::InverseKinematics::get_xs);
    ik.def("get_us", &ik::InverseKinematics::get_us);
    ik.def("return_opt_com", &ik::InverseKinematics::return_opt_com);
    ik.def("return_opt_mom", &ik::InverseKinematics::return_opt_mom);

    // cost
    ik.def("add_position_tracking_task", &ik::InverseKinematics::add_position_tracking_task);
    ik.def("add_position_tracking_task_single", py::overload_cast<pinocchio::FrameIndex, \
                            Eigen::MatrixXd, double, std::string, int>(&ik::InverseKinematics::add_position_tracking_task_single));
    ik.def("add_position_tracking_task_single", py::overload_cast<pinocchio::FrameIndex, \
                            Eigen::MatrixXd, Eigen::VectorXd, std::string, int>(&ik::InverseKinematics::add_position_tracking_task_single));
    ik.def("add_terminal_position_tracking_task",  py::overload_cast<pinocchio::FrameIndex, \
                            Eigen::MatrixXd, double, std::string>(&ik::InverseKinematics::add_terminal_position_tracking_task));
    ik.def("add_terminal_position_tracking_task",  py::overload_cast<pinocchio::FrameIndex, \
                            Eigen::MatrixXd, Eigen::VectorXd, std::string>(&ik::InverseKinematics::add_terminal_position_tracking_task));
    ik.def("add_velocity_tracking_task", &ik::InverseKinematics::add_velocity_tracking_task);
    ik.def("add_com_position_tracking_task", &ik::InverseKinematics::add_com_position_tracking_task);
    ik.def("add_centroidal_momentum_tracking_task", &ik::InverseKinematics::add_centroidal_momentum_tracking_task);
    ik.def("add_state_regularization_cost", &ik::InverseKinematics::add_state_regularization_cost);
    ik.def("add_state_regularization_cost_single", &ik::InverseKinematics::add_state_regularization_cost_single);
    ik.def("add_ctrl_regularization_cost", &ik::InverseKinematics::add_ctrl_regularization_cost);
    ik.def("add_ctrl_regularization_cost_single", &ik::InverseKinematics::add_ctrl_regularization_cost_single);

};

