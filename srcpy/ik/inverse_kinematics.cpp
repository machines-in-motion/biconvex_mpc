// This file contains python bindings for inverse kinematics


#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <inverse_kinematics.hpp>

using namespace ik;
namespace py = pybind11;



PYBIND11_MODULE(inverse_kinematics_cpp, m)
{
    m.doc() = "Inverse Kinematics based on DDP";

    py::class_<ik::InverseKinematics> ik (m, "InverseKinematics");
    ik.def(py::init<std::string, double, double>());
    ik.def("setup_costs", &ik::InverseKinematics::setup_costs);
    ik.def("optimize", &ik::InverseKinematics::optimize);
    ik.def("get_xs", &ik::InverseKinematics::get_xs);

    // cost
    ik.def("add_position_tracking_task", &ik::InverseKinematics::add_position_tracking_task);
    ik.def("add_velocity_tracking_task", &ik::InverseKinematics::add_velocity_tracking_task);
    ik.def("add_com_position_tracking_task", &ik::InverseKinematics::add_com_position_tracking_task);
    ik.def("add_centroidal_momentum_tracking_task", &ik::InverseKinematics::add_centroidal_momentum_tracking_task);

};

