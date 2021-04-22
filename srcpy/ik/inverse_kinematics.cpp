// This file contains python bindings for inverse kinematics


#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include <inverse_kinematics.hpp>

using namespace ik;
namespace py = pybind11;



PYBIND11_MODULE(inverse_kinematics_cpp, m)
{
    m.doc() = "Inverse Kinematics based on DDP";

    py::class_<ik::InverseKinematics> ik (m, "InverseKinematics");
    ik.def(py::init<std::string, double, double>());
    ik.def("tmp", &ik::InverseKinematics::tmp);


};
