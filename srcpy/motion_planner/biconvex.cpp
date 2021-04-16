// This file contains python bindings for biconvex motion planner


#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include <biconvex.hpp>

using namespace motion_planner;
namespace py = pybind11;

PYBIND11_MODULE(biconvex_mpc_cpp, m)
{
    m.doc() = "Biconvex motion planner";

    py::class_<motion_planner::BiConvexMP> mp (m, "BiconvexMP");
    mp.def(py::init<double, double, double, int>());
    mp.def("set_contact_plan", &motion_planner::BiConvexMP::set_contact_plan);    

}