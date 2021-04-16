// This file contains python bindings for biconvex motion planner


#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include <biconvex.hpp>
#include <centroidal.hpp>

using namespace motion_planner;
using namespace dynamics;
namespace py = pybind11;

PYBIND11_MODULE(biconvex_mpc_cpp, m)
{
    m.doc() = "Biconvex motion planner";

    py::class_<motion_planner::BiConvexMP> mp (m, "BiconvexMP");
    mp.def(py::init<double, double, double, int>());
    mp.def("set_contact_plan", &motion_planner::BiConvexMP::set_contact_plan); 
    mp.def("print_contact_array", &motion_planner::BiConvexMP::return_cnt_plan);
    mp.def("return_A_x", &motion_planner::BiConvexMP::return_A_x);
    mp.def("return_b_x", &motion_planner::BiConvexMP::return_b_x);
    mp.def("return_A_f", &motion_planner::BiConvexMP::return_A_f);
    mp.def("return_b_f", &motion_planner::BiConvexMP::return_b_f);
   

    py::class_<dynamics::CentroidalDynamics> dyn (m, "CentroidalDynamics");
    dyn.def(py::init<double, double, double, int>());
    dyn.def("create_contact_array", &dynamics::CentroidalDynamics::create_contact_array);

}