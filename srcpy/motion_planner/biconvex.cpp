// This file contains python bindings for biconvex motion planner


#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include <motion_planner/biconvex.hpp>

using namespace motion_planner;
using namespace dynamics;
namespace py = pybind11;

PYBIND11_MODULE(biconvex_mpc_cpp, m)
{
    m.doc() = "Biconvex motion planner";

    py::class_<motion_planner::BiConvexMP> mp (m, "BiconvexMP");
    mp.def(py::init<double, double, double, int>());
    mp.def("set_contact_plan", &motion_planner::BiConvexMP::set_contact_plan);
    mp.def("set_contact_plan_2", &motion_planner::BiConvexMP::set_contact_plan_2);
    mp.def("create_contact_array", &motion_planner::BiConvexMP::create_cnt_array);
    mp.def("create_contact_array_2", &motion_planner::BiConvexMP::create_cnt_array_2);
    mp.def("return_A_x", &motion_planner::BiConvexMP::return_A_x);
    mp.def("return_b_x", &motion_planner::BiConvexMP::return_b_x);
    mp.def("return_A_f", &motion_planner::BiConvexMP::return_A_f);
    mp.def("return_b_f", &motion_planner::BiConvexMP::return_b_f);
    mp.def("set_cost_x", &motion_planner::BiConvexMP::set_cost_x);
    mp.def("set_cost_f", &motion_planner::BiConvexMP::set_cost_f);
    mp.def("set_bounds_x", &motion_planner::BiConvexMP::set_bounds_x);
    mp.def("set_bounds_f", &motion_planner::BiConvexMP::set_bounds_f);
    mp.def("return_opt_x", &motion_planner::BiConvexMP::return_opt_x);
    mp.def("return_opt_f", &motion_planner::BiConvexMP::return_opt_f);
    mp.def("return_opt_p", &motion_planner::BiConvexMP::return_opt_p);

    mp.def("set_warm_start_vars", &motion_planner::BiConvexMP::set_warm_start_vars);
    mp.def("optimize", &motion_planner::BiConvexMP::optimize);

    py::class_<dynamics::CentroidalDynamics> dyn (m, "CentroidalDynamics");
    dyn.def(py::init<double, double, double, int>());
    dyn.def("create_contact_array", &dynamics::CentroidalDynamics::create_contact_array);
    dyn.def("create_contact_array_2", &dynamics::CentroidalDynamics::create_contact_array_2);

}