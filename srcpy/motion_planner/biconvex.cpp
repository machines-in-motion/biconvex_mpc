// This file contains python bindings for biconvex motion planner

#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include <motion_planner/biconvex.hpp>
#include <motion_planner/kino_dyn.hpp>

using namespace motion_planner;
using namespace dynamics;
namespace py = pybind11;

PYBIND11_MODULE(biconvex_mpc_cpp, m)
{
    m.doc() = "Biconvex motion planner";

    py::class_<motion_planner::BiConvexMP> mp (m, "BiconvexMP");
    mp.def(py::init<double, int, int>());
    mp.def("set_contact_plan", &motion_planner::BiConvexMP::set_contact_plan);
    mp.def("set_rotation_matrix_f", &motion_planner::BiConvexMP::set_rotation_matrix_f);
    mp.def("return_A_x", &motion_planner::BiConvexMP::return_A_x);
    mp.def("return_b_x", &motion_planner::BiConvexMP::return_b_x);
    mp.def("return_A_f", &motion_planner::BiConvexMP::return_A_f);
    mp.def("return_b_f", &motion_planner::BiConvexMP::return_b_f);
    mp.def("set_cost_x", &motion_planner::BiConvexMP::set_cost_x);
    mp.def("create_cost_X", &motion_planner::BiConvexMP::create_cost_X);
    mp.def("set_cost_f", &motion_planner::BiConvexMP::set_cost_f);
    mp.def("create_cost_F", &motion_planner::BiConvexMP::create_cost_F);
    mp.def("set_bounds_x", &motion_planner::BiConvexMP::set_bounds_x);
    mp.def("set_bounds_f", &motion_planner::BiConvexMP::set_bounds_f);
    mp.def("create_bound_constraints", &motion_planner::BiConvexMP::create_bound_constraints);
    mp.def("set_rho", &motion_planner::BiConvexMP::set_rho);
    mp.def("return_opt_x", &motion_planner::BiConvexMP::return_opt_x);
    mp.def("return_opt_f", &motion_planner::BiConvexMP::return_opt_f);
    mp.def("return_opt_p", &motion_planner::BiConvexMP::return_opt_p);
    mp.def("return_opt_com", &motion_planner::BiConvexMP::return_opt_com);
    mp.def("return_opt_mom", &motion_planner::BiConvexMP::return_opt_mom);

    mp.def("set_warm_start_vars", &motion_planner::BiConvexMP::set_warm_start_vars);
    mp.def("optimize", &motion_planner::BiConvexMP::optimize);
    mp.def("return_dyn_viol_hist", &motion_planner::BiConvexMP::return_dyn_viol_hist);
    mp.def("collect_statistics", &motion_planner::BiConvexMP::collect_statistics);

    #ifdef USE_OSQP
        mp.def("optimize_osqp", &motion_planner::BiConvexMP::optimize_osqp);
    #endif


    py::class_<dynamics::CentroidalDynamics> dyn (m, "CentroidalDynamics");
    dyn.def(py::init<double, int, int>());
    // dyn.def("create_contact_array", &dynamics::CentroidalDynamics::create_contact_array);

    py::class_<motion_planner::KinoDynMP> kd (m, "KinoDynMP");
    kd.def(py::init<std::string, double, int, int, int>());
    kd.def("return_dyn", &motion_planner::KinoDynMP::return_dyn, py::return_value_policy::reference);
    kd.def("return_ik", &motion_planner::KinoDynMP::return_ik, py::return_value_policy::reference);
    kd.def("optimize", &motion_planner::KinoDynMP::optimize);
    kd.def("set_com_tracking_weight", &motion_planner::KinoDynMP::set_com_tracking_weight);
    kd.def("set_mom_tracking_weight", &motion_planner::KinoDynMP::set_mom_tracking_weight);
    kd.def("compute_solve_times", &motion_planner::KinoDynMP::compute_solve_times);
    kd.def("return_solve_times", &motion_planner::KinoDynMP::return_solve_times, py::return_value_policy::reference);




}