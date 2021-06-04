// This file contains python bindings for the gait_planner

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>

#include <gait_planner/gait_planner.hpp>

using namespace gait_planner;
namespace py = pybind11;

PYBIND11_MODULE(gait_planner_cpp, m)
{
    m.doc() = "Gait Planner";
    //Constructor
    py::class_<gait_planner::QuadrupedGait> gait_planner(m, "GaitPlanner");
    gait_planner.def(py::init<double, Eigen::Vector4d, Eigen::Vector4d, double>());

    //Getters
    gait_planner.def("get_phase", py::overload_cast<double, int>(&gait_planner::QuadrupedGait::get_phase));
    gait_planner.def("get_phase", py::overload_cast<double>(&gait_planner::QuadrupedGait::get_phase));
    gait_planner.def("get_phi", py::overload_cast<double, int>(&gait_planner::QuadrupedGait::get_phi));
    gait_planner.def("get_phi", py::overload_cast<double>(&gait_planner::QuadrupedGait::get_phi));
    gait_planner.def("get_percent_in_phase", py::overload_cast<double, int>
            (&gait_planner::QuadrupedGait::get_percent_in_phase));
    gait_planner.def("get_percent_in_phase", py::overload_cast<double>
            (&gait_planner::QuadrupedGait::get_percent_in_phase));
    gait_planner.def("get_contact_phase_plan", &gait_planner::QuadrupedGait::get_contact_phase_plan);

    //Setters
    gait_planner.def("get_phi", &gait_planner::QuadrupedGait::set_step_height);
    gait_planner.def("get_phi", &gait_planner::QuadrupedGait::set_stance_percent);
}; //PYBIND11_MODULE

