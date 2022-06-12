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
    py::class_<gait_planner::QuadrupedGait> quad_gp(m, "Quad_GaitPlanner");
    quad_gp.def(py::init<double, Eigen::Vector4d, Eigen::Vector4d, double>());

    //Getters
    quad_gp.def("get_phase", py::overload_cast<double, int>(&gait_planner::QuadrupedGait::get_phase));
    quad_gp.def("get_phase", py::overload_cast<double>(&gait_planner::QuadrupedGait::get_phase));
    quad_gp.def("get_phi", py::overload_cast<double, int>(&gait_planner::QuadrupedGait::get_phi));
    quad_gp.def("get_phi", py::overload_cast<double>(&gait_planner::QuadrupedGait::get_phi));
    quad_gp.def("get_percent_in_phase", py::overload_cast<double, int>
            (&gait_planner::QuadrupedGait::get_percent_in_phase));
    quad_gp.def("get_percent_in_phase", py::overload_cast<double>
            (&gait_planner::QuadrupedGait::get_percent_in_phase));
    quad_gp.def("get_contact_phase_plan", &gait_planner::QuadrupedGait::get_contact_phase_plan);

    //Setters
    quad_gp.def("step_step_height", &gait_planner::QuadrupedGait::set_step_height);
    quad_gp.def("set_phi", &gait_planner::QuadrupedGait::set_stance_percent);

    //Constructor for biped
    py::class_<gait_planner::BipedGait> biped_gp(m, "Biped_GaitPlanner");
    biped_gp.def(py::init<double, Eigen::Vector2d, Eigen::Vector2d, double>());

    //Getters
    biped_gp.def("get_phase", py::overload_cast<double, int>(&gait_planner::BipedGait::get_phase));
    biped_gp.def("get_phase", py::overload_cast<double>(&gait_planner::BipedGait::get_phase));
    biped_gp.def("get_phi", py::overload_cast<double, int>(&gait_planner::BipedGait::get_phi));
    biped_gp.def("get_phi", py::overload_cast<double>(&gait_planner::BipedGait::get_phi));
    biped_gp.def("get_percent_in_phase", py::overload_cast<double, int>
            (&gait_planner::BipedGait::get_percent_in_phase));
    biped_gp.def("get_percent_in_phase", py::overload_cast<double>
            (&gait_planner::BipedGait::get_percent_in_phase));
    biped_gp.def("get_contact_phase_plan", &gait_planner::BipedGait::get_contact_phase_plan);

    //Setters
    biped_gp.def("set_step_height", &gait_planner::BipedGait::set_step_height);
    biped_gp.def("set_phi", &gait_planner::BipedGait::set_stance_percent);
}; //PYBIND11_MODULE

