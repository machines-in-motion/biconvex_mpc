// This file contains python bindings for fista

#include <fista.hpp>

#include <pybind11/pybind11.h>
#include <eigen3/Eigen/Dense>

using namespace solvers;
using namespace function;
namespace py = pybind11;

PYBIND11_MODULE(fista_py, m)
{
    m.doc() = "FISTA (first order shrinkage method)";

    //FISTA
    py::class_<FISTA> fista(m, "Fista instance");
    fista.def(py::init<double,double,double>());
    fista.def("optimize", &solvers::FISTA::optimize);

    //Problem
    py::class_<ProblemData> problem_data(m, "Problem Data");
    problem_data.def(py::init<Eigen::MatrixXd, Eigen::VectorXd, Eigen::MatrixXd, Eigen::VectorXd>());
    problem_data.def("compute_obj", &function::ProblemData::compute_obj);
}

