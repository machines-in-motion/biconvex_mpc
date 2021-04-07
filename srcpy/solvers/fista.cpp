// This file contains python bindings for fista
#include <fista.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>


using namespace solvers;
using namespace function;
namespace py = pybind11;

PYBIND11_MODULE(fista_py, m)
{
    m.doc() = "FISTA (first order shrinkage method)";

    //FISTA
    py::class_<solvers::FISTA> fista(m, "instance");
    fista.def(py::init<double, double, int, double>());
    fista.def("optimize", &solvers::FISTA::optimize_pybind);
    fista.def("set_data", &solvers::FISTA::set_data);
    //fista.def("compute_obj", &solvers::FISTA::compute_obj);

    //Problem
     py::class_<ProblemData> problem_data(m, "data");
     problem_data.def(py::init<Eigen::MatrixXd, Eigen::VectorXd, Eigen::MatrixXd, Eigen::VectorXd>());
     problem_data.def("compute_obj", &function::ProblemData::compute_obj);
}

