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
    fista.def("optimize", &solvers::FISTA::optimize);
    fista.def("set_data", &solvers::FISTA::set_data);
    fista.def("set_l0", &solvers::FISTA::set_l0);

    //Problem
     py::class_<ProblemData> problem_data(m, "data");
     problem_data.def(py::init<Eigen::MatrixXd, Eigen::VectorXd, Eigen::MatrixXd, Eigen::VectorXd, 
                                Eigen::VectorXd, int, double>());
     problem_data.def("compute_obj", &function::ProblemData::compute_obj);
     problem_data.def("compute_grad", &function::ProblemData::compute_grad_obj);
     problem_data.def("compute_proj", &function::ProblemData::proj);
     problem_data.def("set_bounds", &function::ProblemData::set_bounds);

}

