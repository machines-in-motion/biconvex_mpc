// This file contains python bindings for fista

#include <pybind11/pybind11.h>
#include <fista.hpp>

namespace py = pybind11;

namespace solvers 
{
void bind_fista(py::module& m){

    py::class_<FISTA>(m, "FISTA")
        // .def(py::init<>())

        .def("optimize", & FISTA::optimize, "optimizes the problem");
    }
}

