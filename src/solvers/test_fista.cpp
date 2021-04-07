#include "problem.hpp"
#include "fista.hpp"
#include <chrono>

#include <iostream>

int main(int argc, char **argv)
{
    std::cout << "testing FISTA creation" << std::endl;
    auto testFista = std::make_shared<solvers::FISTA>(1.0, 1.0, 1e-5);
    return 0;
}