#include <iostream>
#include "ik/inverse_kinematics.cpp"

int main(int argc, char **argv) {

    std::string urdf = "/home/ameduri/.local/lib/python3.6/site-packages/robot_properties_solo/resources/solo12.urdf";
    double dt = 5e-2; 
    double T = 1.0;

    // std::cout << "done .." << std::endl;
    // ik::InverseKinematics kk(urdf, dt, T);
    // std::cout << "done .." << std::endl;
    // kk.setup_costs();
    // std::cout << "done .." << std::endl;

    return 0;
}