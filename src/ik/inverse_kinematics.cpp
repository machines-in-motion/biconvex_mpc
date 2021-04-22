#include "inverse_kinematics.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>


namespace ik{

    InverseKinematics::InverseKinematics(std::string rmodel_path, double dt, double T)
    {

        pinocchio::urdf::buildModel(rmodel_path,rmodel_);
        pinocchio::Data rdata_(rmodel_);

    }
}