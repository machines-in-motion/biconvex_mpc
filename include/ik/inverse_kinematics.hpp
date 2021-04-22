//This file contains the implementation of the DDP based IK
// Author : Avadesh Meduri
// Date : 22/04/2021


#ifndef _INVERSE_KINEMATICS_
#define _INVERSE_KINEMATICS_

#include <iostream>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/data.hpp"

namespace ik{

    class InverseKinematics{

        public:

            InverseKinematics(std::string rmodel_path, double dt, double T);

            void tmp();

        protected:

            // robot model
            pinocchio::Model rmodel_;
            // robot data
            pinocchio::Data rdata_;
            

    };

}


#endif