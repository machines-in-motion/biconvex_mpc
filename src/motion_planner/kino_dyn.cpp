#include "motion_planner/kino_dyn.hpp"


namespace motion_planner{

    KinoDynMP::KinoDynMP(std::string urdf, int n_eff, int dyn_col, int ik_col):
                dyn(0, dyn_col, n_eff), ik(urdf, ik_col){

        std::cout << "Initialized Kino-Dyn planner" << std::endl;
        pinocchio::urdf::buildModel(urdf,pinocchio::JointModelFreeFlyer(), rmodel_) ;
        pinocchio::Data rdata_tmp(rmodel_);
        rdata_ = rdata_tmp;

        // make sure this is not causing a blow up
        m_ = pinocchio::computeTotalMass(rmodel_);
        dyn.set_robot_mass(m_);

        X_init.resize(9); X_init.setZero();
        X_wm.resize(9*(dyn_col+1)); X_wm.setZero();
        F_wm.resize(3*n_eff*dyn_col); F_wm.setZero();
        P_wm.resize(9*(dyn_col+1)); P_wm.setZero();
        
    };

    void KinoDynMP::optimize(Eigen::VectorXd q, Eigen::VectorXd v, int dyn_iters, int kino_dyn_iters){

        pinocchio::computeCentroidalMomentum(rmodel_, rdata_q, v);
        for (unsigned i = 0; i < X_wm.size()/9; ++i){
            X_wm[9*i] = rdata_.com[0][0];
            X_wm[9*i+1] = rdata_.com[0][1];
            X_wm[9*i+2] = rdata_.com[0][2];
            X_wm[9*i+3] = rdata_.vcom[0][0];
            X_wm[9*i+4] = rdata_.vcom[0][1];
            X_wm[9*i+5] = rdata_.vcom[0][2];
            // check this issue
            X_wm[9*i+6] = rdata_.hg[3];
            X_wm[9*i+7] = rdata_.hg[4];
            X_wm[9*i+8] = rdata_.hg[5];
        }

        dyn.set_warm_start_vars(X_wm, F_wm, P_wm);
        dyn.optimize(X_wm.head(9), dyn_iters);

    }


}