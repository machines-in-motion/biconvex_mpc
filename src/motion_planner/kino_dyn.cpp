#include "motion_planner/kino_dyn.hpp"
#include <chrono>

namespace motion_planner{

    KinoDynMP::KinoDynMP(std::string urdf, double m, int n_eff, int dyn_col, int ik_col):
                dyn_col_(dyn_col), ik_col_(ik_col), dyn(m, dyn_col, n_eff), ik(urdf, ik_col){
        pinocchio::urdf::buildModel(urdf,pinocchio::JointModelFreeFlyer(), rmodel_) ;
        pinocchio::Data rdata_tmp(rmodel_);
        rdata_ = rdata_tmp;

        X_init.resize(9);
        X_init.setZero();
        x0.resize(rmodel_.nq + rmodel_.nv);
        x0.setZero();

        X_wm.resize(9*(dyn_col+1));
        X_wm.setZero();
        F_wm.resize(3*n_eff*dyn_col);
        F_wm.setZero();
        P_wm.resize(9*(dyn_col+1));
        P_wm.setZero();

        dyn_com_opt.resize(dyn_col+1, 3);
        dyn_com_opt.setZero();
        dyn_mom_opt.resize(dyn_col+1, 6);
        dyn_mom_opt.setZero();

        ik_com_opt.resize(ik_col+1, 3);
        ik_com_opt.setZero();
        ik_mom_opt.resize(ik_col+1, 6);
        ik_mom_opt.setZero();

        std::cout << "Initialized Kino-Dynamic Planner" << std::endl;
    };

    void KinoDynMP::optimize(Eigen::VectorXd q, Eigen::VectorXd v, int dyn_iters, int kino_dyn_iters){
        pinocchio::computeCentroidalMomentum(rmodel_, rdata_, q, v);
        x0.head(rmodel_.nq) = q;
        x0.tail(rmodel_.nv) = v;
        set_warm_starts();

        auto dyn_start = high_resolution_clock::now();
        if (osqp_) {
            dyn.optimize_osqp(X_wm.head(9), dyn_iters);
        } else {
            dyn.optimize(X_wm.head(9), dyn_iters);
        }
        auto dyn_end = high_resolution_clock::now();
        auto dyn_duration = duration_cast<std::chrono::microseconds>(dyn_end - dyn_start);
        std::cout << "Dynamics Optimization Took: " << dyn_duration.count() << " microseconds" << std::endl;

        //dyn.optimize(X_wm.head(9), dyn_iters);
        dyn_com_opt = dyn.return_opt_com();
        dyn_mom_opt = dyn.return_opt_mom();

        ik.add_centroidal_momentum_tracking_task(0, ik_col_, dyn_mom_opt.topRows(ik_col_), wt_mom_, "mom_track", false);
        ik.add_centroidal_momentum_tracking_task(0, ik_col_, dyn_mom_opt.row(ik_col_), wt_mom_, "mom_track", true);

        ik.add_com_position_tracking_task(0, ik_col_, dyn_com_opt.topRows(ik_col_), wt_com_, "com_track", false);
        ik.add_com_position_tracking_task(0, ik_col_, dyn_com_opt.row(ik_col_), wt_com_, "com_track", true);

        auto kin_start = high_resolution_clock::now();
        ik.optimize(x0);
        auto kin_end = high_resolution_clock::now();
        auto kin_duration = duration_cast<std::chrono::microseconds>(kin_end - kin_start);
        std::cout << "Kinematics Optimization Took: " << kin_duration.count() << " microseconds" << std::endl;
        // Todo: add kino dyn iteration requirement later
        // ik.compute_optimal_com_and_mom(ik_com_opt, ik_mom_opt);

        n++;
    }

    void KinoDynMP::set_warm_starts(){
        for (unsigned i = 0; i < X_wm.size()/9; ++i){
            X_wm[9*i] = rdata_.com[0][0];
            X_wm[9*i+1] = rdata_.com[0][1];
            X_wm[9*i+2] = rdata_.com[0][2];
            X_wm[9*i+3] = rdata_.vcom[0][0];
            X_wm[9*i+4] = rdata_.vcom[0][1];
            X_wm[9*i+5] = rdata_.vcom[0][2];

            X_wm[9*i+6] = rdata_.hg.toVector()[3];
            X_wm[9*i+7] = rdata_.hg.toVector()[4];
            X_wm[9*i+8] = rdata_.hg.toVector()[5];
        }
        dyn.set_warm_start_vars(X_wm, F_wm, P_wm);
    }


}