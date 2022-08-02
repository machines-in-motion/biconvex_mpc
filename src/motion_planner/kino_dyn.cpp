#include "motion_planner/kino_dyn.hpp"

namespace motion_planner{

    KinoDynMP::KinoDynMP(std::string urdf, double m, int n_eff, int dyn_col, int ik_col):
                dyn_col_(dyn_col), ik_col_(ik_col), dyn(m, dyn_col, n_eff), ik(urdf, ik_col){

        std::cout << "Initialized Kino-Dyn planner" << std::endl;
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

        solve_times.resize(3);
        solve_times.setZero(3);

        wt_com_.resize(6);
        wt_com_.setZero(6);

        wt_mom_.resize(6);
        wt_mom_.setZero(6);
    };

    void KinoDynMP::optimize(Eigen::VectorXd q, Eigen::VectorXd v, int dyn_iters, int kino_dyn_iters){

        const auto t1 = std::chrono::steady_clock::now();    
        pinocchio::computeCentroidalMomentum(rmodel_, rdata_, q, v);
        x0.head(rmodel_.nq) = q; x0.tail(rmodel_.nv) = v;
        set_warm_starts();

        const auto t2 = std::chrono::steady_clock::now();    
        dyn.optimize(X_wm.head(9), dyn_iters);
        const auto t3 = std::chrono::steady_clock::now();    

        dyn_com_opt = dyn.return_opt_com();
        dyn_mom_opt = dyn.return_opt_mom();

        ik.add_centroidal_momentum_tracking_task(0, ik_col_, dyn_mom_opt.topRows(ik_col_), wt_mom_, "mom_track", false);
        ik.add_centroidal_momentum_tracking_task(0, ik_col_, dyn_mom_opt.row(ik_col_), wt_mom_, "mom_track_ter", true);
        ik.add_com_position_tracking_task(0, ik_col_, dyn_com_opt.topRows(ik_col_), wt_com_, "com_track", false);
        ik.add_com_position_tracking_task(0, ik_col_, dyn_com_opt.row(ik_col_), wt_com_, "com_track", true);
        const auto t4 = std::chrono::steady_clock::now();    
        ik.optimize(x0);
        const auto t5 = std::chrono::steady_clock::now();    
        // Todo: add kino dyn iteration requirement later
        // ik.compute_optimal_com_and_mom(ik_com_opt, ik_mom_opt);

        n++;

        // For profiling
        if (profile_code){
            std::chrono::duration<double> dyn_time = t3 - t2;
            std::chrono::duration<double> kin_time = t5 - t4;
            std::chrono::duration<double> total_time = t5 - t1;

            solve_times[0] = dyn_time.count();
            solve_times[1] = kin_time.count();
            solve_times[2] = total_time.count();

            std::cout << "Dyn optimize time : " << dyn_time.count() << std::endl; 
            std::cout << "Kin optimize time : " << kin_time.count() << std::endl; 
            std::cout << "Total optimize time : " << total_time.count() << std::endl; 
            std::cout << "===============================================" << std::endl;
        };

    }

    void KinoDynMP::set_warm_starts(){

        // if(n == 0){
        for (unsigned i = 0; i < X_wm.size()/9; ++i){
            X_wm[9*i] = rdata_.com[0][0];
            X_wm[9*i+1] = rdata_.com[0][1];
            X_wm[9*i+2] = rdata_.com[0][2];
            X_wm[9*i+3] = rdata_.vcom[0][0];
            X_wm[9*i+4] = rdata_.vcom[0][1];
            X_wm[9*i+5] = rdata_.vcom[0][2];

            X_wm[9*i+6] = rdata_.hg.toVector()[3]/dyn.m_;
            X_wm[9*i+7] = rdata_.hg.toVector()[4]/dyn.m_;
            X_wm[9*i+8] = rdata_.hg.toVector()[5]/dyn.m_;
        }
        // for (unsigned i = 0; i < F_wm.size()/3; ++i){
        //     F_wm[3*i] = 9.81;
        // }

        dyn.set_warm_start_vars(X_wm, F_wm, P_wm);
    }


}