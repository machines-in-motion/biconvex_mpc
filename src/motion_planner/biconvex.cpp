# include "biconvex.hpp"


namespace motion_planner{

    BiConvexMP::BiConvexMP(double m, double dt, double T, int n_eff):
        m_(m), centroidal_dynamics(m, dt, T, n_eff), 
        prob_data_x(9*(int (T/dt)+1)), prob_data_f(3*n_eff*int (T/dt)), 
        fista_x(), fista_f(){

            dyn_violation.resize(9*(int (T/dt)+1)); dyn_violation.setZero();
            P_k_.resize(9*(int (T/dt)+1)); P_k_.setZero();

            // setting starting line search params
            fista_x.set_l0(2.25e6);
            fista_f.set_l0(506.25);

    };

    void BiConvexMP::optimize(Eigen::VectorXd x_init, int no_iters){
        // updating x_init

        centroidal_dynamics.update_x_init(x_init);
        for (unsigned i = 0; i < no_iters; ++i){
            maxit = init_maxit/(int(i)/10 + 1);
            // optimizing for F
            // std::cout << "optimizing F..." << std::endl;
            centroidal_dynamics.compute_x_mat(prob_data_x.x_k);
            prob_data_f.set_data(centroidal_dynamics.A_x, centroidal_dynamics.b_x, P_k_, rho_);
            fista_f.optimize(prob_data_f, maxit, tol);

            // optimizing for X
            // std::cout << "optimizing X..." << std::endl;
            centroidal_dynamics.compute_f_mat(prob_data_f.x_k);
            prob_data_x.set_data(centroidal_dynamics.A_f, centroidal_dynamics.b_f, P_k_, rho_);
            fista_x.optimize(prob_data_x, maxit, tol);

            dyn_violation = centroidal_dynamics.A_f * prob_data_x.x_k - centroidal_dynamics.b_f;
            P_k_ += dyn_violation;

            if (dyn_violation.norm() < exit_tol){
                std::cout << "breaking outerloop due to norm ..." << std::endl;
                break;
            };       
        }        
    }    

};
