// Containst the centroidal dynamics class
// Author : Avadesh Meduri & Paarth Shah
// Date : 15/04/2021

#ifndef CENTROIDAL_HPP
#define CENTROIDAL_HPP
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

namespace dynamics{

    class CentroidalDynamics{

        public:
            CentroidalDynamics(double m, int n_col, int n_eff);

            //Computes A(X) which is used when optimizing for forces
            void compute_x_mat(Eigen::VectorXd &X);

            //Computes A(F) which is used when optimizing for states
            void compute_f_mat(Eigen::VectorXd &F);

            //Updates the size of A(F) and b(F)
            void resize_f_mat(int num_states);

            //Updating X
            //TODO: Only do this one not setting x_init explicitly with equality constraints
            //TODO: Do not update A_f every time
            void update_x_init(Eigen::VectorXd &x_init);

            // Set Contact Matrix which includes positions, etc.
            void set_contact_arrays(Eigen::MatrixXd cnt_plan, double dt);

            //Update the binary contact array
            void update_contact_array();

            Eigen::SparseMatrix<double> A_x;
            Eigen::VectorXd b_x;
            Eigen::SparseMatrix<double> A_f;
            Eigen::VectorXd b_f;
            Eigen::VectorXd x_init_;
        
            // location of the contact point array used to create constraints and for calculating forces/amom
            // Dimension: n_col_ x n_eff x 3
            std::vector<Eigen::MatrixXd> r_;

            // contact array that is used to create the constraints (tells if end effector is in contact)
            // Dimension: n_col_ x n_eff
            Eigen::MatrixXd cnt_arr_;

            // location of contact point at time t
            Eigen::MatrixXd r_t;

        private:

            //Mass of total robot
            const double m_;

            //Number of collocation points
            int n_col_;

            //Number of end-effectors
            const double n_eff_;

            //Vector of dt_'s for variable dt_
            Eigen::VectorXd dt_;

            //Boolean of whether or not footsteps are variables
            bool variable_footsteps_ = false;
    };
}
#endif