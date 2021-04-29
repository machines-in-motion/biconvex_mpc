#include "ik/action_model.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/centroidal-derivatives.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>


namespace crocoddyl{
    
    template <typename _Scalar>
    DifferentialFwdKinematicsModelTpl<_Scalar>::DifferentialFwdKinematicsModelTpl(boost::shared_ptr<StateMultibody> state,
                                   boost::shared_ptr<ActuationModelAbstract> actuation,
                                   boost::shared_ptr<CostModelSum> costs)
        :   Base(state, state->get_nv(), costs->get_nr()),
            actuation_(actuation),
            costs_(costs),
            pinocchio_(*state->get_pinocchio().get()),
            A_lin(state->get_nv(), state->get_ndx()),
            B_lin(state->get_nv(), state->get_nv())
            {
                if (costs_->get_nu() != nu_) {
                    std::cout << "Invalid argument (from action model): Costs doesn't have the same control dimension. It should be - " 
                            << nu_ << std::endl;
                }      

                A_lin.setZero();
                B_lin.setZero();

                for (unsigned i = 0; i < state->get_nv(); i++){
                    B_lin(i,i) = 1.0;
                }

    };

    template <typename Scalar>
    DifferentialFwdKinematicsModelTpl<Scalar>::~DifferentialFwdKinematicsModelTpl() {};

    template <typename Scalar>
    void DifferentialFwdKinematicsModelTpl<Scalar>::calc(const boost::shared_ptr<DifferentialActionDataAbstract>& data, 
                                const Eigen::Ref<const VectorXs>& x,
                                const Eigen::Ref<const VectorXs>& u){

        if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
            std::cout << "Dimenstion of state incorrect (from action model) " << std::endl;
        }
    
        if (static_cast<std::size_t>(u.size()) != nu_) {
            std::cout << "Dimenstion of action incorrect (from action model)" << std::endl;
        }
        
        Data* d = static_cast<Data*>(data.get());

        const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());
        const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(state_->get_nv());

        pinocchio::forwardKinematics(pinocchio_, d->pinocchio, q);
        pinocchio::updateFramePlacements(pinocchio_, d->pinocchio);
        pinocchio::centerOfMass(pinocchio_, d->pinocchio, q, v);
        pinocchio::computeCentroidalMomentum(pinocchio_, d->pinocchio, q, v);

        d->xout.noalias() = u;
        
        // Computing the cost value and residuals
        costs_->calc(d->costs, x, u);
        d->cost = d->costs->cost;
    };

    template <typename Scalar>
    void DifferentialFwdKinematicsModelTpl<Scalar>::calcDiff(const boost::shared_ptr<DifferentialActionDataAbstract>& data,
                                    const Eigen::Ref<const VectorXs>& x, 
                                    const Eigen::Ref<const VectorXs>& u){

        Data* d = static_cast<Data*>(data.get());

        const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());
        const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(state_->get_nv());

        pinocchio::computeForwardKinematicsDerivatives(pinocchio_, d->pinocchio, q, v, u);
        pinocchio::jacobianCenterOfMass(pinocchio_, d->pinocchio);
        
        // Maynot be the most effecient to call this function (must investigate)
        pinocchio::computeRNEADerivatives(pinocchio_, d->pinocchio, q, v, u);
        // pinocchio::computeCentroidalDynamicsDerivatives(pinocchio_, d->pinocchio, q, v, u);

        d->Fx.noalias() = A_lin;
        d->Fu.noalias() = B_lin;
        
        // Computing the cost derivatives
        costs_->calcDiff(d->costs, x, u);              
    };

    template <typename Scalar>
    boost::shared_ptr<DifferentialActionDataAbstractTpl<Scalar> >
    DifferentialFwdKinematicsModelTpl<Scalar>::createData() {
        return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this);
    };

    template <typename Scalar>
    pinocchio::ModelTpl<Scalar>& DifferentialFwdKinematicsModelTpl<Scalar>::get_pinocchio() const {
    return pinocchio_;
    }

    template <typename Scalar>
    const boost::shared_ptr<ActuationModelAbstractTpl<Scalar> >&
    DifferentialFwdKinematicsModelTpl<Scalar>::get_actuation() const {
    return actuation_;
    }

    template <typename Scalar>
    const boost::shared_ptr<CostModelSumTpl<Scalar> >& DifferentialFwdKinematicsModelTpl<Scalar>::get_costs()
        const {
    return costs_;
    }

}

// // template instatantiation
// typedef crocoddyl::DifferentialFwdKinematicsModelTpl<double> DifferentialFwdKinematicsModel; 
// typedef crocoddyl::DifferentialFwdKinematicsDataTpl<double> DifferentialFwdKinematicsData; 