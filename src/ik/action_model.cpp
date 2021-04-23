#include "action_model.hpp"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/centroidal-derivatives.hpp>

namespace crocoddyl{
    
    template <typename _Scalar>
    DifferentialFwdKinematicsModelTpl<_Scalar>::DifferentialFwdKinematicsModelTpl(boost::shared_ptr<StateMultibody> state,
                                   boost::shared_ptr<ActuationModelAbstract> actuation,
                                   boost::shared_ptr<CostModelSum> costs)
        :   Base(state, actuation->get_nu(), costs->get_nr()),
            actuation_(actuation),
            costs_(costs),
            pinocchio_(*state->get_pinocchio().get()){

                if (costs_->get_nu() != nu_) {
                    std::cout << "Invalid argument: Costs doesn't have the same control dimension. It should be - " 
                            << nu_ << std::endl;
                }                  

    };

    template <typename Scalar>
    DifferentialFwdKinematicsModelTpl<Scalar>::~DifferentialFwdKinematicsModelTpl() {};

    template <typename Scalar>
    void DifferentialFwdKinematicsModelTpl<Scalar>::calc(const boost::shared_ptr<DifferentialActionDataAbstract>& data, 
                                const Eigen::Ref<const VectorXs>& x,
                                const Eigen::Ref<const VectorXs>& u){

        if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
            std::cout << "Dimenstion of state incorrect" << std::endl;
        }
    
        if (static_cast<std::size_t>(u.size()) != nu_) {
            std::cout << "Dimenstion of action incorrect" << std::endl;
        }
        
        Data* d = static_cast<Data*>(data.get());

        const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());
        const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(state_->get_nv());

        pinocchio::forwardKinematics(pinocchio_, d->pinocchio, q);
        pinocchio::updateFramePlacements(pinocchio_, d->pinocchio, q);
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
        pinocchio::computeCentroidalDynamicsDerivatives(pinocchio_, d->pinocchio, q, v, u);

        d->Fx.noalias() = d->A_lin;
        d->Fu.noalias() = d->B_lin;
        
        // Computing the cost derivatives
        costs_->calcDiff(d->costs, x, u);              
    };

    template <typename Scalar>
    boost::shared_ptr<DifferentialActionDataAbstractTpl<Scalar> >
    DifferentialFwdKinematicsModelTpl<Scalar>::createData() {
        return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this);
    };
}