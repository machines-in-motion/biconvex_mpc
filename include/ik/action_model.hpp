// This file contains the kinematics action model and data for DDP based IK
// Author : Avadesh Meduri
// Date : 21/04/2021

#ifndef CROCODDYL_DIFFERENTIAL_FWD_KINEMATICS
#define CROCODDYL_DIFFERENTIAL_FWD_KINEMATICS

#include "ik/template.hpp"

#include "crocoddyl/core/diff-action-base.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/core/actuation-base.hpp"
#include "crocoddyl/multibody/data/multibody.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "crocoddyl/core/utils/exception.hpp"

#include <crocoddyl/multibody/actions/free-fwddyn.hpp>

namespace crocoddyl{

    template <typename _Scalar>

    class DifferentialFwdKinematicsModelTpl : public DifferentialActionModelAbstractTpl<_Scalar> {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        public:
            typedef _Scalar Scalar;
            typedef DifferentialActionModelAbstractTpl<Scalar> Base;
            // hack
            typedef DifferentialActionDataFreeFwdDynamicsTpl<Scalar> Data;
            // typedef DifferentialFwdKinematicsDataTpl<Scalar> Data;
            typedef MathBaseTpl<Scalar> MathBase;
            typedef CostModelSumTpl<Scalar> CostModelSum;
            typedef StateMultibodyTpl<Scalar> StateMultibody;
            typedef ActuationModelAbstractTpl<Scalar> ActuationModelAbstract;
            typedef DifferentialActionDataAbstractTpl<Scalar> DifferentialActionDataAbstract;
            typedef typename MathBase::VectorXs VectorXs;
            typedef typename MathBase::MatrixXs MatrixXs;


            DifferentialFwdKinematicsModelTpl(boost::shared_ptr<StateMultibody> state,
                                      boost::shared_ptr<ActuationModelAbstract> actuation,
                                      boost::shared_ptr<CostModelSum> costs);
            
            virtual ~DifferentialFwdKinematicsModelTpl();

            virtual void calc(const boost::shared_ptr<DifferentialActionDataAbstract>& data, 
                                const Eigen::Ref<const VectorXs>& x,
                                const Eigen::Ref<const VectorXs>& u);

            virtual void calcDiff(const boost::shared_ptr<DifferentialActionDataAbstract>& data,
                                    const Eigen::Ref<const VectorXs>& x, 
                                    const Eigen::Ref<const VectorXs>& u);

            virtual boost::shared_ptr<DifferentialActionDataAbstract> createData();

            const boost::shared_ptr<ActuationModelAbstract>& get_actuation() const;
            const boost::shared_ptr<CostModelSum>& get_costs() const;
            pinocchio::ModelTpl<Scalar>& get_pinocchio() const;

        protected:
            using Base::has_control_limits_;  //!< Indicates whether any of the control limits
            using Base::nr_;                  //!< Dimension of the cost residual
            using Base::nu_;                  //!< Control dimension
            using Base::state_;               //!< Model of the state
            using Base::u_lb_;                //!< Lower control limits
            using Base::u_ub_;                //!< Upper control limits
            using Base::unone_;               //!< Neutral state

            MatrixXs A_lin;
            MatrixXs B_lin;


        private:
            boost::shared_ptr<ActuationModelAbstract> actuation_;
            boost::shared_ptr<CostModelSum> costs_;
            pinocchio::ModelTpl<Scalar>& pinocchio_;


    };

    template <typename _Scalar>
    struct DifferentialFwdKinematicsDataTpl : public DifferentialActionDataAbstractTpl<_Scalar> {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef _Scalar Scalar;
        typedef MathBaseTpl<Scalar> MathBase;
        typedef DifferentialActionDataAbstractTpl<Scalar> Base;
        typedef typename MathBase::VectorXs VectorXs;
        typedef typename MathBase::MatrixXs MatrixXs;
    
        template <template <typename Scalar> class Model>
        explicit DifferentialFwdKinematicsDataTpl(Model<Scalar>* const model)
            : Base(model),
                pinocchio(pinocchio::DataTpl<Scalar>(model->get_pinocchio())),
                multibody(&pinocchio, model->get_actuation()->createData()),
                costs(model->get_costs()->createData(&multibody)),
                // could cause an error
                A_lin(model->get_state()->get_nv(), model->get_state()->get_ndx()),
                B_lin(model->get_state()->get_nv(),model->get_state()->get_nv()){
            
            costs->shareMemory(this);
            A_lin.setZero();
            B_lin.setZero();
            for (unsigned i = 0; i < model->get_state()->get_nv(); i++){
                B_lin(i,i) = 1.0;
            }

        }

        pinocchio::DataTpl<Scalar> pinocchio;
        DataCollectorActMultibodyTpl<Scalar> multibody;
        boost::shared_ptr<CostDataSumTpl<Scalar>> costs;
        

        const MatrixXs A_lin;
        const MatrixXs B_lin;

        using Base::cost;
        using Base::Fu;
        using Base::Fx;
        using Base::Lu;
        using Base::Luu;
        using Base::Lx;
        using Base::Lxu;
        using Base::Lxx;
        using Base::r;
        using Base::xout;

    };
}
// #include "action_model.cpp"

#endif
