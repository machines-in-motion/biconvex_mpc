import numpy as np
import pinocchio as pin
import crocoddyl

print("solo import ")
from robot_properties_solo.config import Solo12Config


class RegularizationCosts:

    def __init__(self):
        ## This is inherited by the inversekinematics class
        ## Note :
            ## Paramteres such as self.dt etc.. are defined in the inverse kinematics
            ## class
        pass

    def add_state_regularization_cost(self, st, et, wt, cost_name, stateWeights = None):
        """
        This funtions adds regularization cost on the state
        Input:
            st : start time (sec)
            et : end time (sec)
            wt : weight of the cost
            cost_name : name of the cost
        """
        sn, en = int(st/self.dt), int(et/self.dt)
        for i in range(sn, en):
            if isinstance(stateWeights, np.ndarray) == None:
                stateWeights = np.array([0.] * 3 + [500.] * 3 + [0.01] * (self.state.nv - 6) \
                    + [10.] * 6 + [5.0] *(self.state.nv - 6))

            q0 = np.array(Solo12Config.initial_configuration)
            x0 = np.concatenate([q0, pin.utils.zero(self.state.nv)])
            xRegCost = crocoddyl.CostModelState(self.state, \
                            crocoddyl.ActivationModelWeightedQuad(stateWeights**2), \
                                            x0)

            self.rcost_model_arr[i].addCost(cost_name+str(i), xRegCost, wt)
        
    def add_ctrl_regularization_cost(self, st, et, wt, cost_name):
        """
        This funtions adds regularization cost on the control
        Input:
            st : start time (sec)
            et : end time (sec)
            wt : weight of the cost
            cost_name : name of the cost
        """
        sn, en = int(st/self.dt), int(et/self.dt)
        for i in range(sn, en):
            # uRegCost = crocoddyl.CostModelControl(self.state, self.actuation.nu)
            uRegCost = crocoddyl.CostModelControl(self.state)
            self.rcost_model_arr[i].addCost(cost_name+str(i), uRegCost, wt)