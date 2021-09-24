 import pinocchio as pin

 from gait_planner_cpp import GaitPlanner

 class ContactPlanner:

    def __init__(self, acyclic_plan, robot_info, height_map = None):
        self.gait_period = gait_params.gait_period
        self.stance_percent = gait_params.stance_percent
        self.nominal_height = gait_params.nom_ht
        self.phase_offset = gait_params.phase_offset
        self.step_height = gait_params.step_height

        self.foot_size = robot_info.foot_size
        self.gravity = 9.81

        #Read this in from somewhere
        #TODO: 
        self.dt = 0.005
        
        #Create instance of continuous gait planner
        self.acyclic_contact_plan = acyclic_plan
    def create_cnt_plan(self, q, v, t):
            """
            Creates Contact Plan Matrix:
            Dimensions: horizon (number of collocation points) x num_eef x 9
            Extra Information: The final dimension (9) gives the contact boolean
                               i.e. the last vector should be [1/0, pos, orientation (quaternion), dt] 
                               where 1/0 gives a boolean for contact (1 = contact, 0 = no cnt)
            """
            cnt_plan = np.zeros((self.horizon, len(self.eff_names), 9))
            evaluated_time = t

            #TODO: Read this from a height-map?
            temp_quat = np.array([0,0,0,1])
            
            for i in range(self.horizon):
                evaluated_time += i*self.dt

                for j in range(len(self.eff_names)):
                    for k in range(self.acyclic_contact_plan[j].shape[0]):
                        if k == 0:
                            if self.evaluated_time <= self.acyclic_contact_plan[j][0][3] \
                                and self.evaluated_time >= self.acyclic_contact_plan[j][0][4]:
                                cnt_plan[i][j][0] = 1
                                cnt_plan[i][j][1:4] = self.acyclic_contact_plan[j][0][0:3]
                                cnt_plan[i][j][4:8] = temp_quat
                                cnt_plan[i][j][8] = self.dt
                        elif k == self.acyclic_contact_plan[j].shape[0]:
                            if self.evaluated_time <= self.acyclic_contact_plan[j][k][3] \
                                and self.evaluated_time >= self.acyclic_contact_plan[j][k][4]:
                                cnt_plan[i][j][0] = 1
                                cnt_plan[i][j][1:4] = self.acyclic_contact_plan[j][k][0:3]
                                cnt_plan[i][j][4:8] = temp_quat
                                cnt_plan[i][j][8] = self.dt
                            if self.evaluated_time >= self.acyclic_contact_plan[j][k][3]:
                                cnt_plan[i][j][0] = 1
                                cnt_plan[i][j][1:4] = self.acyclic_contact_plan[j][k][0:3]
                                cnt_plan[i][j][4:8] = temp_quat
                                cnt_plan[i][j][8] = self.dt
                        else:
                            if self.evaluated_time <= self.acyclic_contact_plan[j][k][3] \
                                and self.evaluated_time >= self.acyclic_contact_plan[j][k][4]:
                                cnt_plan[i][j][0] = 1
                                cnt_plan[i][j][1:4] = self.acyclic_contact_plan[j][k][0:3]
                                cnt_plan[i][j][4:8] = temp_quat
                                cnt_plan[i][j][8] = self.dt
                            if self.evaluated_time >= self.acyclic_contact_plan[j][k][3]\
                                and self.evaluated_time <= self.acyclic_contact_plan[j][k+1][4]:
                                cnt_plan[i][j][0] = 0
                    

            return cnt_plan