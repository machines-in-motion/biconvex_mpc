 import pinocchio as pin

 from gait_planner_cpp import GaitPlanner

 class ContactPlanner:

    def __init__(self, gait_params, robot_info, height_map = None):
        self.gait_period = gait_params.gait_period
        self.stance_percent = gait_params.stance_percent
        self.nominal_height = gait_params.nom_ht
        self.phase_offset = gait_params.phase_offset
        self.step_height = gait_params.step_height

        self.foot_size = robot_info.foot_size
        self.gravity = 9.81
        
        #Create instance of continuous gait planner
        self.gait_planner = GaitPlanner(self.gait_period, np.array(self.stance_percent), \
                                            np.array(self.phase_offset), self.step_height)

        self.raibert_feedback = 0.05

    def create_cnt_plan(self, q, v, t, v_des, w_des):
            """
            Creates Contact Plan Matrix:
            Dimensions: horizon (number of collocation points) x num_eef x 4
            Extra Information: The final dimension (4) gives the contact boolean
                               i.e. the last vector should be [1/0, x, y, z] 
                               where 1/0 gives a boolean for contact (1 = contact, 0 = no cnt)
            """
            self.cnt_plan = np.zeros((self.horizon, len(self.eff_names), 4))
            self.swing_time = np.zeros((self.horizon, len(self.eff_names)))
            self.prev_cnt = np.zeros((len(self.eff_names), 3))
            self.curr_cnt = np.zeros(len(self.eff_names))
            
            com = np.round(pin.centerOfMass(self.rmodel, self.rdata, q, v), 3)
            z_height = self.nominal_height
            vcom = np.round(v[0:3], 3)

            #Get current rotation
            R = pin.Quaternion(np.array(q[3:7])).toRotationMatrix()

            vtrack = v_des
            #vtrack = vcom[0:2] #Explicitly track actual center of mass velocity (i.e. raibert style)
            
            for i in range(self.horizon):
                for j in range(len(self.eff_names)):
                    if i == 0:
                        if self.gait_planner.get_phase(t, j) == 1:
                            self.cnt_plan[i][j][0] = 1
                            self.cnt_plan[i][j][1:4] = np.round(self.rdata.oMf[self.ee_frame_id[j]].translation, 3)
                            self.prev_cnt[j] = self.cnt_plan[i][j][1:4]
                        else:
                            self.cnt_plan[i][j][0] = 0
                            self.cnt_plan[i][j][1:4] = np.round(self.rdata.oMf[self.ee_frame_id[j]].translation, 3)

                    else:
                        #All other time steps
                        future_time = np.round(t + i*self.gait_dt,3)

                        if self.gait_planner.get_phase(future_time, j) == 1:
                            #If foot will be in contact
                            self.cnt_plan[i][j][0] = 1
                            
                            if self.cnt_plan[i-1][j][0] == 1:
                                self.cnt_plan[i][j][1:4] = self.cnt_plan[i-1][j][1:4]
                            else:
                                hip_loc = com + np.matmul(R, self.offsets[j]) + i*self.params.gait_dt*vtrack
                                raibert_step = 0.5*vtrack*self.params.gait_period*self.params.stance_percent[j] - \
                                    self.raibert_feedback*(vtrack - v_des)
                                ang_step = 0.5*np.sqrt(z_height/self.gravity)*vtrack
                                ang_step = np.cross(ang_step, [0.0, 0.0, w_des])
                            
                                self.cnt_plan[i][j][1:4] = raibert_step + hip_loc + ang_step

                                if self.height_map != None:
                                    self.cnt_plan[i][j][3] = self.height_map.getHeight(self.cnt_plan[i][j][1], self.cnt_plan[i][j][2]) +\
                                                                                    self.foot_size
                                else:
                                    self.cnt_plan[i][j][3] = self.foot_size

                            self.prev_cnt[j] = self.cnt_plan[i][j][1:4]

                        else:
                            #If foot will not be in contact
                            self.cnt_plan[i][j][0] = 0

                            hip_loc = com + np.matmul(R,self.offsets[j]) + i*self.params.gait_dt*vtrack
                            ang_step = 0.5*np.sqrt(z_height/self.gravity)*vtrack
                            ang_step = np.cross(ang_step, [0.0, 0.0, w_des])

                            percent_phase = np.round(self.gait_planner.get_percent_in_phase(future_time, j), 3)
                            
                            if percent_phase < 0.5:
                                self.cnt_plan[i][j][1:4] = hip_loc + ang_step
                            else:
                                raibert_step = 0.5*vtrack*self.params.gait_period*self.params.stance_percent[j] - 
                                    self.raibert_feedback*(vtrack - v_des)
                                self.cnt_plan[i][j][1:4] = hip_loc + ang_step

                            if percent_phase - 0.5 < 0.02:
                                self.swing_time[i][j] = 1

                            if self.height_map != None:
                                self.cnt_plan[i][j][3] = self.height_map.getHeight(self.cnt_plan[i][j][1], self.cnt_plan[i][j][2]) + \
                                                        self.foot_size
                            else:
                                self.cnt_plan[i][j][3] = self.foot_size

            return self.cnt_plan