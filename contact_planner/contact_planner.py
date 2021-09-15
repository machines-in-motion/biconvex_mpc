 
 class ContactPlanner:

    def __init__(self, gait_params, robot_model, height_map = None):

 
    def create_cnt_plan(self, q, v, t, v_des, w_des):
            """
            Creates Contact Plan Matrix:
            Dimensions: horizon (number of collocation points) x num_eef x 4
            Extra Information: The final dimension (4) gives the contact boolean
                               i.e. the last vector should be [1/0, x, y, z] 
                               where 1/0 gives a boolean for contact (1 = contact, 0 = no cnt)
            """
            com = np.round(pin.centerOfMass(self.rmodel, self.rdata, q, v)[0:2], 3)
            z_height = pin.centerOfMass(self.rmodel, self.rdata, q, v)[2]
            vcom = np.round(v[0:3], 3)

            #Get current rotation
            R = pin.Quaternion(np.array(q[3:7])).toRotationMatrix()

            vtrack = v_des[0:2] # this effects the step location (if set to vcom it becomes raibert)
            #vtrack = vcom[0:2]

            self.cnt_plan = np.zeros((self.horizon, len(self.eff_names), 4))

            # This array determines when the swing foot cost should be enforced in the ik
            self.swing_time = np.zeros((self.horizon, len(self.eff_names)))
            self.prev_cnt = np.zeros((len(self.eff_names), 3))
            self.curr_cnt = np.zeros(len(self.eff_names))

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
                                hip_loc = com + np.matmul(R, self.offsets[j])[0:2] + i*self.params.gait_dt*vtrack
                                raibert_step = 0.5*vtrack*self.params.gait_period*self.params.stance_percent[j] - 0.05*(vtrack - v_des[0:2])
                                ang_step = 0.5*np.sqrt(z_height/self.gravity)*vtrack
                                ang_step = np.cross(ang_step, [0.0, 0.0, w_des])
                            
                                self.cnt_plan[i][j][1:3] = raibert_step[0:2] + hip_loc + ang_step[0:2]

                                if self.height_map != None:
                                    self.cnt_plan[i][j][3] = self.height_map.getHeight(self.cnt_plan[i][j][1], self.cnt_plan[i][j][2]) +\
                                                                                    self.foot_size
                                else:
                                    self.cnt_plan[i][j][3] = self.foot_size

                            self.prev_cnt[j] = self.cnt_plan[i][j][1:4]

                        else:
                            #If foot will not be in contact
                            self.cnt_plan[i][j][0] = 0

                            hip_loc = com + np.matmul(R,self.offsets[j])[0:2] + i*self.params.gait_dt*vtrack
                            ang_step = 0.5*np.sqrt(z_height/self.gravity)*vtrack
                            ang_step = np.cross(ang_step, [0.0, 0.0, w_des])

                            percent_phase = np.round(self.gait_planner.get_percent_in_phase(future_time, j), 3)
                            
                            if percent_phase < 0.5:
                                self.cnt_plan[i][j][1:3] = hip_loc + ang_step[0:2]
                            else:
                                raibert_step = 0.5*vtrack*self.params.gait_period*self.params.stance_percent[j] - 0.05*(vtrack - v_des[0:2])
                                self.cnt_plan[i][j][1:3] = hip_loc + ang_step[0:2]

                            if percent_phase - 0.5 < 0.02:
                                self.swing_time[i][j] = 1

                            if self.height_map != None:
                                self.cnt_plan[i][j][3] = self.height_map.getHeight(self.cnt_plan[i][j][1], self.cnt_plan[i][j][2]) + \
                                                        self.foot_size
                            else:
                                self.cnt_plan[i][j][3] = self.foot_size

            return self.cnt_plan