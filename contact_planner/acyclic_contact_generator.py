import pinocchio as pin
import numpy as np

class AcyclicContactPlanner:
    def __init__(self, acyclic_plan, robot_info = None, height_map = None):
        self.foot_size = 0.18
        self.gravity = 9.81
        self.n_eef = 4

        #Read this in from somewhere
        self.dt_arr = acyclic_plan.dt_arr
        self.horizon = acyclic_plan.n_col

        #Create instance of continuous gait planner
        self.acyclic_contact_plan = np.array(acyclic_plan.cnt_plan)

        #Use current contact location when computing locations
        self.use_current_eef_location = False
        self.use_current_contact = False

    def create_cnt_plan(self, q, v, t, current_contact, eef_locations):
            """
            Creates Contact Plan Matrix:
            Input: q (position), v (velocity), t (current time), current_contact (boolean array of which end-effectors
                are in contact), eef_locations (n_eef x 3 matrix of end-effector positions)
            Output: Contact Plan Matrix
            Dimensions: horizon (number of collocation points) x num_eef x 9
            Extra Information: The final dimension (9) gives the contact boolean
                               i.e. the last vector should be [1/0, pos, orientation (quaternion), dt]
                               where 1/0 gives a boolean for contact (1 = contact, 0 = no cnt)
            """
            cnt_plan = np.zeros((self.horizon, self.n_eef, 9))
            evaluated_time = t

            #TODO: Read this from a height-map
            temp_quat = np.array([0,0,0,1])
            prev_current_eef_used = np.ones(self.n_eef)

            for i in range(self.horizon):
                for j in range(self.n_eef):
                    for k in range(self.acyclic_contact_plan.shape[0]):
                        if self.acyclic_contact_plan[k][j][4] <= evaluated_time \
                                < self.acyclic_contact_plan[k][j][5]:
                            cnt_plan[i][j][0] = self.acyclic_contact_plan[k][j][0]
                            cnt_plan[i][j][1:4] = self.acyclic_contact_plan[k][j][1:4]
                            cnt_plan[i][j][4:8] = temp_quat
                            cnt_plan[i][j][8] = self.dt_arr[i]

                            if self.use_current_eef_location and cnt_plan[i][j][0] == 1 and \
                                prev_current_eef_used[j] == 1:
                                    cnt_plan[i][j][1:4] = eef_locations[j]

                            if cnt_plan[i][j][0] == 0:
                                prev_current_eef_used[j] = 0

                        if evaluated_time >= self.acyclic_contact_plan[-1][j][5]:
                            cnt_plan[i][j][0] = 1
                            cnt_plan[i][j][1:4] = self.acyclic_contact_plan[-1][j][1:4]
                            cnt_plan[i][j][4:8] = temp_quat
                            cnt_plan[i][j][8] = self.dt_arr[i]

                            if self.use_current_eef_location and cnt_plan[i][j][0] == 1 and \
                                prev_current_eef_used[j] == 1:
                                    cnt_plan[i][j][1:4] = eef_locations[j]

                evaluated_time += self.dt_arr[i]

            return cnt_plan