## Contains functions to generate trajectories. Not specific to any robot.
## Author: Avadesh Meduri
## Date: 9/12/2019

import numpy as np
from . qp_solver import quadprog_solve_qp

class TrajGenerator:
    
    def __init__(self, robot):
        
        self.robot = robot
        self.W = [10, 10, 10, 10, 10] # weights for qp traj

    def get_frame_location(self, q, dq, frame_idx):
        """
            returns the global location of the frame by computing forward kinematics
            Input:
                q: current joint configuration of the robot
                dq: current joint velocity of the robot
        """
        
        self.robot.framesForwardKinematics(q)
        return np.reshape(np.array(self.robot.model.oMf[frame_idx].translation), (3,))
        
    def generate_qp_traj(self, start, end, via_point, traj_time, t):
        """
            Generates foot trajectory for walking, using a qp
            Fits a 3rd degree polynomial
            Input:
                Start: Current location of the foot 3d
                end: desired location of the of the foot at the end of the step
                via_point: the hieght in the z axis the leg has to rise
                traj_time: step time
                t: current time
        """
        assert np.shape(start) == np.shape(end)
        assert np.shape(start) == np.shape(via_point)
        
        P = np.identity(5)
        P[0][0], P[1][1], P[2][2], P[3][3], P[4][4] = self.W
        q = np.zeros(5)

        G = np.matrix([[0, 0, 0, 0, 0]])
        h = np.array([0])

        A = np.matrix([ [1, t, t*t, t**3, t**4],
                    [0, 1, 2*t, 3*t*t, 4*t**3],
                    [1, traj_time, traj_time**2, traj_time**3, traj_time**4],
                    [0, 1, 2*traj_time, 3*traj_time**2, 4*traj_time**3]])
        
        b = np.array([start[0], 0, end[0], 0])

        x_opt = quadprog_solve_qp(P, q, G, h, A, b)

        b = np.array([start[1], 0, end[1], 0])
        y_opt = quadprog_solve_qp(P, q, G, h, A, b)

        A = np.matrix([[1, t, t*t, t**3, t**4],
                        [0, 1, 2*t, 3*t*t, 4*t**3],
                        [1, 0.5*traj_time, (1/4)*traj_time**2, (1/8)*traj_time**3, (1/16)*traj_time**4],
                        [1, traj_time, traj_time**2, traj_time**3, traj_time**4],
                        [0, 1, 2*traj_time, 3*traj_time**2, 4*traj_time**3]])
        
        b = np.array([start[2], 0, via_point[2], end[2], 0])        
        z_opt = quadprog_solve_qp(P, q, G, h, A, b)

        return x_opt, y_opt, z_opt
    
    def compute_polynomial(self, a, t):
        '''
        Computes the value of a the polynomial
        a : co effecient matrix 
        t : time
        '''
        x = 0
        xd = 0
        for i in range(len(a)):
            x += a[i]*t**i
            if i > 0:
                xd += i*a[i]*t**(i-1)
        
        return x, xd

    def generate_foot_traj(self,start, end, via_point, traj_time,t):
        """
            Generates foot trajectory for walking. A uniform velocity is tracked in x,y direction.
            A sine trajectory is generated in the z direction. 
            Input:
                Start: Current location of the foot 3d
                end: desired location of the of the foot at the end of the step
                via_point: the hieght in the z axis the leg has to rise
                traj_time: step time
                t: current time
        """
 
        assert np.shape(start) == (3,)
        assert np.shape(end) == (3,)
        x_des = start
        xd_des = np.zeros(3)

        if t == 0:
            self.x_opt, self.y_opt, self.z_opt = \
                    self.generate_qp_traj(start, end, via_point, traj_time,t)
            
        x_des[0], xd_des[0] =  self.compute_polynomial(self.x_opt, t)
        x_des[1], xd_des[1] =  self.compute_polynomial(self.y_opt, t)
        x_des[2], xd_des[2] =  self.compute_polynomial(self.z_opt, t)
            
        return x_des, xd_des

