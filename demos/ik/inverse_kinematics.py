## This file contains the implementation of the DDP based IK
## Author : Avadesh Meduri
## Date : 24/02/2021

import numpy as np

import pinocchio
import crocoddyl
from . action_model import DifferentialFwdKinematics


# class InverseKinematics:

#     def __init__(self, rmodel, dt, T):
#         """
#         This class handles the inverse kinematics for the plan with crocoddyl
#         Input:
#             rmodel : pinocchio robot model
#         """
