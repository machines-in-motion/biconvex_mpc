## This file contains the motion plan for Standing In Place
## Author : Avadesh Meduri & Paarth Shah
## Date : 21/09/2021

import numpy as np
from robot_properties_atlas.config import AtlasConfig
from motions.weight_abstract import ACyclicMotionParams
import pinocchio as pin
###########################################################################################

robot = AtlasConfig.buildRobotWrapper()
rmodel = robot.model
rdata = robot.data

q0 = np.array(AtlasConfig.initial_configuration)
v0 = pin.utils.zero(rmodel.nv)
x0 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])


###########################################################################################
plan = ACyclicMotionParams("Atlas", "stand")

T = 10
dt = 5e-2
plan.n_col = 30
# dt = T/30
plan.dt_arr = plan.n_col*[dt,]
plan.plan_freq = [[0.1, 0, T]]

plan.cnt_plan = [[[ 1.,      -0.13039155,  0.1784435,   0., 0.,  T    ],
                  [ 1.,      0.09660845,   0.1784435,   0., 0.,  T    ],
                  [ 1.,      -0.13039155,  0.0445565,   0., 0.,  T    ],
                  [ 1.,      0.09660845,   0.0445565,   0., 0.,  T    ],
                  [ 1.,      -0.13039155, -0.0445565,   0., 0.,  T    ],
                  [ 1.,      0.09660845,  -0.0445565,   0., 0.,  T    ],
                  [ 1.,      -0.13039155, -0.1784435,   0., 0.,  T    ],
                  [ 1.,      0.09660845,  -0.1784435,   0., 0.,  T    ]]]
# j: [-0.13039155  0.1784435   0.131473  ]
# j: [0.09660845 0.1784435  0.131473  ]
# j: [-0.13039155  0.0445565   0.131473  ]
# j: [0.09660845 0.0445565  0.131473  ]
# j: [-0.13039155 -0.0445565   0.131473  ]
# j: [ 0.09660845 -0.0445565   0.131473  ]
# j: [-0.13039155 -0.1784435   0.131473  ]
# j: [ 0.09660845 -0.1784435   0.131473  ]


#  dynamic optimization params
plan.W_X =        np.array([1e0, 1e0, 1e+4, 1e-4, 1e-4, 2e2, 3e+4, 1e+4, 3e+4])
plan.W_X_ter = 10.*np.array([1e3, 1e-5, 1e+4, 1e-1, 1e-1, 2e2, 1e+1, 1e+1, 1e+1])
plan.W_F = np.array(8*[1e-1, 1e-1, 1e-1])
plan.rho = 5e+5

plan.X_nom = [[0.01423163, 0.00103309, 1.12, 0, 0, 0, 0, 0.00, 0.0, 0.0, T]]

plan.X_ter = [0.01423163, 0.00103309, 1.12, 0, 0, 0, 0, 0.0, 0.0]
plan.bounds = [[-0.5, -0.5, 1.1, 0.5, 0.5, 1.14, 0., T]]

# ik optimization params
plan.cent_wt = [3*[100.,], 6*[.04,]]
plan.cnt_wt = 1e2

x_reg1 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])

state_wt_1 = np.array([1e-2, 1e-2, 1e2 ] + [5.0, 5.0, 5.0] + [1e2] * (robot.model.nv - 6) +\
                      [0.00, 0.00, 0.00] + [5.0, 5.0, 5.0] + [3.5] * (robot.model.nv - 6)
                      )

plan.state_reg = [np.hstack((x_reg1, [0, T]))]
plan.state_wt = [np.hstack((state_wt_1, [0, T]))]
plan.state_scale = [[1e-2, 0, T]]

ctrl_wt = [0, 0, 1] + [1, 1, 1] + [5.0] *(rmodel.nv - 6)
plan.ctrl_wt = [np.hstack((ctrl_wt, [0, T]))]
plan.ctrl_reg = [np.hstack((np.zeros(rmodel.nv), [0, T]))]
plan.ctrl_scale = [[1e-4, 0, T]]

# controller details
plan.kp = [[2.5, 0, T]]
plan.kd = [[0.5, 0, T]]
