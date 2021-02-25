# The file is a demo Fast Iterative Shrinkage Thresholding
# This algorithm is based on the Algorithm described in First Order Methods 
# by Amir Beck. ch10
# Author : Avadesh Meduri
# Date : 5/12/2020

import numpy as np
from py_biconvex_mpc.solvers.fista import FISTA


Q = np.matrix(np.identity(2))
q = np.matrix([[-5.0], [10.0]])

f = lambda x : np.transpose(x)*Q*x + q.T*x
f_grad = lambda x : np.matrix(2.0*Q*x + q)

g = lambda x, L : np.matrix(np.clip(x, 0.5, 2))

x0 = np.matrix([[200],[30]])
fista = FISTA(0.001, 1.1)
x_opt = fista.optimize(f, f_grad, g, x0, 100, 0.001)
# fista.stats()
print(x_opt)