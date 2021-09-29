## This is a demo for Acyclic Motions
## Author : Paarth Shah
## Date : 21/04/2021

import time
import numpy as np

from plan_hifive import plan
from contact_planner.acyclic_contact_generator import AcyclicContactPlanner

contact_planner = AcyclicContactPlanner(plan)

plan1 = contact_planner.create_cnt_plan(0, 0, 0, np.zeros(4), np.zeros(4))
print("Plan 1")
print(plan1)

plan2 = contact_planner.create_cnt_plan(0, 0, 1.75, np.zeros(4), np.zeros(4))
print("Plan 2")
print(plan2)