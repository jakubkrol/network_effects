import sys

import numpy as np

import recompute_maps_4d_func
sys.path.append('/mnt/c/Users/user/Documents/myLibs')
sys.path.append(r'C:\Users\user\Documents\myLibs')

from g_active_control import run_simulation_var_param
from gp_maps import GPModel

combinations = [ 
    ('ADAM_TRADE', 0.25)]  

for comb in combinations:
    key, percentage = comb
    run_simulation_var_param(key, percentage, win=True, horizon=1, gui=False)


keys = ['ADAM_DSM', 'ADAM_MIN', 'ADAM_TRADE']
percentages = [0.5]

for percentage in percentages:
    for key in keys:
        run_simulation_var_param(key, percentage, win=True, horizon=1, gui=False)




