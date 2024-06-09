import pickle
import stopit
import sys
import threading
import time

import gpytorch
import matlab.engine
import numpy as np
import torch

from scipy import interpolate

try:
    import thread
except ImportError:
    import _thread as thread

from threading import Thread
import functools


def gp_receding_horizon(W1, x_l_0, v_0, v_l_0, v_max, sol_space, 
    opt_horizon=1):

    Soc = 0.7
    x_e_0 = 0.0
    horizon = 60.0
    t0 = 0.0
    tf = t0 + horizon
    SoC_0 = Soc
    delta_t = 0.1
    
    t_lead = np.arange(t0, tf + delta_t, delta_t)
    v_lead = float(v_l_0) * np.ones(t_lead.shape) 

    s_d = 2.5
    T_d = 1.5

    tolerance = 0.1
    gp_treshold = 1.

    # some heuristic rules
    empty_road_check = abs(v_0 - v_l_0) < tolerance / 10 and \
        abs(v_max - v_l_0) < tolerance / 10 and (s_d + T_d * v_max) < x_l_0

    if empty_road_check:
        return v_0 * np.ones((opt_horizon, 1))

    red_light_check = v_0 < tolerance / 10 and v_l_0 < tolerance / 10 and \
        x_l_0 < s_d

    if red_light_check:
        return np.zeros((opt_horizon, 1))

    # try using current solution map
    pred_vels = []
    uncertainties = []
    for timestep in range(opt_horizon):
        model = sol_space.models[str(v_max)][timestep]
        likelihood = sol_space.likelihoods[str(v_max)][timestep]
        input_scaler = sol_space.scalers[str(v_max)][timestep][0]
        output_scaler = sol_space.scalers[str(v_max)][timestep][1]

        model.eval() 
        likelihood.eval()
        input_ = np.array([v_0, x_l_0, v_l_0])
        input_ = input_scaler.transform(input_)
        with torch.no_grad(), gpytorch.settings.fast_pred_var() 
            input_ = torch.tensor(input_, dtype=torch.float)
            observed_pred = likelihood(model(input_))
            prediction = observed_pred.mean.numpy()
            lower, upper = observed_pred.confidence_region()
        pred_vels.append(output_scaler.inverse_transform(prediction))
        
        lower = output_scaler.inverse_transform(lower)        
        upper = output_scaler.inverse_transform(upper)
        
        uncertainties.append(upper - lower)

    pred_vels = np.array(pred_vels)
    uncertainties = np.array(uncertainties)

    if sum(uncertainties > gp_treshold) == 0:
        return pred_vels
    
    # falling back on the matlab optimisation   
    else:

        # run gpops optimisation
        eng = matlab.engine.start_matlab()
        
        t_lead = matlab.double(list(t_lead))
        v_lead = matlab.double(list(v_lead))
  
        output = eng.EV_SimpleDMMain_func(float(W1), t_lead, v_lead, v_0, x_e_0, 
            x_l_0, v_max, s_d, T_d, SoC_0)
       
        output = np.array(output._data)
        time = output[:int(output.size / 2)]
        v = output[int(output.size / 2):] 
        
        f = interpolate.interp1d(time, v, 'cubic') 
        v_e_interp = f(t_lead)
        v_append = v_e_interp[0][1:] # keep whole thing in solution space
        v_out = v_e_interp[0][1: opt_horizon + 1]

        # append solution space

        name_map = {1: 'sol_space_dsm_60.pkl', 4000: 'sol_space_trade_60.pkl',
            100000: 'sol_space_min_60.pkl'} 
        
        with open(name_map[int(W1)], 'wb') as handle:
            sol_space = pickle.load(handle)      
  
        sol_space['headways'].append(x_l_0)
        sol_space['opt_vel'] = np.hstack([sol_space['opt_vel'], 
            v_append.reshape(-1, 1)])
        sol_space['ego_vels'].append(v_0)
        sol_space['max_vels'].append(v_max)
        sol_space['lead_vels'].append(v_l_0)
       
        with open(name_map[int(W1)], 'wb') as handle:
            pickle.dump(sol_space, handle, protocol=pickle.HIGHEST_PROTOCOL)

        eng.quit() 
        return v_out
       



