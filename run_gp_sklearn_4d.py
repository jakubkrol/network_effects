import pickle
import sys
import threading
import time

try:
    import matlab.engine
except:
    print("Did not manage to import the matlab engine") 

import numpy as np
import torch

from scipy import interpolate

try:
    import thread
except ImportError:
    import _thread as thread

from threading import Thread
import functools

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.preprocessing import StandardScaler

# ADD THIS MOMENT FUNCTION IS NOT SUITABLE FOR LONG PREDICTION HORIZONS
def prepare_inputs(ego_vels_trunc, headways_trunc, lead_vels_trunc, 
    max_vels_trunc, opt_vel_trunc):
    # the opt vel should be one dimensional here
    inputs = np.array([ego_vels_trunc, headways_trunc, lead_vels_trunc, 
        max_vels_trunc]).T
    inputs = np.squeeze(inputs)
    # Standardise data
    input_scaler = StandardScaler()
    output_scaler = StandardScaler()

    inputs_scaled = input_scaler.fit_transform(inputs)
    output_scaled = output_scaler.fit_transform(opt_vel_trunc)

    return inputs_scaled, output_scaled, input_scaler, output_scaler


def gp_receding_horizon(W1, x_l_0, v_0, v_l_0, v_max, gp_model, 
    opt_horizon=1, lead_id=None, ego_id=None, gp_treshold=1.):
 
    Soc = 0.7
    x_e_0 = 0.0
    x_l_0 = min(250., x_l_0)
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
    gp_treshold = gp_treshold

    # some heuristic rules
    empty_road_check = abs(v_0 - v_l_0) < tolerance / 10 and \
        abs(v_max - v_l_0) < tolerance / 10 and (s_d + T_d * v_max) < x_l_0

    if empty_road_check:
        return v_0 * np.ones((opt_horizon, 1)), gp_model

    red_light_check = v_0 < tolerance / 10 and v_l_0 < tolerance / 10 and \
        x_l_0 < s_d

    if red_light_check:
        return np.zeros((opt_horizon, 1)), gp_model

    # try using current solution map
    pred_vels = []
    uncertainties = []
    for timestep in range(opt_horizon):
        model = gp_model['model']
        input_scaler = gp_model['scaler_in']
        output_scaler = gp_model['scaler_out']

        input_ = np.array([v_0, x_l_0, v_l_0, v_max]).reshape(1, -1)
        input_ = input_scaler.transform(input_)
        output = model.predict(input_, return_std=True)
        prediction = output[0]
        uncertainty = output[1]
 
        pred_vels.append(output_scaler.inverse_transform(prediction))        
        uncertainties.append(uncertainty)

    pred_vels = np.array(pred_vels)
    uncertainties = np.array(uncertainties)
    if sum(uncertainties > gp_treshold) == 0:
        return pred_vels, gp_model
    
    # falling back on the matlab optimisation   
    else:
        # falling back on the matlab optimisation   
        try:
            print('Uncertainty', uncertainties)
            print('INPUTS', 'headway:', x_l_0, 'lead:', v_l_0, 'ego:', v_0, 'vmax:', 
                v_max)
            print('Lead vehicle:', lead_id, "Ego vehicle:", ego_id)
        except:
            pass

        # run gpops optimisation
        eng = matlab.engine.start_matlab()
        t_lead = np.arange(t0, tf + delta_t, delta_t)
        v_lead = float(v_l_0) * np.ones(t_lead.shape) 

 
        t_lead = matlab.double(list(t_lead))
        v_lead = matlab.double(list(v_lead))
        output = eng.EV_SimpleDMMain_func(float(W1), t_lead, v_lead, v_0, x_e_0, 
            x_l_0, v_max, s_d, T_d, SoC_0)
        output = np.array(output._data)
        time = output[:int(output.size / 2)]
        v = output[int(output.size / 2):] 
        
        f = interpolate.interp1d(time, v, 'linear') 
        v_e_interp = f(t_lead)
        v_append = v_e_interp[0][1:] # keep whole thing in solution space
        v_out = v_e_interp[0][1: opt_horizon + 1]

        # append solution space
        name_map = {1: 'new_sol_space_dsm.pkl', 
            4000: 'new_sol_space_trade.pkl',
            100000: 'new_sol_space_min.pkl'} 
        
        with open(name_map[int(W1)], 'rb') as handle:
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
        print('Recalculating')
        # recompute the solution space
        X_train, y_train, scaler_x, scaler_y = prepare_inputs(
            sol_space['ego_vels'], sol_space['headways'], 
            sol_space['lead_vels'], sol_space['max_vels'],
            sol_space['opt_vel'].T[:, 0].reshape(-1, 1))
        gpr = GaussianProcessRegressor(random_state=42, alpha=1e-4).fit(X_train, y_train) 
        gp_model = {'model': gpr, 'scaler_in': scaler_x, 
            'scaler_out': scaler_y}
 
        model_name_map = {1: 'gp_sol_space_dsm_sklearn.pkl', 
            4000: 'gp_sol_space_trade_sklearn.pkl',
            100000: 'gp_sol_space_min_sklearn.pkl'} 
 
        with open(model_name_map[int(W1)], 'wb') as handle:
            pickle.dump(gp_model, handle, protocol=pickle.HIGHEST_PROTOCOL)      
 
        return v_out, gp_model
       
