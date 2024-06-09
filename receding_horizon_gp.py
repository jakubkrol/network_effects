import pickle

import numpy as np

# load solution space (ideally only do it once)
with open('gp_trade_3s.pkl', 'rb') as handle:
    gp_model = pickle.load(handle)

model = gp_model['model']
input_scaler = gp_model['scaler_in']
output_scaler = gp_model['scaler_out']

kick_start = model.predict(np.array([4., 6.12, 8.12, 13.41]).reshape(1, -1), 
    return_std=True)

def receding_horizon_gp(x_l_0, v_0, v_l_0, v_max):
    
    # truncate to ensure numerical stability    
    x_l_0 = min(250., x_l_0)

    # check if heuristic rules apply
    
    s_d = 2.5
    T_d = 1.5
    tolerance = 0.1
    uncertainty_scale_factor =  1. / 6.;

    empty_road_check = abs(v_0 - v_l_0) < tolerance / 10 and \
        abs(v_max - v_l_0) < tolerance / 10 and (s_d + T_d * v_max) < x_l_0

    if empty_road_check:
        return v_0, v_0, v_0

    red_light_check = v_0 < tolerance / 10 and v_l_0 < tolerance / 10 and \
        x_l_0 < s_d

    if red_light_check:
        return 0., 0., 0.

    # use the gaussian processes model
   
    input_ = np.array([v_0, x_l_0, v_l_0, v_max]).reshape(1, -1)
    input_ = input_scaler.transform(input_)
    print('Second input', input_)
    
    output = model.predict(input_, return_std=True)
    print('Second output', output[0])
    
    prediction = output[0]
    uncertainty = output[1] * uncertainty_scale_factor 

    higher_bound = prediction + uncertainty
    lower_bound = prediction - uncertainty
    
    prediction = output_scaler.inverse_transform(prediction)
    higher_bound = output_scaler.inverse_transform(higher_bound)
    lower_bound = output_scaler.inverse_transform(lower_bound)
   
    return prediction, higher_bound, lower_bound
    
