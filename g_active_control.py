# -*- coding: utf-8 -*-
import copy
import math
import multiprocessing
import os
import pickle
import random
import sys

sys.path.insert(0, '/mnt/c/Users/user/Documents/myLibs/1_sumoAPI')
sys.path.append(r'C:\Users\user\Documents\myLibs\1_sumoAPI')

import sumoConnect
import traci
from conf_load_state import sumoConfigGen, sumoConfigGen_win
import numpy as np
import traci.constants as tc
import time
from collections import defaultdict
import signalTools as st

from copy import deepcopy

def run_reference_sim(win=True):
    
    timer = st.simTimer()
    timer.start()

    # Define road model directory
    modelname = 'sellyOak'
    modelBase = modelname.split('_')[0]
    model = '{}/'.format(modelBase)

    # Generate new routes
    # stepSize = 0.1  # simulation step size in seconds (default 1 sec)
    stepSize = 0.1
    CVP = 0  # percentage of connected vehicles
    seed = 7  # seed for random number generator

    print('STARTING: {}, Run: {:03d}, AVR: {:03d}%, Date: {}'
            .format(modelname, seed, int(CVP*100), time.ctime()))

    # Edit the the output filenames in sumoConfig
    configFile = model + modelBase + ".sumocfg"

    exportPath = 'test_results/'
    if not os.path.exists(model+exportPath): # this is relative to cfg
        os.makedirs(model+exportPath)

    simport = 8857  # comm port for traci
    sumo_config_gen = sumoConfigGen_win if win else sumoConfigGen 
    sumo_config_gen(modelname, configFile, exportPath, CVP=CVP, 
        stepSize=stepSize, run=seed, port=simport, seed=seed, 
        output_name='adam_original_reference_sim') 
    
    # Connect to model
    connector = sumoConnect.sumoConnect(configFile, gui=False, port=simport)
    connector.launchSumoAndConnect()

    # Step simulation while there are vehicles
    simTime, simActive = 0, True
    timeDelta = int(1000*stepSize)

    # get g_active vehicles
    veh_in_sim = list(traci.vehicle.getIDList())
    SEED = 448
    random.seed(SEED)
    random.shuffle(veh_in_sim)
    g_active_veh = veh_in_sim

    with open('seeded_veh_cong.pkl', 'wb') as handle:
        pickle.dump(g_active_veh, handle, protocol=pickle.HIGHEST_PROTOCOL)

    # set min gap to 2.0 for a more reliable comparison
    for veh in g_active_veh:
        traci.vehicle.setMinGap(veh, 2.0)
    
    # for gathering solution
    counter = 0
    while simActive:

        counter += 1
        traci.simulationStep()
        simTime += timeDelta

        for vehid in g_active_veh: 
            if (vehid not in traci.vehicle.getIDList()):
                g_active_veh.remove(vehid)

        if len(g_active_veh) == 0:
            simActive = 0

        if counter % 100 == 0:
            print('still alive at ', counter)    
            print(g_active_veh)

    connector.disconnect()
    timer.stop()
    print('Simulations complete, exectime: {}, {}'.format(timer.strTime(), \
        time.ctime()))
    print('DONE')


def run_simulation_to_xml(control_key, percentage, win=False, horizon=10, 
    gui=False):
    
    weights = {'ADAM_DSM': 1e0, 'ADAM_MIN': 1e5, 'ADAM_TRADE': 4e3}

    # Define road model directory
    modelname = 'sellyOak'
    model = '{}/'.format(modelname)

    # Generate new routes
    stepSize = 0.1
    CVP = 0  # percentage of connected vehicles
    seed = 7  # seed for random number generator

    print('STARTING: {}, Run: {:03d}, AVR: {:03d}%, Date: {}'
            .format(modelname, seed, int(CVP*100), time.ctime()))

    # Edit the the output filenames in sumoConfig
    configFile = "{}/{}.sumocfg".format(modelname, modelname)

    exportPath = 'test_results/'
    if not os.path.exists(model+exportPath): # this is relative to cfg
        os.makedirs(model+exportPath)

    simport = 8857  # comm port for traci
    sumo_config_gen = sumoConfigGen_win if win else sumoConfigGen 
    sumo_config_gen(modelname, configFile, exportPath, CVP=CVP, 
        stepSize=stepSize, run=seed, port=simport, seed=seed, 
        output_name=control_key.lower() + str(percentage).replace('.', '')) 
    
    # Connect to model
    connector = sumoConnect.sumoConnect(configFile, gui=gui, port=simport)
    connector.launchSumoAndConnect()

    # Step simulation while there are vehicles
    simTime, simActive = 0, True
    timeDelta = int(1000*stepSize)

    # get g_active vehicles
    
    veh_in_sim = list(traci.vehicle.getIDList())
    SEED = 448
    random.seed(SEED)
    random.shuffle(veh_in_sim)
    g_active_no = int(math.ceil(percentage * len(veh_in_sim)))
    g_active_veh = veh_in_sim[:g_active_no]
  
    g_active_controller = st.GaussianProcessesController(
        weights[control_key], horizon, g_active_veh)
    speed_mode = st.SpeedModeSetter(copy.deepcopy(g_active_veh))
  
    # for gathering solution
    counter = 0
    stopping_integer = 0
    while simActive:

        counter += 1
        traci.simulationStep()
        simTime += timeDelta

        if not speed_mode.is_empty():
            speed_mode.try_setting()
        
        # here we will control all vehicles
        g_active_controller.control() 

        # iterate over g_active_veh and append velocity profile
        g_active_veh = veh_in_sim[:g_active_no]
        current_vehicles = traci.vehicle.getIDList()
        for vehid in veh_in_sim[:g_active_no]:
            if vehid not in current_vehicles:
                g_active_veh.remove(vehid)

        if len(g_active_veh) == 0:
            stopping_integer += 1
        else:
            stopping_integer = 0

        if stopping_integer == 200:
            simActive = 0

        if counter % 100 == 0:
            print('still alive at ', counter)    
            print('Gactive vehicles:', g_active_veh)


    connector.disconnect()
    print('Simulations complete')
    print('DONE')


def run_simulation_var_param(control_key, percentage, win=False, horizon=10, 
    gui=False):
    
    weights = {'ADAM_DSM': 1e0, 'ADAM_MIN': 1e5, 'ADAM_TRADE': 4e3}

    # Define road model directory
    modelname = 'sellyOak'
    model = '{}/'.format(modelname)

    print('STARTING:', control_key, 'Percentage', percentage) 

    # Edit the the output filenames in sumoConfig
    configFile = "{}/{}.sumocfg".format(modelname, modelname)

    exportPath = 'test_results/'
    if not os.path.exists(model+exportPath): # this is relative to cfg
        os.makedirs(model+exportPath)

    simport = 8857  # comm port for traci
    sumo_config_gen = sumoConfigGen_win if win else sumoConfigGen 
    sumo_config_gen(modelname, configFile, exportPath,
          port=simport, 
        output_name=control_key.lower() + str(percentage).replace('.', '')) 
   
    # Connect to model
    connector = sumoConnect.sumoConnect(configFile, gui=gui, port=simport)
    connector.launchSumoAndConnect()

    # initialise the simulation
    simActive = True

    # get g_active vehicles
    
    veh_in_sim = list(traci.vehicle.getIDList())



    # SET VARIABLE PARAMETERS IN VEHICLE
    # generate list of random numbers with normal dist of 1 +- 0.1
    no_param = 5
    for_extra_vehicles = 5
    np.random.seed(42)
    rand_numbers = np.random.normal(1, 0.1, 
        size=no_param * for_extra_vehicles * len(veh_in_sim))
    # lower bound the rand_numbers by 0.5
    rand_numbers = np.clip(rand_numbers, 0.5, None)

    # new variable to track vehicles entering simulation
    param_counter = 0
    all_veh = veh_in_sim
    for veh in all_veh:
        val = traci.vehicle.getDecel(veh)
        traci.vehicle.setDecel(veh, val * rand_numbers[param_counter])
        param_counter += 1
        val = traci.vehicle.getAccel(veh)
        traci.vehicle.setAccel(veh, val * rand_numbers[param_counter])
        param_counter += 1
        val = traci.vehicle.getMaxSpeed(veh)
        traci.vehicle.setMaxSpeed(veh, val * rand_numbers[param_counter])
        param_counter += 1
        val = traci.vehicle.getMinGap(veh)
        traci.vehicle.setMinGap(veh, val * rand_numbers[param_counter])
        param_counter += 1
        val = traci.vehicle.getTau(veh)
        traci.vehicle.setTau(veh, val * rand_numbers[param_counter])
        param_counter += 1

    SEED = 448
    random.seed(SEED)
    random.shuffle(veh_in_sim)
    g_active_no = int(math.ceil(percentage * len(veh_in_sim)))
    g_active_veh = veh_in_sim[:g_active_no]
  
    g_active_controller = st.SklearnGPController(
        weights[control_key], horizon, g_active_veh)
    speed_mode = st.SpeedModeSetter(copy.deepcopy(g_active_veh))
  
    # for gathering solution
    counter = 0
    stopping_integer = 0
    while simActive:

        counter += 1
        traci.simulationStep()

        if not speed_mode.is_empty():
            speed_mode.try_setting()
        
        current_vehicles = traci.vehicle.getIDList()
        # check if there are any_new vehicles
        new_veh = set(current_vehicles).difference(set(all_veh))
        # add new_veh to all 
        all_veh += list(new_veh)
        # set random parameters for new vehicles
        for veh in new_veh:
            val = traci.vehicle.getDecel(veh)
            traci.vehicle.setDecel(veh, val * rand_numbers[param_counter])
            param_counter += 1
            val = traci.vehicle.getAccel(veh)
            traci.vehicle.setAccel(veh, val * rand_numbers[param_counter])
            param_counter += 1
            val = traci.vehicle.getMaxSpeed(veh)
            traci.vehicle.setMaxSpeed(veh, val * rand_numbers[param_counter])
            param_counter += 1
            val = traci.vehicle.getMinGap(veh)
            traci.vehicle.setMinGap(veh, val * rand_numbers[param_counter])
            param_counter += 1
            val = traci.vehicle.getTau(veh)
            traci.vehicle.setTau(veh, val * rand_numbers[param_counter])
            param_counter += 1
       
        if counter > 18000:
            simActive = 0


        # here we will control all vehicles
        g_active_controller.control() 

        # check if there are any g_active vehicles in simulation
        g_active_veh = veh_in_sim[:g_active_no]
        current_vehicles = traci.vehicle.getIDList()
        for vehid in veh_in_sim[:g_active_no]:
            if vehid not in current_vehicles:
                g_active_veh.remove(vehid)

        if len(g_active_veh) == 0:
            stopping_integer += 1
        else:
            stopping_integer = 0

        if stopping_integer == 200:
            simActive = 0

        if counter % 100 == 0:
            print('still alive at ', counter)    
            print('Gactive vehicles:', g_active_veh)


    connector.disconnect()
    print('Simulations complete')
    print('DONE')


def run_ref_var_param(win=True, horizon=1, gui=False):
    
    weights = {'ADAM_DSM': 1e0, 'ADAM_MIN': 1e5, 'ADAM_TRADE': 4e3}

    # Define road model directory
    modelname = 'sellyOak'
    model = '{}/'.format(modelname)

    print('STARTING: ref sim')

    # Edit the the output filenames in sumoConfig
    configFile = "{}/{}.sumocfg".format(modelname, modelname)

    exportPath = 'test_results/'
    if not os.path.exists(model+exportPath): # this is relative to cfg
        os.makedirs(model+exportPath)

    simport = 8857  # comm port for traci
    sumo_config_gen = sumoConfigGen_win if win else sumoConfigGen 
    sumo_config_gen(modelname, configFile, exportPath, 
        port=simport, output_name='adam_original_reference_sim') 
   
    # Connect to model
    connector = sumoConnect.sumoConnect(configFile, gui=gui, port=simport)
    connector.launchSumoAndConnect()

    # initialise the simulation
    simActive = True

    # get g_active vehicles
    
    veh_in_sim = list(traci.vehicle.getIDList())

    # SET VARIABLE PARAMETERS IN VEHICLE
    # generate list of random numbers with normal dist of 1 +- 0.1
    no_param = 5
    for_extra_vehicles = 20
    np.random.seed(42)
    rand_numbers = np.random.normal(1, 0.1, 
        size=no_param * for_extra_vehicles * len(veh_in_sim))
    # lower bound the rand_numbers by 0.5
    rand_numbers = np.clip(rand_numbers, 0.5, None)


    ### FIND TYPE OF EACH VEHICLE ################
    
    vtypes = {} # will store both id and type
    
    # iterate through all vehicles at the beginning of simulation
    for veh in veh_in_sim:
        vtypes[veh] = traci.vehicle.getTypeID(veh)

    # new variable to track vehicles entering simulation
    param_counter = 0
    all_veh = deepcopy(veh_in_sim)
    for veh in all_veh:
        val = traci.vehicle.getDecel(veh)
        traci.vehicle.setDecel(veh, val * rand_numbers[param_counter])
        param_counter += 1
        val = traci.vehicle.getAccel(veh)
        traci.vehicle.setAccel(veh, val * rand_numbers[param_counter])
        param_counter += 1
        val = traci.vehicle.getMaxSpeed(veh)
        traci.vehicle.setMaxSpeed(veh, val * rand_numbers[param_counter])
        param_counter += 1
        val = traci.vehicle.getMinGap(veh)
        traci.vehicle.setMinGap(veh, val * rand_numbers[param_counter])
        param_counter += 1
        val = traci.vehicle.getTau(veh)
        traci.vehicle.setTau(veh, val * rand_numbers[param_counter])
        param_counter += 1

    SEED = 448
    random.seed(SEED)
    random.shuffle(veh_in_sim)
    g_active_veh = deepcopy(veh_in_sim)

    with open('seeded_veh_cong.pkl', 'wb') as handle:
        pickle.dump(g_active_veh, handle, protocol=pickle.HIGHEST_PROTOCOL)

    # for gathering solution
    counter = 0
    stopping_integer = 0
    while simActive:

        counter += 1
        traci.simulationStep()
        #st.LeaderInfoGatherer('1555.3', {'1555.3': -1}).get_leader_info()

        current_vehicles = traci.vehicle.getIDList()
        # check if there are any_new vehicles
        new_veh = set(current_vehicles).difference(set(all_veh))
        
        # add new_veh to all 
        all_veh += list(new_veh)
        # set random parameters for new vehicles
        for veh in new_veh:
            # append new vehicles to vehicle type dictionary
            vtypes[veh] = traci.vehicle.getTypeID(veh)
        
            # change vehicles parameters
            val = traci.vehicle.getDecel(veh)
            traci.vehicle.setDecel(veh, val * rand_numbers[param_counter])
            param_counter += 1
            val = traci.vehicle.getAccel(veh)
            traci.vehicle.setAccel(veh, val * rand_numbers[param_counter])
            param_counter += 1
            val = traci.vehicle.getMaxSpeed(veh)
            traci.vehicle.setMaxSpeed(veh, val * rand_numbers[param_counter])
            param_counter += 1
            val = traci.vehicle.getMinGap(veh)
            traci.vehicle.setMinGap(veh, val * rand_numbers[param_counter])
            param_counter += 1
            val = traci.vehicle.getTau(veh)
            traci.vehicle.setTau(veh, val * rand_numbers[param_counter])
            param_counter += 1

        if counter > 18000:
            simActive = 0

        # check if there are any g_active vehicles in simulation
        g_active_veh = deepcopy(veh_in_sim)
        current_vehicles = traci.vehicle.getIDList()
        for vehid in veh_in_sim:
            if vehid not in current_vehicles:
                g_active_veh.remove(vehid)

        if len(g_active_veh) == 0:
            stopping_integer += 1
        else:
            stopping_integer = 0

        if stopping_integer == 200:
            simActive = 0

        if counter % 100 == 0:
            print('still alive at ', counter)    
            print('Gactive vehicles:', g_active_veh)


    connector.disconnect()
    
    # save vehicle types to pkl 
    with open('veh_types_cong.pkl', 'wb') as handle:
        pickle.dump(vtypes, handle, 
            protocol=pickle.HIGHEST_PROTOCOL)
    
    print('Simulations complete')
    print('DONE')



