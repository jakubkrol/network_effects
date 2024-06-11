# -*- coding: utf-8 -*-
"""
@file    simpleTest.py
@author  Simon Box, Craig Rafter
@date    29/01/2016

test Miller's algorithm
"""
import sys
import os
sys.path.insert(0, '../1_sumoAPI')
import sumoConnect
import traci
from sumoConfigGen import sumoConfigGen

# Define road model directory
modelname = 'sellyOak'
modelBase = modelname.split('_')[0]
model = '../2_models/{}/'.format(modelBase)

# Edit the the output filenames in sumoConfig
configFile = model + modelBase + ".sumocfg"
exportPath = '../../4_simulation/test_results/'
if not os.path.exists(model+exportPath): # this is relative to cfg
    os.makedirs(model+exportPath)

simport = 8857  # comm port for traci
sumoConfigGen(modelname, configFile, exportPath, 
              port=simport)

# Connect to model
connector = sumoConnect.sumoConnect(configFile, gui=True, port=simport)
connector.launchSumoAndConnect()

# Step simulation while there are vehicles
simActive = True

counter = 0 
while simActive:
    counter += 1
    traci.simulationStep()

    if counter % 120 == 0:
        simActive = traci.simulation.getMinExpectedNumber()
        
    print(traci.vehicle.getNextTLS("65.0"))
    traci.vehicle.setSpeed("65.0", 0.1)    
    # if counter == 1000:
        # simActive = False
        
#system_info.save()

connector.disconnect()
print('Simulations complete')
print('DONE')

