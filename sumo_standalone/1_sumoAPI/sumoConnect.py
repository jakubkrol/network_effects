#!/usr/bin/env python
"""
@file    sumoConnect.py
@author  Simon Box
@date    31/01/2013

Parent class for signal control algorithms

"""
import platform, sys, traci
from sumolib import checkBinary

class sumoConnect(object):
    
    def __init__(self, pathToConfig, gui, port=8813):
        self.setPort(port)
        programme = ""
        if gui:
            programme = "sumo-gui"
        else:
            programme = "sumo"
            
        self.sumoBinary = programme
        self.sumoConfig = pathToConfig
        if platform.system() == 'Windows':
            self.sumoBinary = checkBinary(programme)
            
        self.isConnected = False
        
    
    def launchSumoAndConnect(self):
        traci.start([self.sumoBinary, "-c", self.sumoConfig])
        self.isConnected = True
                    
    def runSimulationForSeconds(self, seconds):
        start = self.getCurrentSUMOtime()
        while (self.getCurrentSUMOtime() - start) < (seconds*1000):
            traci.simulationStep()
    
    def runSimulationForOneStep(self):
            traci.simulationStep()
            
    def getCurrentSUMOtime(self):
        return traci.simulation.getCurrentTime()
    
    
    def disconnect(self):
        self.isConnected = False
        traci.close()
        sys.stdout.flush()
    
        
    def setPort(self, port):
        self.Port = port
