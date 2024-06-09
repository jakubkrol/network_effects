#!/usr/bin/env python
"""
@file    signalTools.py
@author  Craig Rafter
@date    09/05/2018

class for fixed time signal control
"""
import json
import os
import re
import pickle
import random
import sys
import time

import dill
import numpy as np
import pandas as pd
import traci 
import traci.constants as tc

from collections import defaultdict
from datetime import datetime
from glob import glob
from math import atan2, degrees, ceil, hypot, floor
from psutil import cpu_count
from scipy.spatial import distance

sys.path.append('/mnt/c/Users/user/Documents/myLibs')
sys.path.append(r'C:\Users\user\Documents\myLibs')

try:
    import matlab.engine
except:
    print('Did not manage to import the matlab engine.')

from run_gp_sklearn_4d import gp_receding_horizon
#    from run_gp_receding_horizon_4d import gp_receding_horizon

class BaseInfoGatherer(object):
    
    def __init__(self, id_list):
        self.id_list = id_list 
        self.time_series = {}

    def append(self, timestamp=None):
        if timestamp is None:
            timestamp = traci.simulation.getCurrentTime() / 1000.
        self.time_series[timestamp] = self._get_values_from_network()

    def save(self, filepath):
        pd.DataFrame.from_dict(self.time_series).T.to_pickle(filepath)

    def _get_values_from_network(self):
        traci_id_list = self._get_network_info()
        ids_to_append = self.id_list if len(self.id_list) > 0 else traci_id_list
        temp_dict = {id_: (self._method(id_) if id_ in traci_id_list else None)\
                for id_ in ids_to_append}
        return temp_dict
    
    def _get_time_invariant_info(self):
        raise NotImplementedError

    def _method(self, id_):
        raise NotImplementedError

    def _get_network_info(self):
        raise NotImplementedError


class VehicleInfoGatherer(BaseInfoGatherer):

    def __init__(self, *args, **kwargs):
        super(VehicleInfoGatherer, self).__init__(*args, **kwargs)

    def _method(self, id_):
        return {'Distance': traci.vehicle.getDistance(id_), 
                'Speed': traci.vehicle.getSpeed(id_),
                'Acceleration': self._get_vehicle_acceleration(id_),
                'Emissions': self._get_vehicle_emissions(id_)}
    
    @staticmethod
    def _get_vehicle_emissions(id_):
        return {'CO2': traci.vehicle.getCO2Emission(id_),
                'CO': traci.vehicle.getCOEmission(id_),
                'HC': traci.vehicle.getHCEmission(id_),
                'NOx': traci.vehicle.getNOxEmission(id_),
                'PMx': traci.vehicle.getPMxEmission(id_),
                'Fuel': traci.vehicle.getFuelConsumption(id_)}

    @staticmethod
    def _get_network_info():
        return traci.vehicle.getIDList()

    def _get_vehicle_acceleration(self, id_):
        past_timestamp, current_timestamp = self._get_timestamps()
        past_speed, current_speed = self._get_speeds(id_)
        return (current_speed - past_speed)/(current_timestamp-past_timestamp)

    def _get_timestamps(self):
        return traci.simulation.getCurrentTime()/1000. - \
            traci.simulation.getDeltaT() / 1000., traci.simulation.getCurrentTime()/1000.
 
    def _get_speeds(self, id_):
        past_speed = 0 if len(self.time_series) == 0\
            else self._get_past_speed(id_)
        return past_speed, traci.vehicle.getSpeed(id_)
 
    def _get_past_speed(self, id_):
        last_timestamp = max(self.time_series.keys())
        return self.time_series[last_timestamp][id_]['Speed'] if\
            self.time_series[last_timestamp].get(id_) is not None else 0  


class LaneInfoGatherer(BaseInfoGatherer):

    def __init__(self, *args, **kwargs):
        super(LaneInfoGatherer, self).__init__(*args, **kwargs)
        self.time_inv_info = self._get_time_invariant_info()
    
    def save(self, filepath):
        super(LaneInfoGatherer, self).save(filepath[0])
        self.time_inv_info.to_pickle(filepath[1])

    def _method(self, id_):
        available_vehicles = traci.lane.getLastStepVehicleIDs(id_)
        return {'Vehicle Info' : {veh_id : self._get_vehicle_info(veh_id) for veh_id in 
            available_vehicles}, 'Emission Info': self._get_lane_emissions(id_)}    
 
    def _get_vehicle_info(self, id_):
        return {'Distance': traci.vehicle.getDistance(id_), 
                'Speed': traci.vehicle.getSpeed(id_),
                'Acceleration': self._get_vehicle_acceleration(id_),
                'Length': traci.vehicle.getLength(id_)}

    def _get_vehicle_acceleration(self, id_):
        if len(self.time_series.keys()) > 0:
            return self._calculate_acceleration(id_)
        else:
            return 0

    def _calculate_acceleration(self, id_):
        past_timestamp, current_timestamp = self._get_timestamps()
        past_speed, current_speed = self._get_speeds(id_)
        return (current_speed - past_speed)/(current_timestamp-past_timestamp)

    def _get_timestamps(self):
        return max(self.time_series.keys())/1000.,\
            traci.simulation.getCurrentTime()/1000.
 
    def _get_speeds(self, id_):
        return self._get_past_speed(id_), traci.vehicle.getSpeed(id_)
 
    def _get_past_speed(self, id_):
        last_timestamp = max(self.time_series.keys())
        lane_id = traci.vehicle.getLaneID(id_)
        temp_dict =  self.time_series[last_timestamp][lane_id]['Vehicle Info']
        return temp_dict[id_]['Speed'] if id_ in temp_dict.keys() else 0

    @staticmethod       
    def _get_network_info():
        return traci.lane.getIDList()

    def _get_time_invariant_info(self):
        out_df = pd.DataFrame()
        for id_ in self.id_list:
            out_df[id_] = pd.Series([traci.lane.getLength(id_), 
                traci.lane.getWidth(id_)], index = ['Length', 'Width'])
        return out_df

    @staticmethod
    def _get_lane_emissions(id_):
        return {'CO2': traci.lane.getCO2Emission(id_),
                'CO': traci.lane.getCOEmission(id_),
                'HC': traci.lane.getHCEmission(id_),
                'NOx': traci.lane.getNOxEmission(id_),
                'PMx': traci.lane.getPMxEmission(id_)}


class InductionLoopInfoGatherer(BaseInfoGatherer):
    KEYS = ['ID', 'Length', 'Entry Time', 'Exit Time', 'Type']

    def __init__(self, *args, **kwargs):
        super(InductionLoopInfoGatherer, self).__init__(*args, **kwargs)

    def _method(self, id_):
        return {'Vehicle Data' : self._convert_to_dict(traci.inductionloop.\
           getVehicleData(id_)), 'Mean Speed' : traci.inductionloop.\
           getLastStepMeanSpeed(id_)}

    @staticmethod
    def _get_network_info():
        return traci.inductionloop.getIDList()

    def _convert_to_dict(self, ind_loop_list):
        return [self._add_keys(ind_loop) for ind_loop in ind_loop_list]

    def _add_keys(self, single_list):
        return {key: value for key, value in zip(self.KEYS, single_list)}

class GlobalInfoGatherer(object):

    CLASS_MAP = {
            'vehicles' : VehicleInfoGatherer,
            'lanes' : LaneInfoGatherer,
            'induction_loops': InductionLoopInfoGatherer
            }
    
    FUNCTION_MAP = {
            'vehicles' : traci.vehicle,
            'lanes': traci.lane,
            'induction_loops': traci.inductionloop
            }

    def __init__(self, config_path):
        self.config = self._read_config(config_path)
        self._initialise_objects()
        self._get_file_locations() 
    
    @staticmethod
    def _read_config(config_path):
        with open(config_path) as json_data:
            return json.load(json_data)

    def _initialise_objects(self):
        self.object_list = []
        for key, value in self.config.items():
            self.object_list.append(self._class_map(key, value)) 

    def _class_map(self, key, value):
        return self.CLASS_MAP[key](value['ids']) if len(value['ids']) != 0\
            else self.CLASS_MAP[key](self.FUNCTION_MAP[key].getIDList())
                
    def append(self, timestamp=None):
        # iterate through object
        for instance in self.object_list:
            instance.append(timestamp)

    def save(self):
        for filepath, instance in zip(self.save_filepaths, self.object_list):
            instance.save(filepath)

    def _get_file_locations(self):
        self.save_filepaths = []
        for value in self.config.values():
            self.save_filepaths.append(value['filepath'])


class VehicleVelocityController(object):
    
    def __init__(self, veh_list, max_time=None):
        self.veh_list = veh_list
        self.max_time = max_time

    def control_till_max_time(self): 
        if traci.simulation.getCurrentTime() < self.max_time:
            self.control()
        else:
            self.reset()

    def control(self, control_funct = None):
        if control_funct is None:
            control_funct = self._control_method
        self._assign_speed(control_funct)
        self._set_speed()

    def _control_method(self, veh_id, *args, **kwargs):
        raise NotImplementedError

    def _assign_speed(self, method):
        veh_in_network = self._check_if_inside_network(self.veh_list)
        self.veh_speed = {veh_id : method(veh_id) for veh_id in veh_in_network}

    def _set_speed(self):
        for veh_id in self.veh_speed.keys():
            set_speed = max(self.veh_speed[veh_id], 1e-7)
            traci.vehicle.setSpeed(veh_id, set_speed)
    
    @staticmethod
    def _check_if_inside_network(veh_ids):
        id_list = traci.vehicle.getIDList()
        return [veh_id for veh_id in veh_ids if veh_id in id_list]

    def reset(self):
        self.control(lambda x:-1)


class GActiveOptimisedController(VehicleVelocityController):

    NAME_MAP = {1: 'sol_space_dsm_60.pkl', 4000: 'sol_space_trade_60.pkl',
        100000: 'sol_space_min_60.pkl'} 

    def __init__(self, opt_weight, horizon, *args, **kwargs):
        super(GActiveOptimisedController, self).__init__(*args, **kwargs)
        self.opt_weight = opt_weight
        self.horizon = horizon
        self._initialise_solutions()
        self._get_solution_space()
        self.old_leaders = {veh_id: '' for veh_id in self.veh_list}

    def _initialise_solutions(self):    
        self.counter = {veh_id: 0 for veh_id in self.veh_list}
        self.current_solution = {veh_id: None for veh_id in self.veh_list}

    def _get_solution_space(self):
        with open(self.NAME_MAP[int(self.opt_weight)], 'rb') as handle:
            self.sol_space = dill.load(handle)

    def _control_method(self, veh_id):
        if self.counter[veh_id] % self.horizon == 0:
            info = self._gather_info_for_optimisation(veh_id)
            self._solve_optimisation(info, veh_id)

        if self.horizon == 1:
            sol_out = self.current_solution[veh_id]
        else:
            sol_out = self.current_solution[veh_id][self.counter[veh_id] % \
                self.horizon]
            self.counter[veh_id] += 1
        # return sol_out
        return min(sol_out, traci.vehicle.getAllowedSpeed(veh_id))

    def _gather_info_for_optimisation(self, veh_id):
        ve = self._get_ego_speed(veh_id)
        vl, headway, self.lead_id, self.old_leaders = self._get_leader_info(veh_id)
        v_max = self._get_lane_max_speed(veh_id)
        return [headway, ve, vl, v_max]
        
    @staticmethod
    def _get_ego_speed(veh_id):
        return traci.vehicle.getSpeed(veh_id)

    @staticmethod
    def _get_lane_max_speed(veh_id):
        return traci.vehicle.getAllowedSpeed(veh_id)
#        try:
#            lane = traci.vehicle.getLaneID(veh_id)
#        except:
#            print('Failed for', veh_id)
#            lane = traci.vehicle.getLaneID(veh_id)
#        return traci.lane.getMaxSpeed(lane)
    
    def _get_leader_info(self, veh_id):
        return LeaderInfoGatherer(veh_id, self.old_leaders).get_leader_info()

    def _solve_optimisation(self, info, veh_id):
        pass
 

class LeaderInfoGatherer(object):

    def __init__(self, veh_id, old_leaders):
        self.ego_id = veh_id
        self.old_leaders = old_leaders         

    def get_leader_info(self):
        self._get_info_required_to_find_leader() 
        self._choose_the_case()
        
        if self.case == 'sumo_leader':
            self.vel_lead, self.headway, self.lead_id = self._run_sumo_leader()
        if self.case == 'red_empty':
            self.vel_lead, self.headway, self.lead_id = self._run_red_empty()
        if self.case == 'green_empty':
            #self.vel_lead, self.headway, self.lead_id = self._run_green_empty()
            self.vel_lead, self.headway, self.lead_id = self._run_green_empty_lanes()
        if self.case == 'has_leader':
            self.vel_lead, self.headway, self.lead_id = self._run_has_leader()
        if self.case == 'has_leader_on_next_junction':
            self.vel_lead, self.headway, self.lead_id = self._run_has_leader_on_next_junction()

        if self.ego_id == -1: 
            print('Leader', self.lead_id, self.case,'headway', self.headway, 
                'lead_vel', self.vel_lead, 'max speed',
                traci.vehicle.getAllowedSpeed(self.ego_id))
            print('Sumo leader and headway', 
                traci.vehicle.getLeader(self.ego_id))
            print('Follower', self.ego_id, 'ego_vel', 
                traci.vehicle.getSpeed(self.ego_id),
                'currently at', 
                traci.vehicle.getLaneID(self.ego_id),
                traci.vehicle.getLanePosition(self.ego_id))
            try:
                print('Old leader', self.old_leaders[self.ego_id], 
                    'currently at', 
                     traci.vehicle.getLaneID(self.old_leaders[self.ego_id]),
                     traci.vehicle.getLanePosition(self.old_leaders[self.ego_id]))
            except:
                pass
            print('New lane information', traci.vehicle.getBestLanes(self.ego_id))
            print()
        try:
            self.old_leaders[self.ego_id] = self.lead_id
        except:
            print(self.case)
            print(self.lead_id)
            self.old_leaders[self.ego_id] = self.lead_id

        # this can be deleted later
        # self.append_info_file()

        return self.vel_lead, self.headway, self.lead_id, self.old_leaders

    def append_info_file(self):
        if self.ego_id == '1555.3':
            # load file
            vel_profile_path = r'C:\Users\user\Documents\Gaussian Processes EcoDriving'\
                r'\sellyOak\test_results\intermediary_results\vel_profiles'
            with open(os.path.join(vel_profile_path, 'extra_info.pkl'), 'rb') as handle:
                data = pickle.load(handle)
            #construct dictionary of data 
            timestep_data = {'headway': self.headway, 
                'lead_vel': self.vel_lead, 'allowed_speed': traci.vehicle.getAllowedSpeed(self.ego_id)}
            data[traci.simulation.getTime()] = timestep_data
            # save the data
            with open(os.path.join(vel_profile_path, 'extra_info.pkl'), 'wb') as handle:
                pickle.dump(data, handle, protocol=pickle.HIGHEST_PROTOCOL)

    def _get_info_required_to_find_leader(self):
        # need to get some info about leaders and traffic lights
        self.lane = traci.vehicle.getLaneID(self.ego_id)
        distance = traci.vehicle.getLanePosition(self.ego_id)
        self.vehicles = traci.lane.getLastStepVehicleIDs(self.lane)
        distances = [traci.vehicle.getLanePosition(veh) for veh in \
             self.vehicles]
        self.distances = np.array(distances)
        self.leader_distances = self.distances > distance
        self.next_tl = traci.vehicle.getNextTLS(self.ego_id)

    def _choose_the_case(self):
        # check which case is appropriate
        self.case = 'has_leader'
        if np.sum(self.leader_distances) == 0:
            self.case = 'green_empty'
        if np.sum(self.leader_distances) == 0 and len(self.next_tl) > 0:
            self.next_tl = self.next_tl[0]
            # check if we are first at the red light queue
            if self.lane in traci.trafficlight.getControlledLanes(self.next_tl[0]) \
                and self.next_tl[-1].lower() == 'r':
                self.case = 'red_empty'
        if self.case == 'green_empty':
            # append the headway and leader with what is happening ahead
            old_lead_veh = self.old_leaders[self.ego_id]
            if old_lead_veh in traci.vehicle.getIDList():
                old_lead_id = traci.vehicle.getLaneID(self.old_leaders[self.ego_id])
                if old_lead_id[:5] == ':junc' or old_lead_id[:5] == ':node':
                    self.case = 'has_leader_on_next_junction'     

        if traci.vehicle.getLeader(self.ego_id) is not None:
            self.case = 'sumo_leader'   

    def _run_sumo_leader(self):
        self.lead_id, self.headway = traci.vehicle.getLeader(self.ego_id)
        self.headway += 2.7 # is this a bug in sumo ?
        self.vel_lead = traci.vehicle.getSpeed(self.lead_id)
        return self.vel_lead, self.headway, self.lead_id
 
    def _run_red_empty(self):
        self.vel_lead = 0
        self.headway = traci.lane.getLength(self.lane) - \
            traci.vehicle.getLanePosition(self.ego_id)
        self.lead_id = 'Red TL'
        return self.vel_lead, self.headway, self.lead_id

    def _run_green_empty_lanes(self):
        self.vel_lead = traci.lane.getMaxSpeed(self.lane)
        self.headway = traci.lane.getLength(self.lane) - traci.vehicle.\
            getLanePosition(self.ego_id)
        self.lead_id = None
        # find route
        options = traci.vehicle.getBestLanes(self.ego_id)
        for option in options:
            if option[3] == 0:
                route = option[-1][1:] if option[-1][0] == \
                    traci.vehicle.getLaneID(self.ego_id) else option[-1]
        leader = False
        for lane in route:
            vehicles_lane = traci.lane.getLastStepVehicleIDs(lane)
            if len(vehicles_lane) > 0:
                leader = True
                distances_lane = [traci.vehicle.getLanePosition(veh) for veh \
                    in vehicles_lane]
                index_min = distances_lane.index(min(distances_lane))
                self.lead_id = vehicles_lane[index_min] 
                self.vel_lead = traci.vehicle.getSpeed(self.lead_id)
                self.headway += min(distances_lane) - \
                    traci.vehicle.getLength(vehicles_lane[index_min])
            else:
                self.headway += traci.lane.getLength(lane)

            if leader:
                break

        # we need to check if there are any red lights on the way
        index_tl = 0
        no_red_tl = True
        all_tls = traci.vehicle.getNextTLS(self.ego_id)
        while no_red_tl and index_tl < len(all_tls):
            red_tl = all_tls[index_tl]
            if red_tl[-1] == 'r' and red_tl[-2] < self.headway:
                self.headway = red_tl[-2]
                self.vel_lead = 0.
                self.lead_id = 'Red TL'
            index_tl += 1 
        return self.vel_lead, self.headway, self.lead_id


    def _run_green_empty(self):
        #self.vel_lead = traci.vehicle.getAllowedSpeed(self.ego_id)
        self.vel_lead = traci.lane.getMaxSpeed(self.lane)
        self.headway = traci.lane.getLength(self.lane) - traci.vehicle.\
            getLanePosition(self.ego_id)
        self.lead_id = None
        route = traci.vehicle.getRoute(self.ego_id)
        index_r = traci.vehicle.getRouteIndex(self.ego_id)
        no_leader = True
        if index_r + 1 < len(route): # if its last index do not do this step
            while no_leader:
                index_r += 1
                if index_r + 1 == len(route):
                    no_leader = False # we want it to be the last iteration
                current_edge = route[index_r]
                vehicles_edge = traci.edge.getLastStepVehicleIDs(current_edge)
                if len(vehicles_edge) > 0:
                    distances_edge = [traci.vehicle.getLanePosition(veh) for veh in \
                            vehicles_edge]
                    index_min = distances_edge.index(min(distances_edge))
                    self.lead_id = vehicles_edge[index_min] 
                    self.vel_lead = traci.vehicle.getSpeed(self.lead_id)
                    self.headway += min(distances_edge) - traci.vehicle.getLength(vehicles_edge[index_min])
                    no_leader = False 
                else:
                    lane_next = route[index_r] + '_0'
                    self.headway += traci.lane.getLength(lane_next)

        # we need to check if there are any red lights on the way
        index_tl = 0
        no_red_tl = True
        all_tls = traci.vehicle.getNextTLS(self.ego_id)
        while no_red_tl and index_tl < len(all_tls):
            red_tl = all_tls[index_tl]
            if red_tl[-1] == 'r' and red_tl[-2] < self.headway:
                self.headway = red_tl[-2]
                self.vel_lead = 0.
                self.lead_id = 'Red TL'
            index_tl += 1 
        return self.vel_lead, self.headway, self.lead_id

    def _run_has_leader(self):
        index_lead = np.where(self.distances ==\
            np.min(self.distances[self.leader_distances]))
        veh_interest = self.vehicles[index_lead[0][0]]
        self.vel_lead = traci.vehicle.getSpeed(veh_interest)
        self.headway = traci.vehicle.getLanePosition(veh_interest) -\
            traci.vehicle.getLanePosition(self.ego_id) - traci.vehicle.\
            getLength(veh_interest)   
        self.lead_id = veh_interest
        return self.vel_lead, self.headway, self.lead_id

    def _run_has_leader_on_next_edge(self):  
        self.headway = traci.lane.getLength(self.lane) - traci.vehicle.\
            getLanePosition(self.ego_id)
        # append the headway and leader with what is happening ahead
        route = traci.vehicle.getRoute(self.ego_id)
        index_r = traci.vehicle.getRouteIndex(self.ego_id)
        if index_r + 1 == len(route):
            return self.vel_lead, self.headway, self.lead_id
        edge_next = route[index_r + 1]
            
        vehicles_edge = traci.edge.getLastStepVehicleIDs(edge_next)
        distances_edge = [traci.vehicle.getLanePosition(veh) for veh in \
                vehicles_edge]
 
        index_min = distances_edge.index(min(distances_edge))
        veh_interest = vehicles_edge[index_min] 
        self.vel_lead = traci.vehicle.getSpeed(veh_interest)
        self.headway += min(distances_edge) - traci.vehicle.getLength(vehicles_edge[index_min])
        self.headway = max(0, self.headway)

        self.lead_id = veh_interest
        return self.vel_lead, self.headway, self.lead_id

    def _run_has_leader_on_next_junction(self):  
        self.headway = traci.lane.getLength(self.lane) - traci.vehicle.\
            getLanePosition(self.ego_id)
        veh_interest = self.old_leaders[self.ego_id]
        self.headway += traci.vehicle.getLanePosition(veh_interest) - \
            traci.vehicle.getLength(veh_interest)
        self.headway = max(0, self.headway)

#        if self.ego_id == '1635.0':
#            print('Term1', self.headway, 'term2', traci.vehicle.getLanePosition(veh_interest),
#                'term3', traci.vehicle.getLength(veh_interest), 'veh_interest', veh_interest) 
#            print('summation', self.headway)                
#
        self.vel_lead = traci.vehicle.getSpeed(veh_interest)
        self.lead_id = veh_interest
        return self.vel_lead, self.headway, self.lead_id        


class SklearnGPController(GActiveOptimisedController):

    NAME_MAP = {1: 'gp_sol_space_dsm_sklearn.pkl', 
        4000: 'gp_sol_space_trade_sklearn.pkl',
        100000: 'gp_sol_space_min_sklearn.pkl'} 

    def __init__(self, *args, **kwargs):
        super(SklearnGPController, self).__init__(*args, 
            **kwargs)

    def _get_solution_space(self):
        with open(self.NAME_MAP[int(self.opt_weight)], 'rb') as handle:
            self.sol_space = pickle.load(handle)

    def _solve_optimisation(self, info, veh_id):
        threshold = 0.01
        try:
            self.current_solution[veh_id], self.sol_space = gp_receding_horizon(
                self.opt_weight, info[0], info[1], info[2], 
                info[3], self.sol_space, self.horizon, lead_id=self.lead_id, 
                ego_id=veh_id, gp_treshold=threshold)
        except:
            try:
                self.current_solution[veh_id], self.sol_space = gp_receding_horizon(
                   self.opt_weight, info[0] + random.random() / 1000, 
                   info[1] + random.random() / 1000, 
                   info[2] - random.random() / 1000, 
                   info[3], self.sol_space, self.horizon, lead_id=self.lead_id, 
                   ego_id=veh_id, gp_treshold=threshold)
            except:
                try:
                    self.current_solution[veh_id], self.sol_space = gp_receding_horizon(
                        self.opt_weight, info[0] + random.random() / 1000, 
                        info[1] + random.random() / 1000, 
                        info[2] + random.random() / 1000, 
                        info[3], self.sol_space, self.horizon, lead_id=self.lead_id, 
                        ego_id=veh_id, gp_treshold=threshold)
                except:
                    try:
                        self.current_solution[veh_id], self.sol_space = gp_receding_horizon( 
                            self.opt_weight, info[0] + random.random() / 1000, 
                            info[1] + random.random() / 1000, 
                            info[2] + random.random() / 1000, 
                            info[3], self.sol_space, self.horizon, lead_id=self.lead_id, 
                            ego_id=veh_id, gp_treshold=threshold)

                    except:
                        self.current_solution[veh_id], self.sol_space = gp_receding_horizon(
                            self.opt_weight, info[0] + random.random() / 1000, 
                            info[1] + random.random() / 1000, 
                            info[2] + random.random() / 1000, 
                            info[3], self.sol_space, self.horizon, lead_id=self.lead_id, 
                            ego_id=veh_id, gp_treshold=threshold)


class GaussianProcessesController(GActiveOptimisedController):

    NAME_MAP = {1: 'gp_sol_space_dsm_4d.pkl', 4000: 'gp_sol_space_trade_4d.pkl',
        100000: 'gp_sol_space_min_4d.pkl'} 

    def __init__(self, *args, **kwargs):
        super(GaussianProcessesController, self).__init__(*args, 
            **kwargs)

    def _get_solution_space(self):
        with open(self.NAME_MAP[int(self.opt_weight)], 'rb') as handle:
            self.sol_space = dill.load(handle)

    def _condition_to_reload(self):
        now = datetime.now()
        return datetime.hour == 9 and datetime.minute < 3
  
    def _solve_optimisation(self, info, veh_id):
        if self._condition_to_reload():
            self._get_solution_space()
        threshold = 0.1	
        try:
            self.current_solution[veh_id] = gp_receding_horizon(
                self.opt_weight, info[0], info[1], info[2], 
                info[3], self.sol_space, self.horizon, lead_id=self.lead_id, 
                ego_id=veh_id, gp_treshold=threshold)
        except:
            try:
                self.current_solution[veh_id] = gp_receding_horizon(
                   self.opt_weight, info[0] + random.random() / 1000, 
                   info[1] + random.random() / 1000, 
                   info[2] - random.random() / 1000, 
                   info[3], self.sol_space, self.horizon, lead_id=self.lead_id, 
                   ego_id=veh_id, gp_treshold=2*threshold)
            except:
                try:
                    self.current_solution[veh_id] = gp_receding_horizon(
                        self.opt_weight, info[0] + random.random() / 1000, 
                        info[1] + random.random() / 1000, 
                        info[2] + random.random() / 1000, 
                        info[3], self.sol_space, self.horizon, lead_id=self.lead_id, 
                        ego_id=veh_id, gp_treshold=3*threshold)
                except:
                    try:
                        self.current_solution[veh_id] = gp_receding_horizon( 
                            self.opt_weight, info[0] + random.random() / 1000, 
                            info[1] + random.random() / 1000, 
                            info[2] + random.random() / 1000, 
                            info[3], self.sol_space, self.horizon, lead_id=self.lead_id, 
                            ego_id=veh_id, gp_treshold=4*threshold)

                    except:
                        self.current_solution[veh_id] = gp_receding_horizon(
                            self.opt_weight, info[0] + random.random() / 1000, 
                            info[1] + random.random() / 1000, 
                            info[2] + random.random() / 1000, 
                            info[3], self.sol_space, self.horizon, lead_id=self.lead_id, 
                            ego_id=veh_id, gp_treshold=5*threshold)

    @staticmethod
    def add_input_to_be_solved_in_matlab(input_):
        today = datetime.now()
        save_name = 'mat_sol_' + str(today.month) + str(today.day) + '.pkl'
        save_path = r'C:\Users\user\Documents\Gaussian Processes EcoDriving\sp'
        full_name = os.path.join(save_path, save_name)
        if os.path.exists(full_name):
            with open(full_name, 'rb') as handle:
                sol_space = pickle.load(handle)
        else:
            sol_space = {'headways': [], 'ego': [], 'lead': [], 'max': []}           
        sol_space['headways'].append(input_[0])
        sol_space['ego'].append(input_[1])
        sol_space['lead'].append(input_[2])
        sol_space['max'].append(input_[3])
        with open(full_name, 'wb') as handle:
            pickle.dump(sol_space, handle, protocol=pickle.HIGHEST_PROTOCOL)


class GActiveOptimisedControllerWithMaps(GActiveOptimisedController):

    NAME_MAP = {1: 'sol_space_dsm_60.pkl', 4000: 'sol_space_trade_60.pkl',
        100000: 'sol_space_min_60.pkl'} 

    def __init__(self, *args, **kwargs):
        super(GActiveOptimisedControllerWithMaps, self).__init__(*args, 
            **kwargs)

    def _solve_optimisation(self, info, veh_id):
        self.current_solution[veh_id], self.sol_space =\
            gp_receding_horizon(self.opt_weight, info[0], info[1], info[2], 
            info[3], self.sol_space, self.horizon)



class RandomAdditiveController(VehicleVelocityController):
    
    def __init__(self, freq, amplitude, *args, **kwargs):
        super(RandomAdditiveController, self).__init__(*args, **kwargs)
        self.counter_dict = {veh_id : 0 for veh_id in self.veh_list}
        self.freq = freq
        self.amplitude = amplitude

    def _control_method(self, veh_id):
        self.counter_dict[veh_id] += 1
        ref_vel = traci.vehicle.getSpeed(veh_id)
        return ref_vel + self.amplitude * np.random.random() if\
            (self.counter_dict[veh_id] + 1) % self.freq == 0 else ref_vel


class PlatoonLookUpController(VehicleVelocityController):
    def __init__(self, adam_key, *args, **kwargs):
        super(PlatoonLookUpController, self).__init__(*args, **kwargs)
        self.data = pd.read_pickle(adam_key + '.pkl')
   
    def _control_method(self, veh_id):
        timestamp = traci.simulation.getCurrentTime()/1000.
        return self.data[veh_id].loc[timestamp]


class ModeSetter(object):
    def __init__(self, ids_to_set):
        self.ids_to_set = ids_to_set

    def try_setting(self):
        self.to_remove = set()
        for id_ in self.ids_to_set:
            self._try_to_set(id_)
        self._remove_already_set()
    
    def _try_to_set(self, id_):
        if id_ in traci.vehicle.getIDList():
            self._method(id_)
            self.to_remove.add(id_)

    def _remove_already_set(self):
        for id_ in self.to_remove:
            self.ids_to_set.remove(id_)

    def is_empty(self):
        return len(self.ids_to_set) == 0

    def _method(self, id_):
        raise NotImplementedError

def get_json_file(route_file, key):
    json_to_save = {"vehicles":{"ids": [], "filepath": route_file[:-8] + \
        '_' + key.lower() + '.pkl'}}
    with open(route_file[:-8] + '.json', 'w') as outfile:  
        json.dump(json_to_save, outfile) 

class LaneChangeSetter(ModeSetter):
    def __init__(self, *args, **kwargs):
        super(LaneChangeSetter, self).__init__(*args, **kwargs)
    
    def _method(self, id_):
        traci.vehicle.setLaneChangeMode(id_, 513)


class SpeedModeSetter(ModeSetter):
    def __init__(self, *args, **kwargs):
        super(SpeedModeSetter, self).__init__(*args, **kwargs)
    
    def _method(self, id_):
        traci.vehicle.setSpeedMode(id_, 24)


def getIntergreen(dist):
    # <10 & 10-18 & 19-27 & 28-37 & 38-46 & 47-55 & 56-64 & >65
    #  5  &   6   &   7   &   8   &   9   &  10   &  11   & 12
    diamThresholds = [10, 19, 28, 38, 47, 56, 65]
    intergreen = 5
    for threshold in diamThresholds:
        if dist < threshold:
            return intergreen
        else:
            intergreen += 1
    return intergreen


def getIntergreenTime(junctionID):
    juncPos = traci.junction.getPosition(junctionID)
    edges = traci.trafficlights.getControlledLinks(junctionID)
    edges = [x for z in edges for y in z for x in y[:2]]
    edges = list(set(edges))
    boundingCoords = []
    for edge in edges:
        dMin, coordMin = 1e6, []
        for laneCoord in traci.lane.getShape(edge):
            dist = getDistance(juncPos, laneCoord)
            if dist < dMin:
                dMin, coordMin = dist, laneCoord
        boundingCoords.append(coordMin)
    # get max of closest edge pairwise distances
    dMax = np.max(distance.cdist(boundingCoords, boundingCoords))
    return getIntergreen(dMax)


def getSUMOHeading(currentLoc, prevLoc):
    dy = currentLoc[1] - prevLoc[1]
    dx = currentLoc[0] - prevLoc[0]
    if currentLoc[1] == prevLoc[1] and currentLoc[0] == prevLoc[0]:
        heading = -1
    else:
        if dy >= 0:
            heading = degrees(atan2(dy, dx))
        else:
            heading = 360 + degrees(atan2(dy, dx))
    
    # Map angle to make compatible with SUMO heading
    if 0 <= heading <= 90:
        heading = 90 - heading
    elif 90 < heading < 360:
        heading = 450 - heading

    return heading


def unique(sequence):
    return list(set(sequence))


def mean(x):
    return sum(x)/float(len(x))


def getIncomingLaneInfo(controlledLanes):
    laneInfo = {}
    for lane in unique(controlledLanes):
        shape = traci.lane.getShape(lane)
        width = traci.lane.getWidth(lane)
        heading = getSUMOHeading(shape[-1], shape[0])
        x1, y1 = shape[0]
        x2, y2 = shape[-1]
        dx = abs(x2 - x1) 
        dy = abs(y2 - y1)
        if dx > dy:
            y1 += width
            y2 -= width
        else: 
            x1 += width
            x2 -= width
        laneInfo[lane] = {'heading': heading, 
                          'bounds': {'x1': x1, 'y1': y1,
                                     'x2': x2, 'y2': y2}
                         }

    return laneInfo


def getRouteDict():
    fileNames = glob('../2_models/VALIDROUTES/*.rou.xml')
    models = []
    regex = re.compile('.+edges="(.+?)"')
    routeDict = {}

    for fileName in fileNames:
        file = open(fileName, 'r')
        model = os.path.basename(fileName).split('_')[0]
        routeDict[model] = []
        for line in file:
            match = regex.match(line)
            if not match:
                continue
            else:
                routeDict[model].append(match.groups()[0].split())
        file.close()

    return routeDict


def isInRange(vehPosition, scanRange, jcnGeometry):
    center, JCR = jcnGeometry # jcnPos, jcnCtrlRegion
    distance = hypot(*(vehPosition - center))
    c1 = distance < scanRange
    # shorten variable name and check box is in bounds
    c2 = JCR['W'] <= vehPosition[0] <= JCR['E']
    c3 = JCR['S'] <= vehPosition[1] <= JCR['N']
    return (c1 and c2 and c3)


# default dict that finds and remembers road speed limits (only if static)
# needs to be updated otherwise
class speedLimDict(defaultdict):
    def __missing__(self, key):
        self[key] = traci.lane.getMaxSpeed(key)
        return self[key]


# defaultdict that finds and remembers vehicle types (only if static)
# needs to be updated otherwise
class vTypeDict(defaultdict):
    def __missing__(self, key):
        self[key] = traci.vehicle.getTypeID(key)
        return self[key]


def getDistance(A, B):
    x1, y1 = A
    x2, y2 = B
    return hypot(x1-x2, y1-y2)


def flatten(listOfLists):
    return [elem for subList in listOfLists for elem in subList]


class StopCounter(object):
    def __init__(self):
        self.stopSubscription()  # makes self.subkey
        self.stopCountDict = defaultdict(int)

    def stopSubscription(self):
        self.subkey = traci.edge.getIDList()[0]
        traci.edge.subscribeContext(self.subkey, 
                                    tc.CMD_GET_VEHICLE_VARIABLE, 
                                    1000000, 
                                    varIDs=(tc.VAR_WAITING_TIME,))

    def getStops(self):
        subResults = traci.edge.getContextSubscriptionResults(self.subkey)
        vtol = 1e-3
        wait = tc.VAR_WAITING_TIME
        try:
            for vehID in subResults.keys():
                if 0.099 < subResults[vehID][wait] < 0.101:
                    self.stopCountDict[vehID] += 1
        except KeyError:
            pass
        except AttributeError:
            pass

    def writeStops(self, filename):
        with open(filename, 'w') as f:
            f.write('vehID,stops\n')
            vehIDs = self.stopCountDict.keys()
            vehIDs.sort()
            for vehID in vehIDs:
                f.write('{},{}\n'.format(vehID, self.stopCountDict[vehID]))


class simTimer(object):
    def __init__(self):
        self.startTime = 0
        self.stopTime = 0
        self.started = False
        self.stopped = False

    def start(self):
        if not self.started:
            self.startTime = time.time()
            self.started = True
        else:
            print('WARNING: Timer already started')

    def stop(self):
        if self.started and not self.stopped:
            self.stopTime = time.time()
            self.stopped = True
        else:
            print('WARNING: Timer already stopped/not active')

    def runtime(self):
        if self.started and self.stopped: 
            return self.stopTime - self.startTime
        elif self.started and not self.stopped:
            return time.time() - self.startTime
        else:
            print('WARNING: Timer not active')
            return -1

    def strTime(self):
        return time.strftime("%dd %X", time.gmtime(self.runtime()))


def getNproc(mode='best'):
    mode = mode.lower()
    physical = cpu_count(logical=False)
    logical = cpu_count()
    if mode == 'best':
        return np.mean([physical, logical], dtype=int)
    elif mode in ['max', 'logical']:
        return logical
    elif mode in ['phy', 'physical']:
        return physical
    elif mode == 'low':
        return max(1, int(physical/2.))
    else:
        return 1


def isSimGridlocked(model, timeMS):

    timeHours = timeMS/3600000.0
    forceSimEndTime = 40.0 if 'selly' in model else 6.0

    if timeHours >= forceSimEndTime:
        print('TIMEOUT: {} >= {} on {}'.format(timeHours, forceSimEndTime, model))
        sys.stdout.flush()
        return True

    try:
        vehIDs = traci.vehicle.getIDList()
        isStationary = []
        isWaiting = []
        for vID in vehIDs:
            isStationary.append(traci.vehicle.getSpeed(vID) < 0.1)
            # vehicle is waiting too long if all cycles complete and still blocked
            isWaiting.append(traci.vehicle.getWaitingTime(vID) > 500.0)


        if all(isStationary) and all(isWaiting):
            meanStationary = np.mean(isStationary)
            meanWaiting = np.mean(isWaiting)
            if 'nan' in [str(meanStationary), str(isWaiting)]:
                return False
            print('GRIDLOCK: all vehicles stationary, hour: {}'.format(timeHours))
            print('GRIDLOCK: stopped {} waiting {}'.format(meanStationary, meanWaiting))
            sys.stdout.flush()
            return True
        else:
            return False
    except:
        print(isStationary)
        print(isWaiting)
        return False


def lane2edge(lanes):
    if type(lanes) is str:
        return unique([lanes.split('_')[0]])
    elif type(lanes) is list:
        return unique([x.split('_')[0] for x in lanes])
    else:
        raise TypeError("lanes not list or string")

def edge2lanes(edges, laneNdict):
    lanes = []
    if type(edges) is str:
        eList = [edges]
    else:
        eList = edges

    for edge in eList:
        for i in range(laneNdict[edge]):
            lanes.append(edge+'_'+str(i))
    return lanes

def getLaneNumbers():
    edges = traci.edge.getIDList()
    lanes = traci.lane.getIDList()
    laneNdict = defaultdict(int)
    for edge in edges:
        for lane in lanes:
            if edge == lane.split('_')[0]:
                laneNdict[edge] += 1
    return laneNdict

def edgeLaneMap():
    eldict = defaultdict(list)
    edges = traci.edge.getIDList()
    lanes = traci.lane.getIDList()
    for edge in edges:
        for lane in lanes:
            if edge == lane.split('_')[0]:
                eldict[edge].append(lane)
    return eldict

def nearRound(x, base):
    fb = float(base)
    return round(float(x)/fb)*fb

def floorRound(x, base):
    fb = float(base)
    return floor(float(x)/fb)*fb

def ceilRound(x, base):
    fb = float(base)
    return ceil(float(x)/fb)*fb
