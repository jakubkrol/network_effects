The main file required to run simulation is var_param_sim.py.

Please note that in order to run the simulation GPOPS-II licence and SUMO installations are required.

More details regarding installation is coming soon...

The GPR models that were obtained as a result of running the simulation are not included here due to their size but are available on request. 

### SUMO Installation

Below is the link to the SUMO installation guide

https://sumo.dlr.de/docs/Installing/index.html

We have included a sumo standalone folder which allows to run the simulation in the Selly Oak area of Birmingham without inclusion of eco-vehicles. In order to run the simulation please run simpleTest.py file in the 4_simulation folder. It is possible that some paths will have to be added so that the file is able to find the sellyOak model files that are located in 2_models/sellyOak folder.

### `EV_SimpleDMMain_func.m`

This MATLAB function simulates the dynamics and control of an electric vehicle (EV) following a lead vehicle, optimizing for parameters such as energy consumption and safety. The script utilizes dynamic optimization techniques and is built for integration with the GPOPS-II optimal control software.

#### Function Signature
```matlab
function output = EV_SimpleDMMain_func(W1, t_lead, v_lead, v_0, x_e_0, x_l_0, v_d, s_d, T_d, SoC_0)

Parameters:
- W1: Weighting factor affecting the optimization objective.
- t_lead: Time vector for the simulation duration.
- v_lead: Velocity profile of the lead vehicle.
- v_0: Initial velocity of the ego vehicle.
- x_e_0: Initial position of the ego vehicle.
- x_l_0: Initial relative distance from the lead vehicle.
- v_d, s_d, T_d: Driving parameters for the ego vehicle.
- SoC_0: Initial state of charge of the battery.

Outputs
- output: A matrix where the first column is time and the second column is the velocity profile of the ego vehicle.

Features
- Implements detailed vehicle dynamics including motor characteristics and aerodynamic properties based on parameters of a Nissan Leaf.
- Includes a driver model to simulate realistic driving behavior.
- Utilizes multiple weighting factors to balance objectives like energy efficiency and following distance.
- Provides comprehensive setup for optimization problem boundaries, initial guesses, and mesh refinement.
- Integrates vehicle physics calculations and state estimation in a modular, scalable manner.
