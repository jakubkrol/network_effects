function output = EV_SimpleDMMain_func(W1, t_lead, v_lead, v_0, ...
    x_e_0, x_l_0, v_d, s_d, T_d, SoC_0)
% INPUTS:
% Wl - weight corresponding to MIN, TRADE, DSM
% t_lead - time to get the solution for
% v_lead - velocity profile of the leader (constant)
% v_0 - initial ego velocity
% x_e_0 - initial position of ego veh, should be zero
% x_l_0 - headway, get from SUMO
% v_max - maximum velocity on the road
% s_d, T_d, SoC_0 - parameters for driver model, initial State of Charge


w1_s = W1;
w2_s = 1;
w3_s = W1 / (1e6);      % terminal cost weighting, scales with energy penalty -- James
auxdata.v_a = v_d;

% %-------------------------------------------------------------------%
% %---- All Data Required by User Functions is Stored in AUXDATA -----%
% %-------------------------------------------------------------------%
%% Weighting Factor for cost function
auxdata.w1 = w1_s;
auxdata.w2 = w2_s;	
auxdata.w3 = w3_s;	
	
%% Motor Data  [Nissan Leaf]
% Polynomical parameters
auxdata.mi_p01 =    0.001983 * 1e3;		% conversion N -> kN added
auxdata.mi_p02 =    0.004723 * 1e3;		% conversion N -> kN added
auxdata.mi_p03 =    0.00071  * 1e6;		% conversion N -> kN added
auxdata.Qmax   =    24000;              % Maximum battery Capacity
Tmax           =      320;              % Maximum motor torque [N]
auxdata.Tmax   =     Tmax;              % Maximum motor torque
Pmax           =      110;              % Maximum power of motor [kW]

%% Powertrain Parameters  [Nissan Leaf]
% Car dynamical model - NB: mf in g, f in kN, others in SI base units
auxdata.mass   = 1.5;			        % Vehicle mass (in Mg, not kg)
auxdata.grav   = 9.81;		            % Gravity (N/kg)
auxdata.CdA    = 0.7;			        % Coefficient of drag (m^2)
auxdata.rho    = 1.225*1e-3;		    % Air density (Mg/m^3)
auxdata.Crr    = 0.01;		            % Coefficient of rolling resistance (kN/kN)
auxdata.rw     = 0.4;		         	% Wheel radius (m)
auxdata.Ngear  = 7.94;		            % Gear ratio (fixed for now - later include in fuel map)

%% Driver Model Parameters
auxdata.delt   = 4;
bmax = 4;
amax = 2.5;
auxdata.amax   = amax;
auxdata.bmax   = bmax;
auxdata.vd     = v_d;    
auxdata.s_d    = s_d;
auxdata.T_d    = T_d;

auxdata.v_lead = v_lead;
auxdata.t_lead = t_lead;


%% Regulation function parameters
auxdata.k      = 30; 
auxdata.kappa1 = 0.5;
auxdata.kappa2 = 1;
auxdata.w_th   = 20;   % Motor speed threshold


%% Braking Distribusion Parameters
auxdata.La     = 0.95; % the length from the vehicle�s center of gravity to the front axle [m]
auxdata.L      = 2.7;  % the wheel base [m]
auxdata.hg     = 0.5;  % the gravity center height of the vehicle [m]
auxdata.m      = auxdata.mass*1e3;  % the vehicle mass [kg]

%% GriddedInterpolant object for v_lead interpolation -- James

% must have equally-spaced grid to use 'cubic', so resample:
N_resamp = 100;     % number of resampling points
t_resamp = linspace(t_lead(1), t_lead(end), N_resamp);
v_resamp = interp1(t_lead, v_lead, t_resamp, 'linear');

% Now create the interpolant, to be reused by the callback functions
auxdata.vlead = griddedInterpolant(t_resamp,v_resamp,'cubic');


%% Load 

t0 = t_lead(1);
tf = t_lead(end);

x_l_min = 0;
x_l_max = 1e4;

x_e_min = 0;
x_e_max = 1e4;

SoC_min = 0.1;
SoC_max = 0.9;

f_min = -1e10;
f_max = 1e10;

v_min = 0;
v_max = 50;

x0_l = [x_l_0  , v_0  , SoC_0,   x_e_0];
x0_u = [x_l_0  , v_0  , SoC_0,   x_e_0];
xf_l = [x_l_min  , v_min, SoC_min, x_e_min];
xf_u = [x_l_max  , v_max, SoC_max, x_e_max];
xMin = [x_l_min, v_min, SoC_min, x_e_min];
xMax = [x_l_max, v_max, SoC_max, x_e_max];

uMin =   f_min;
uMax =   f_max;
%-------------------------------------------------------------------------%
%----------------------- Setup for Problem Bounds ------------------------%
%-------------------------------------------------------------------------%
bounds.phase.initialtime.lower = t0;
bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower = tf;
bounds.phase.finaltime.upper = tf;
bounds.phase.initialstate.lower = x0_l;
bounds.phase.initialstate.upper = x0_u;
bounds.phase.state.lower = xMin;
bounds.phase.state.upper = xMax;
bounds.phase.finalstate.lower = xf_l;
bounds.phase.finalstate.upper = xf_u;
bounds.phase.control.lower = uMin;
bounds.phase.control.upper = uMax;
bounds.phase.integral.lower = 0;
bounds.phase.integral.upper = 10e10;


% phaseout.path = [dot_v Pmm s Tm FBM];
bounds.phase.path.lower=[-bmax    -Pmax        0   -Tmax/1e3    -7 * auxdata.mass];
bounds.phase.path.upper=[amax      Pmax      1e3    Tmax/1e3   0];

%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%
load('Guess.mat')

% guess.phase.time     = [t0; tf];
% guess.phase.state    = [x0_l; xf_l];
% guess.phase.control  = [ 0; 0];
% guess.phase.integral = 0;
guess.phase.time     = time_g;
guess.phase.state    = state_g;
guess.phase.control  = control_g;
guess.phase.integral = integral_g;

%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method          = 'hp-LiuRao-Legendre';
mesh.tolerance       = 1e-3;
mesh.maxiterations   = 10;
mesh.colpointsmin    = 4;
mesh.colpointsmax    = 10;
mesh.phase.colpoints = 4*ones(1,10);
mesh.phase.fraction  = 0.1*ones(1,10);

%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%
%d
%-------------------------------------------------------------------------%
setup.name                           = 'EV_SimpleDM';
setup.functions.continuous           = @EV_SimpleDMContinuous;
setup.functions.endpoint             = @EV_SimpleDMEndpoint;
setup.displaylevel                   = 2;
setup.bounds                         = bounds;
setup.guess                          = guess;
setup.mesh                           = mesh;
setup.nlp.solver                     = 'ipopt';
setup.auxdata                        = auxdata;
setup.nlp.ipoptoptions.linear_solver = 'ma57';
setup.nlp.ipoptoptions.tolerance     = 1e-9;
% setup.derivatives.supplier           = 'adigator';
setup.derivatives.supplier           = 'sparseCD';
setup.derivatives.derivativelevel    = 'second';
setup.method                         = 'RPM-Differentiation';
setup.nlp.ipoptoptions.maxiterations = 2000;
%-------------------------------------------------------------------------%
%---------------------- Solve Problem Using GPOPS2 -----------------------%
%-------------------------------------------------------------------------%
tic
output = gpops2(setup);
toc

solution = output.result.solution;

time = solution.phase(1).time;
state = solution.phase(1).state;

%% State signals
v   = state(:,2);  % Speed of the ego vehicle
output = vertcat(time, v);
% figure()
% plot(time, v)

end

