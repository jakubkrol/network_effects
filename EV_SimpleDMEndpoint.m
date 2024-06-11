%---------------------------------%
% BEGIN: EV_SimpleDMEndpoint.m %
%---------------------------------%
function output = EV_SimpleDMEndpoint(input)

CONSTANTS = input.auxdata; %Extract auxdata from the main

%% Weighting Factor for cost function
w1 = CONSTANTS.w1;
w2 = CONSTANTS.w2;
w3 = CONSTANTS.w3;
v_a = CONSTANTS.v_a;
% initialstate   = input.phase.initialstate;
finalstate = input.phase.finalstate;
initialtime   = input.phase.initialtime;
finaltime = input.phase.finaltime;

% SoC_ini = initialstate(:,3);
SoC_fnl = finalstate(:,3);

% x_e_ini = initialstate(:,4);
v_e_fnl = finalstate(:,2);
x_e_fnl = finalstate(:,4);

Horizon = finaltime-initialtime;
%output.objective = w1*(-SoC_fnl)+w2*input.phase.integral./Horizon-w3*(v_e_fnl-v_a);

output.objective = w1*(-SoC_fnl) + w2*input.phase.integral./Horizon + w3*(x_e_fnl-v_a*finaltime).^2;
%---------------------------------%
% BEGIN: EV_SimpleDMEndpoint.m %
%---------------------------------%
