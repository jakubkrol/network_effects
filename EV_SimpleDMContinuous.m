%-----------------------------------%
% BEGIN: EV_SimpleDMContinuous.m %
%-----------------------------------%
function phaseout = EV_SimpleDMContinuous(input)

CONSTANTS = input.auxdata; %Extract auxdata from the main 

%% Weighting Factor for cost function
w1 = CONSTANTS.w1;
w2 = CONSTANTS.w2;	

%% Motor Data
% Polynomical parameters
mi_p01 =    CONSTANTS.mi_p01;		% 
mi_p02 =    CONSTANTS.mi_p02;		%
mi_p03 =    CONSTANTS.mi_p03;		% 
Qmax   =    CONSTANTS.Qmax;         % Maximum battery Capacity
Tmax   =    CONSTANTS.Tmax;         % Maximum motor torque

%% Powertrain Parameters
mass       = CONSTANTS.mass      ;	% Vehicle mass (in Mg, not kg)
grav       = CONSTANTS.grav      ;	% Gravity (N./kg)
CdA        = CONSTANTS.CdA       ;	% Coefficient of drag (m.^2)
rho        = CONSTANTS.rho       ;	% Air density (Mg./m.^3)
Crr        = CONSTANTS.Crr       ;	% Coefficient of rolling resistance (kN./kN)
rw         = CONSTANTS.rw        ;	% Wheel radius (m)
Ngear      = CONSTANTS.Ngear     ;	% Gear Ratio

%% Driver Model Parameters
% delt  = CONSTANTS.delt;       % Assumed as 4 for efficiency -- James
amax   = CONSTANTS.amax;
bmax  = CONSTANTS.bmax;
vd     = CONSTANTS.vd;    
s_d    = CONSTANTS.s_d;
T_d    = CONSTANTS.T_d;
v_lead = CONSTANTS.v_lead;
t_lead = CONSTANTS.t_lead;

%% Regulation function parameters
k      = CONSTANTS.k;
% kappa1 = CONSTANTS.kappa1;
% w_th   = CONSTANTS.w_th;   % Motor speed threshold
% kappa2 = CONSTANTS.kappa2;

% %% Braking Distribusion Parameters
% La     = CONSTANTS.La;     % the length from the vehicle�s center of gravity to the front axle [m]
% L      = CONSTANTS.L;      % the wheel base [m]
% hg     = CONSTANTS.hg;     % the gravity center height of the vehicle [m]
% m      = CONSTANTS.m;      % the vehicle mass [kg]

%% Input signals 
t       = input.phase.time;
state   = input.phase.state;
control = input.phase.control;

%% State signals 
x_l = state(:,1);  % Travel distance for the lead vehicle
v   = state(:,2);  % Speed of the ego vehicle 
SoC = state(:,3);  % Battery SoC
x_e = state(:,4);  % Travel distance for the ego vehicle

%% Input signals
f   =  control(:,1); % Total force on the wheel

%% Dependent Variables
% Jakub - removed that -3
s   = x_l-x_e; %-3     % Headway Distance (length of vehicle subtracted)
ft  = f/2+sqrt(k^2.*f.^2+4)/(2*k); % Traction force on the wheel by regulation function
fb  = f-ft;        % Overall braking force

% v_lead_c = interp1(t_lead,v_lead, t,'spline'); % Lead vehicle speed

% NB: I have replaced interp1 above by a cubic griddedInterpolant object
% This should have less overhead, and be less oscillatory -- James
v_lead_c = CONSTANTS.vlead(t);

%% Motor Dynamics Calculation

% Braking Distribusion
% fbb   = -fb*1e3;  % Unit and sign conversion from [kN] to [N] and from negative to positve
% fbf   = fbb.^2*hg/m/grav/L+(L-La)/L*fbb; % Front wheel braking force [N]
% fbr   = (La/L)*fbb-fbb.^2*hg/(m*grav*L); % Rear  wheel braking force [N]
% fbf_c = -fbf/1e3; % Unit and sign conversion from [N] to [kN] and from positive to negative
% fbr_c = -fbr/1e3; % Unit and sign conversion from [N] to [kN] and from positive to negative

% F_max = Tmax/rw*Ngear; % Motor force maximum

wm    = Ngear.*v./rw;  % Motor speed

% Motor regenerative braking force on the wheel which is function of fbr and wm
% f_RG  =  -(-sqrt(kappa1^2*(-fbf_c*1e3-F_max).^2+4)+(-fbf_c*1e3+F_max)*kappa1)/2/kappa1.*(1/2+tanh(kappa2*(wm-w_th))/2)/1e3;

% Motor torque
Tm = rw.*ft./Ngear + 0.3*rw.*fb./Ngear;
% Motor current
Im = mi_p01.*Tm+mi_p02.*wm.*Tm+mi_p03.*Tm.^2;

%% Dynamics
dot_x_l = v_lead_c;							                               % Lead Vehicle Distance
dot_x_e = v;							                                   % Ego Vehicle Distance				
dot_v   = (f - 0.5.*rho.*CdA.*v.*v - Crr.*mass.*grav)./mass;			   % Acceleration with drag forces
dot_SoC = -Im/Qmax;                                                        % Battery SoC


%% Path constraints
Pmm = Tm.*wm; % Motor power 
FBM = fb*0.7;           %Mechanical Braking 
        
%% Driver Model Cost

%%% NEW VERSION - 17/03/2020 %%%
% This is ugly, but multiplication is faster than exponentiation... -- JF
vfac = v_lead_c / vd;
vfacsq = vfac .* vfac;
vfac4 = vfacsq .* vfacsq;
den = sqrt(1-min(vfac4, 0.99));
seq = (s_d + T_d.*v)./den;  % Equilibrium headway distance

vdsq = (v / vd).*(v / vd);  
vd4 = vdsq.*vdsq;  
vddelt = (vd4-1);     
alph = 8 ./ vddelt.*vddelt;

DM_s = alph.*(s./seq - 1).^2 ./ ((s ./ seq).*(s ./ seq) + 1);
DM_v = 16*(v ./ vd - 1).^2;
DM_a = (dot_v./ amax).^2;

DM = DM_a + DM_s + DM_v;

%%% END OF NEW VERSION %%%

% seq = s_d + T_d.*v;  % Desired headway distance
% vdsq = (v / vd).*(v / vd);    % added for efficiency -- James
% vd4 = vdsq.*vdsq;   % added for efficiency -- James
% vddelt = (1 - vd4);     % added for efficiency -- James
% alph = 8 ./ seq.*vddelt.*vddelt;
% % NB: repeated exponentiation is really slow, so let's exploit the fact
% % that delt = 4 and use multiplication instead -- James
% % epsi = (2 - 3 ./ 2 .* v ./vd).*delt ./ 8 .* seq.^2.*bmax.^4;
% epsi = 4*(2 - 3 ./ 2 .* v ./vd) ./ 8 .* seq.^2.*bmax.^4; % -- James
% 
% % DM_v = delt^2*(v ./ vd - 1).^2;
% DM_v = 16*(v ./ vd - 1).^2;     % -- delt = 4 -- James
% DM_a = (dot_v./ amax).^2;
% DM_s = alph.*(seq - s).^2 ./ ((s ./ epsi).*(s ./ epsi) + 1);
% 
% DM = DM_a + DM_s + DM_v;

% %% Driver Model Cost
% % Auxiliary parameters for DSM
% st = s_d + T_d.*v ./ sqrt(1 - (v./vd).^delt);   % 'desired spacing' with bias
% al = delt.^2./vd.^2;
% ep = 2.*(s_d + T_d.*v);
% % bt = 1 * 2/st^2 * (3*(v/vd)^delt + 2) * (st^2/ep^2 + 1);
% bt = 4./st.^2 .* (st.^2./ep.^2 + 1) .* ((v./vd).^delt - 1).^2;
% %bt = 1 + 3/2*(v_lead/vd)^delt;  % when using IDM space penalty
% 
% 
% DM_v = al .* (v - vd).^2; 
% DM_a = (dot_v./amax).^2;
% DM_s = bt .* (s-st).^2 ./ (s.^2./ep.^2 + 1);
% bar  = 1;
% % DM_ba =  - bar*log(1-dot_v/amax) - bar*log(1+dot_v/bmax) - bar*log(s);
% DM_ba = 0;
% DM = DM_v+ DM_a + DM_s + DM_ba; 

%% Problem formulation 

%% Weighting Factor for cost function

phaseout.dynamics = [dot_x_l, dot_v, dot_SoC, dot_x_e];
phaseout.path = [dot_v Pmm s Tm FBM];
phaseout.integrand = DM;

%---------------------------------%
% END: EV_SimpleDMContinuous.m %
%---------------------------------%
