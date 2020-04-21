clc;clear;
%% MPC definitions
M = [0 1 0; 0 0 0];                     % z=Mx
Ts = 0.01;                              % Time Step
N =  20;                                % Prediction Horizon
q1 = [1e8 0; 0 0];                      % State weights
q2 = 1e-8;                              % Input weights
n = 3;                                  % Number of states
m = 1;                                  % Number of inputs
x0_plant = [0.177; 2e4];                % Initial values of plant
x0 = [0.177; 2e4; 0];                   % Initial values of state model

%% ENGINE parameters
R   =  2.93000e+02;  
V_im = 1.80000e-03; 
T_im =  2.93000e+02;  
eta_vol = 0.8;          
V_d  = 1.95300e-03;
N_rps   = 1980/60;      
n_r  = 2.00000e+00;
p_a  = 1.01325e+05;  
T_a  = 2.98150e+02;
tau =  0.04426;                         % Time constant for throttle
gamma = 1.4;

%% Symbolic x and u
xs = sym('x',[n 1], 'real');
us = sym('u',[m 1], 'real');

%% Effektive Area Throttle
a0  = 5.18849e-06;
a1  = -6.38593e-05;
a2  = 1.23068e-03;
A_eff = @(x,u) a0 + x(1)*a1 + x(1)*x(1)*a2;  

%% Psi
Psi_Function = @(PI) sqrt(((gamma+1)/(2*gamma))...      
    *(1-PI)*(PI+((gamma-1)/(gamma+1))));
PI_Linear = 0.99;  
Psi_1 = @(x,u) Psi_Function(1/(gamma+1));
Psi_2 = @(x,u) Psi_Function((x(2)/p_a));
Psi_3 = @(x,u) Psi_Function(PI_Linear)*(1-(x(2)/p_a))/(1-PI_Linear);
PI_Constraint = @(x,u) x(2)/p_a;

%% Mass Flow
m_dot_in_1 = @(x,u) p_a/sqrt(R*T_a)*A_eff(x,u)*Psi_1(x,u);
m_dot_in_2 = @(x,u) p_a/sqrt(R*T_a)*A_eff(x,u)*Psi_2(x,u);
m_dot_in_3 = @(x,u) p_a/sqrt(R*T_a)*A_eff(x,u)*Psi_3(x,u);
m_dot_ut = @(x,u) eta_vol*(V_d*N_rps*x(2))/(n_r*R*T_im);

%% Intake Manifold Pressure 
dp_im_1 = @(x,u) R*T_im/V_im*(m_dot_in_1(x,u) - m_dot_ut(x,u));
dp_im_2 = @(x,u) R*T_im/V_im*(m_dot_in_2(x,u) - m_dot_ut(x,u));
dp_im_3 = @(x,u) R*T_im/V_im*(m_dot_in_3(x,u) - m_dot_ut(x,u));

%% Throttle First Order System
dalpha = @(x,u) 1/tau*(u - x(1));

%% Symbolic Linearization 
f0_1 = @(x,u) [dalpha(x,u) ; dp_im_1(x,u);0];
f0_2 = @(x,u) [dalpha(x,u) ; dp_im_2(x,u);0];
f0_3 = @(x,u) [dalpha(x,u) ; dp_im_3(x,u);0];

f0_plant_1 = @(x,u) [dalpha(x,u) ; dp_im_1(x,u)];
f0_plant_2 = @(x,u) [dalpha(x,u) ; dp_im_2(x,u)];
f0_plant_3 = @(x,u) [dalpha(x,u) ; dp_im_3(x,u)];

jac_A_1 = jacobian(f0_1(xs,us),xs);                               % Creates A1 matrix 
jac_B_1 = jacobian(f0_1(xs,us),us);                               % Creates B1 matrix                                               
f_1 = matlabFunction(f0_1(xs,us),'Vars',{xs,us},...
    'File','mpcControllerf_1');
f_plant_1 = matlabFunction(f0_plant_1(xs,us),'Vars',{xs,us},...
    'File','mpcControllerf_plant_1');

jac_A_2 = jacobian(f0_2(xs,us),xs);                               % Creates A2 matrix 
jac_B_2 = jacobian(f0_2(xs,us),us);                               % Creates B2 matrix 
f_2 = matlabFunction(f0_2(xs,us),'Vars',{xs,us},...
    'File','mpcControllerf_2');
f_plant_2 = matlabFunction(f0_plant_2(xs,us),'Vars',{xs,us},...
    'File','mpcControllerf_plant_2');
PSI_2 = matlabFunction(Psi_2(xs,us),'Vars',{xs,us},...
    'File','mpcControllerPSI_2');

jac_A_3 = jacobian(f0_3(xs,us),xs);                              % Creates A3 matrix 
jac_B_3 = jacobian(f0_3(xs,us),us);                              % Creates B3 matrix                   
f_3 = matlabFunction(f0_3(xs,us),'Vars',{xs,us},...
    'File','mpcControllerf_3');
f_plant_3 = matlabFunction(f0_plant_3(xs,us),'Vars',{xs,us},...
    'File','mpcControllerf_plant_3');


A_1 = matlabFunction(jac_A_1,'vars',{xs,us},'File','mpcControllerA_1');                                      % Converts A matrix to MATLAB function
B_1 = matlabFunction(jac_B_1,'vars',{xs,us},'File','mpcControllerB_1');                                      % Converts B matrix to MATLAB function

A_2 = matlabFunction(jac_A_2,'vars',{xs,us},'File','mpcControllerA_2');                                      
B_2 = matlabFunction(jac_B_2,'vars',{xs,us},'File','mpcControllerB_2');

 
A_3 = matlabFunction(jac_A_3,'vars',{xs,us},'File','mpcControllerA_3');                                      
B_3 = matlabFunction(jac_B_3,'vars',{xs,us},'File','mpcControllerB_3'); 

PI = PI_Constraint(xs,us);                                                                                  
PI_func = matlabFunction(PI,'vars',{xs,us},'File','PI_function');
