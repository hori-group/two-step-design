%% Explanation
% This program creates a mat file that stores the parameters to be used in the simulation.


%% Clear workspace and figures
clear
close all


%% Plant
% Plant parameters
L = 0.5; % Length of the pendulum
M = 0.15; % Mass of the pendulum head
g = 9.8; % Gravitational constant
eta = 0.05; % Friction coefficient
Saturation = 0.5; % Saturation value of the torque
Ts = 0.06; % Sampling period

% Nonlinear dynamics of inverted pendulum  (discretized by Euler method)
f_u = @(x,u)( [ x(1)+Ts*x(2) ; x(2)+Ts*g/L*sin(x(1)) - Ts*eta/(M*L^2)*x(2) + Ts/(M*L^2)*u]);


%% Weighting matrices in cost function 
Q = diag([100 1]);
R = 10;


%% Linearized model
A = [1 Ts; g*Ts/L 1-eta*Ts/(M*L^2)];
B = [0; Ts/(M*L^2)];
C = [1 1];
D = 0;
sys = ss(A, B, C, D,Ts);

% LQR solution K^{\star}
[Kstar,~,~] = lqr(sys,Q,R);
Kstar = -Kstar;


%% Time settings
endTime = 3; % Simulation time 


%% Penalty for failure
penalty = -1000; % Set this if |angle|>0.5 rad when you calcurete cost


%%%%%%%%%%%%%%%%%%%% For step2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%5

%% Setting for reinforcement learning
n_trial = 4000; % Number of trial

% Initial beta and sigma
beta_ini_ACRL = 0.0001;
beta_ini_RLalone = 0.0001;
beta_ini_K0RL = 0.001;
sigma2_u_ACRL = 0.1;
sigma2_u_RLalone = 0.5;
sigma2_u_K0RL = 5;

% Other hyparparameters
alpha = 0.05; % learning rate (Step width)
gamma = .9; % discount rate
et = 1; % use eligibility trace or not( 1:use , 0:not use)
lambda_th = 0.99;  % accumulating trace of theta
lambda_W = 0.99; % accumulating traces of W

% Order
nx = 2;
nu = 1;
n_s_s = nx; % number of states of controlled object
n_s = 11; % number of states of RL for each state of controlled object

% Mean of basis function
c_i=zeros(n_s^n_s_s,n_s_s);
n_s_0=zeros(n_s,1);
for j=1:n_s
    n_s_0(j,1)=j;
end
for kk=1:n_s_s
    c_i(:,kk)=kron(ones(n_s^(kk-1),1),kron(n_s_0,ones(n_s^(n_s_s-kk),1)));
end

% variance of basis function
sigma_i=.5;

% Coefficient for standardization
c_i_max = [.5 4];
c_i_min = [-.5 -4];


%% Save name
pendulum='pnd6';
option='St';

str_Ts=ConvertForSave(Ts); 
str_St=ConvertForSave(Saturation);
str_penalty=ConvertForSave(-penalty);
str_tri=ConvertForSave(n_trial);
str_Beta_ACRL=ConvertForSave(beta_ini_ACRL); str_Beta_RLalone = ConvertForSave(beta_ini_RLalone); str_Beta_K0RL = ConvertForSave(beta_ini_K0RL);
str_sigma_ACRL=ConvertForSave(sigma2_u_ACRL); str_sigma_RLalone=ConvertForSave(sigma2_u_RLalone); str_sigma_K0RL=ConvertForSave(sigma2_u_K0RL);


%% Save
save('parameter_setting.mat')


