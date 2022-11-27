%% Explanation
% This program makes random number list "rand_list_1.mat"
% Angle : -0.4~0.4
% Angular velocity : -1~1


%% Clear and close figures
clear
close all


%% Generate random number
rng(1)

n_epi = 4010;
runs = 5000;

rand_ini_ang_matrix_1 = -0.4+0.8*rand(n_epi,runs); % For initial angle
rand_ini_vel_matrix_1 = -1+2*rand(n_epi,runs); % For initial Angular velocity　


%% Save
filename = 'rand_list_1.mat';
save(filename,'-v7.3')




