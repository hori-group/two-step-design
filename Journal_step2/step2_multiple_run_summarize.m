%% Explanation
% Load files that calcurated by step2_multiple_run_design_~~.m (~~ is a method)


%% Clear workspace and figures
clear
close all


%% Code execution settings

% Result save flag (0 or 1)
TF_saving = 1; % Set 1 if you save silumation result

% Number of simulation
n_set = 3500; % number of simulation

% Load plant and cost parameters
load('../parameter_setting')


%% Figure settings
set(0,'defaultLineLineWidth',1)
set(0,'defaultAxesFontSize',11)
set(0,'defaultTextFontSize',9)
set(0,'defaultAxesFontName','Times New Roman')
set(0,'defaultTextFontName','TImes New Roman')
set(0,'defaultFigurePosition',[1200 500 300 200])
set(0,'defaultFigureColor','white')


%% Save name
version='R10_ver1_test';
basic_info_for_step1 = append(pendulum,'_',option,str_St,'_Ts',str_Ts,'_p',str_penalty);
basic_info_ACRL = append(basic_info_for_step1,'_beta',str_Beta_ACRL,'_sigma',str_sigma_ACRL,'_epi',str_tri,'_',version);
basic_info_RLalone = append(basic_info_for_step1,'_beta',str_Beta_RLalone,'_sigma',str_sigma_RLalone,'_epi',str_tri,'_',version);
basic_info_K0RL = append(basic_info_for_step1,'_beta',str_Beta_K0RL,'_sigma',str_sigma_K0RL,'_epi',str_tri,'_',version);

filename_ACRL = append('data_AC+RL_',basic_info_ACRL);
filename_RLalone = append('data_RLalone_',basic_info_RLalone);
filename_K0RL = append('data_K0RL_',basic_info_K0RL);


%% Load files
myVars = {'rew_res_loop'};

% AC + RL
rew_res_runs_matrix_ACRL = nan(n_set,n_trial); % column : number of attempts (seed_number) , row : episodes
for run_cnt = 1:n_set
    filename = append(filename_ACRL,'_',num2str(run_cnt),'.mat');
    load(filename,myVars{:});
    rew_res_runs_matrix_ACRL(run_cnt,:) = rew_res_loop;
    clear rew_res_loop 
end
rew_res_runs_matrix_ACRL(rew_res_runs_matrix_ACRL<penalty) = penalty;
rew_res_runs_matrix_ACRL(isnan(rew_res_runs_matrix_ACRL)) = penalty;
mean_cost_ACRL_list = -mean(rew_res_runs_matrix_ACRL);

% RLalone
rew_res_runs_matrix_RLalone = nan(n_set,n_trial); % column : number of attempts (seed_number) , row : episodes
for run_cnt = 1:n_set
    filename = append(filename_RLalone,'_',num2str(run_cnt),'.mat');
    load(filename,myVars{:});
    rew_res_runs_matrix_RLalone(run_cnt,:) = rew_res_loop;
    clear rew_res_loop 
end
rew_res_runs_matrix_RLalone(rew_res_runs_matrix_RLalone<penalty) = penalty;
rew_res_runs_matrix_RLalone(isnan(rew_res_runs_matrix_RLalone)) = penalty;
mean_cost_RLalone_list = -mean(rew_res_runs_matrix_RLalone);

% K0 + RL
rew_res_runs_matrix_K0RL = nan(n_set,n_trial); % column : number of attempts (seed_number) , row : episodes
for run_cnt = 1:n_set
    filename = append(filename_K0RL,'_',num2str(run_cnt),'.mat');
    load(filename,myVars{:});
    rew_res_runs_matrix_K0RL(run_cnt,:) = rew_res_loop;
    clear rew_res_loop 
end
rew_res_runs_matrix_K0RL(rew_res_runs_matrix_K0RL<penalty) = penalty;
rew_res_runs_matrix_K0RL(isnan(rew_res_runs_matrix_K0RL)) = penalty;
mean_cost_K0RL_list = -mean(rew_res_runs_matrix_K0RL);


%% Save the result
if TF_saving == 1
    save_filename = append('ToFigData_R10_ver5.mat');
    save(save_filename)
end




