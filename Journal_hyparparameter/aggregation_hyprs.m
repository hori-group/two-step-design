%% Explanation
% Summarizes the data computed in main_inv_pend_iniR_ACRL.m


%% Clear workspace and figures
clear
close all


%% Measure time
tic


%% Code execution settings

% Select an aproach in three
type='AC'; % KAC+RL
%type='RLalone';
%type='K0'; % K0+RL

% hyparparameters
beta_list2 = [0.000005,0.00001,0.00005,0.0001,0.0005,0.001,0.005];% ascending-order　
sigma_list2 = [5,1,0.5,0.1,0.05,0.01];% descending-order
n_set = 3500; % Number of simulations
n_tri_cost = 100; % number of trials

% Load average of the cost using ACalone
myVars = {'mean_cost_KAC'};
load('../Journal_step1/step1_compare_K.mat', myVars{:})
cost_ACalone_rand = mean_cost_KAC;

% load seed file
myVars = {'rand_ini_ang_matrix_1','rand_ini_vel_matrix_1'};
load('../rand_list_1.mat',myVars{:})

% Load plant and cost parameters
load('../parameter_setting')

% Setting for a method
if strcmp(type,'AC')
    % save name
    savename = 'aggregation_ACRL.mat';   
    % load KAC
    myVars2 = {'KAC'};
    load('../Journal_step1/step1_design_KAC.mat',myVars2{:})
    K = KAC;
    LQR = 1; % Use input from LQR or not (1: use, 0: not use)
    RL = 1; % Use input from RL controller or not (1: use, 0: not use)
elseif strcmp(type,'RLalone')
    % save name
    savename = 'aggregation_RLalone.mat'; 
    LQR = 0; % use input from LQR or not (1: use, 0: not use)
    RL = 1; % Use input from RL controller or not (1: use, 0: not use)
elseif strcmp(type,'K0')
    % save name
    savename = 'aggregation_K0RL.mat';
    % load K0
    myVars2 = {'K0'};
    load('../Journal_step1/step1_design_KAC.mat',myVars2{:})
    K = K0;
    LQR = 1; % Use input from LQR or not (1: use, 0: not use)
    RL = 1; % Use input from RL controller or not (1: use, 0: not use)
end


%% Calculate cost and summarize
filename_list = []; % Store summarized data
NNN=1;
for Beta = beta_list2
    for sigma = sigma_list2
        SummaryData = SummaryOfSimulationData; % Class for summarized data
        filename1 = append('beta',ConvertForSave(Beta),'_sigma',ConvertForSave(sigma));
        SummaryData.filename = filename1;
        data10.filename = append('beta',ConvertForSave(Beta),'_sigma',ConvertForSave(sigma),'_');
        mean_cost_runs_list = nan(1,100);
        for NN = 1:n_set
            filename2 = append('data_ACRL_iniR_',filename1,'_',string(NN),'.mat');

            load(filename2)
            %% Calcurate average of the cost

            rew_res_loop = nan(1,n_tri_cost);

            for epi = 1:n_tri_cost
                x_res = nan(1+endTime/Ts,nx);
                ff_for_rand = 0; % flag of fault : ff=1 if control is failed

                ini_ang = rand_ini_ang_matrix_1(epi,1);
                ini_vel = rand_ini_vel_matrix_1(epi,1);
                x = [ini_ang ; ini_vel];
                x0 = [ini_ang ; ini_vel];
                x_res(1,:) = x;
                rew_res_for_rand = -x0' * Q * x0;

                for k = 1:endTime/Ts+1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                    if k > 1
                        u_00 = u_AC;
                    end

                    %% input from model-based controller

                    if LQR == 1
                        u_AC = K*x;
                    else
                        u_AC = 0;
                    end


                    %% Reinforcement learning
                    if RL == 1

                        xx_0 = x;

                        xx = nan(n_s_s,1);
                        phi_i_kai = nan(n_s^n_s_s,1);
                        for j = 1:n_s_s
                            xx(j,1) = (xx_0(j)-c_i_min(j))*(n_s-1)/(c_i_max(j)-c_i_min(j))+1; % normalize each state of the controlled object
                        end

                        if k > 1 % Failure Confirmation
                            if abs(x(1)) > .5
                                reward = penalty;
                                ff_for_rand = 1;
                            else
                                u_00RL = u_00 + u_RL;
                                reward = -(x.'*Q*x+(u_00RL).'*R*(u_00RL));
                            end
                            rew_res_for_rand = rew_res_for_rand+reward;
                        end

                        if k > 1
                            for l = 1:n_s^n_s_s
                                phi_i_kai(l) = exp(-(xx-c_i(l,:).').'*(xx-c_i(l,:).')/(2*sigma_i^2));
                            end
                            phi_i = phi_i_kai;
                        end

                        if k == 1
                            for l = 1:n_s^n_s_s
                                phi_i(l) = exp(-(xx-c_i(l,:).').'*(xx-c_i(l,:).')/(2*sigma_i^2));
                            end
                        end

                        if k > 1
                            if ff_for_rand == 1
                                rew_res_for_rand = penalty;
                                break
                            end
                        elseif k == endTime/Ts+1
                            break
                        end
                        %% select input
                        mu_u = (phi_i.'*W_i).';
                        u_RL = repmat(mu_u,1,1); % control without exploration at the final episode

                    else
                        u_RL = 0;
                        mu_u = 0;
                    end

                    % input
                    u = u_AC + u_RL;
                    u_no_saturation = u_AC + u_RL;

                    if u < -Saturation
                        u = -Saturation;
                    elseif u > Saturation
                        u = Saturation;
                    end
                    x = f_u(x,u);
                    x_res(k+1,:) = x.';

                end %%%%%%%%%%%%%%%%%for 1:endtime/dt+1
                if rew_res_for_rand < penalty
                    rew_res_for_rand = penalty;
                end
                if isnan(rew_res_for_rand)
                    rew_res_for_rand = penalty;
                end

                rew_res_loop(:,epi) = rew_res_for_rand;
            end

            rew_res_mean = mean(rew_res_loop);
            x = 0;

            disp(['Cost    :',num2str(-rew_res_mean)])
            mean_cost = -rew_res_mean;
            mean_cost_runs_list(1,NN) = mean_cost;
            SummaryData.Cost_runs_list(NN) = mean_cost;
            if mean_cost <= cost_ACalone_rand
                SummaryData.Cost_PoI_runs_list(NN) = mean_cost;
            else
                SummaryData.Cost_PoI_runs_list(NN) = 0;
            end
            if mean_cost >= 1000
                SummaryData.ff_runs_list(NN) = 1;
            else
                SummaryData.ff_runs_list(NN) = 0;
            end
            if NN == 100
                SummaryData.mean_cost_all_runs = mean(mean_cost_runs_list);
            end
            clear mean_cost x u u_AC u_00 u_00RL
        end
        SummaryData.N_PoI = nnz(SummaryData.Cost_PoI_runs_list); % Number of improve
        SummaryData.N_PoS = 100 - nnz(SummaryData.ff_runs_list); % Number of failures
        NNN=NNN+1;
        filename_list = [filename_list,SummaryData];

    end
end

clear(myVars{:})


%% Save the result
save(savename)


%% Measure time
toc


