%% Explanation
% This result is in Table IV
% Calcurate mean of cost after step2
% display the conclusion


%% Clear workspace and figures
clear
close all


%% Measure simulation time
tic


%% Code execution settings

% Result save flag (0 or 1)
save_num = 1; % Set 1 if you save silumation result

% load seed file
myVars = {'rand_ini_ang_matrix_1','rand_ini_vel_matrix_1'};
load('../rand_list_1.mat', myVars{:})

% load KAC and K0
myVars2 = {'KAC','K0'};
load('../Journal_step1/step1_design_KAC.mat', myVars2{:})

% Number of simulation
n_set = 3500; % Number of simulation
n_ini_state = 100; % number of calcuratting cost at one controller

% Load plant and cost parameters
load('../parameter_setting')


%% Save name
version='R10_ver1_test';
str_Beta_ACRL=ConvertForSave(beta_ini_ACRL); str_Beta_RLalone = ConvertForSave(beta_ini_RLalone); str_Beta_K0RL = ConvertForSave(beta_ini_K0RL);
str_sigma_ACRL=ConvertForSave(sigma2_u_ACRL); str_sigma_RLalone=ConvertForSave(sigma2_u_RLalone); str_sigma_K0RL=ConvertForSave(sigma2_u_K0RL);
str_Ts=ConvertForSave(Ts); str_penalty=ConvertForSave(-penalty); str_St=ConvertForSave(Saturation); str_epi=ConvertForSave(n_trial);
basic_info_for_step1 = append(pendulum,'_',option,str_St,'_Ts',str_Ts,'_p',str_penalty);
basic_info_ACRL = append(basic_info_for_step1,'_beta',str_Beta_ACRL,'_sigma',str_sigma_ACRL,'_epi',str_epi,'_',version);
basic_info_RLalone = append(basic_info_for_step1,'_beta',str_Beta_RLalone,'_sigma',str_sigma_RLalone,'_epi',str_epi,'_',version);
basic_info_K0RL = append(basic_info_for_step1,'_beta',str_Beta_K0RL,'_sigma',str_sigma_K0RL,'_epi',str_epi,'_',version);

filename_ACRL = append('data_AC+RL_',basic_info_ACRL);
filename_RLalone = append('data_RLalone_',basic_info_RLalone);
filename_K0RL = append('data_K0RL_',basic_info_K0RL);


%% calcurate cost
% type_num == 1 -> method is AC + RL
% type_num == 2 -> method is RL alone
% type_num == 3 -> method is K0 + RL

for type_num = 1:3
    rew_res_runs_matrix = nan(n_set,n_ini_state); % Stores each cost
    for set_cnt = 1:n_set
        % Setting of type
        if type_num == 1
            % AC + RL
            load_file = append(filename_ACRL,'_',num2str(set_cnt),'.mat');
            LQR = 1;
            K = KAC;
        elseif type_num == 2
            % RL alone
            load_file = append(filename_RLalone,'_',num2str(set_cnt),'.mat');
            LQR = 0;
        elseif type_num == 3
            % K0 + RL
            load_file = append(filename_K0RL,'_',num2str(set_cnt),'.mat');
            LQR = 1;
            K = K0;
        end
        myVars = {'W_i'};
        load(load_file,myVars{:});

        % Calculate each cost
        for epi = 1:n_ini_state
            % Variable Preparation
            phi_i = nan(n_s^n_s_s,1);
            phi_i_kai = nan(n_s^n_s_s,1);
            ff_for_rand = 0; % flag of fault : ff=1 if control is failed

            % Initial state
            ini_ang = rand_ini_ang_matrix_1(epi,1);
            ini_vel = rand_ini_vel_matrix_1(epi,1);
            x = [ini_ang ; ini_vel];
            x0 = [ini_ang ; ini_vel];
            rew_res_for_rand = -x0' * Q * x0;

            for k = 1:endTime/Ts+1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                %% Input from linear controller
                if k > 1
                    u_00 = u_AC;
                end

                if LQR == 1
                    u_AC = K*x;
                else
                    u_AC = 0;
                end


                %% Reinforcement learning
                RL = 1;
                if RL == 1

                    xx_0 = x;

                    xx = nan(n_s_s,1);
                    phi_i_kai = nan(n_s^n_s_s,1);
                    for j = 1:n_s_s
                        xx(j,1) = (xx_0(j)-c_i_min(j))*(n_s-1)/(c_i_max(j)-c_i_min(j))+1; % normalize each state of the controlled object
                    end

                    % Failure Confirmation
                    if k > 1 
                        if abs(x(1)) > .5
                            reward = penalty;
                            ff_for_rand = 1;
                        else
                            u_00RL = u_00 + u_RL;
                            reward = -(x.'*Q*x+(u_00RL).'*R*(u_00RL));
                        end
                        rew_res_for_rand = rew_res_for_rand+reward;
                    end

                    % Update phi
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
                    
                    % In case of failure
                    if k > 1
                        if ff_for_rand == 1
                            rew_res_for_rand = penalty;
                            break
                        end
                    elseif k == endTime/Ts+1
                        break
                    end

                    % select input
                    mu_u = (phi_i.'*W_i).';
                    u_RL = repmat(mu_u,1,1); % control without exploration at the final episode
                
                % RL = 0
                else
                    u_RL = 0;
                    mu_u = 0;
                end

                % input
                u = u_AC + u_RL;

                if u < -Saturation
                    u = -Saturation;
                elseif u > Saturation
                    u = Saturation;
                end

                % next state
                x = f_u(x,u);

            end %%%%%%%%%%%%%%%%%for 1:endtime/dt+1

            % If the cost is more than the penalty
            if rew_res_for_rand < penalty
                rew_res_for_rand = penalty;
            end

            % If the controller design calculation fails
            if isnan(rew_res_for_rand)
                rew_res_for_rand = penalty;
            end

            rew_res_runs_matrix(set_cnt,epi) = rew_res_for_rand; % Stores each cost
        end
        x = 0;
        mean_cost_current_set = -mean(rew_res_runs_matrix(set_cnt,:));

        % Notice of Calculation Status
        if rem(set_cnt,50) == 0
            disp([num2str(type_num),'  ',num2str(set_cnt),' set'])
            disp(['mean of the cost',num2str(mean_cost_current_set)])
        end
    end

    Mean_cost = -mean(mean(rew_res_runs_matrix));

    % Store results
    if type_num == 1
        ave_cost_ACRL = Mean_cost;
        rew_res_runs_matrix_ACRL = rew_res_runs_matrix;
    elseif type_num == 2
        ave_cost_RLalone = Mean_cost;
        rew_res_runs_matrix_RLalone = rew_res_runs_matrix;
    elseif type_num == 3
        ave_cost_K0RL = Mean_cost;
        rew_res_runs_matrix_K0RL = rew_res_runs_matrix;
    end
    clear Mean_cost rew_res_runs_matrix
end


%% Save and display the results
if save_num
    clear rand_ini_ang_matrix_1 rand_ini_vel_matrix_1
    save_file = 'compare_cost_after_step2.mat';
    save(save_file)
end

disp(['Cost by AC+RL     :',num2str(ave_cost_ACRL)])
disp(['Cost by RL alone     :',num2str(ave_cost_RLalone)])
disp(['Cost by K0+RL     :',num2str(ave_cost_K0RL)])


%% Measure time
toc


