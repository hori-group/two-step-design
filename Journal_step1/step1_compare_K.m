%% Explanation
% This is the code to calcurate average of the cost by KAC, Kstar or K0
% K0 is same as $K^{init}$ at jornal
% calcurate mean of cost by KAC or K0
% display the conclusion


%% Clear workspace and figures
clear
close all


%% Code execution settings

% Result save flag (0 or 1)
save_num = 1; % Set 1 if you save silumation result

% Load seed file for initial state
myVars = {'rand_ini_ang_matrix_1','rand_ini_vel_matrix_1'};
load('../rand_list_1.mat',myVars{:})

% Load KAC and K0
myVars2 = {'KAC','K0','Kstar'};
load('step1_design_KAC',myVars2{:})

% Load plant and cost parameters
load('../parameter_setting')


%% Time settings
n_set = 100; % Number of set of simulation


%% Control and calculate cost

% ii=1 -> Calculate by KAC
% ii=2 -> Calculate by K0
% ii=3 -> Calculate by Kstar

K = KAC;
for ii = 1:3
    rew_res_loop = nan(1,n_set); % Store result of cost at each initial state
    for epi = 1:n_set
        ff = 0; % Flag of fault : ff=1 if control is failed

        % Set initial state
        ini_ang = rand_ini_ang_matrix_1(epi,1);
        ini_vel = rand_ini_vel_matrix_1(epi,1);

        x = [ini_ang ; ini_vel];
        rew_res = -x' * Q * x; % Cumulative reward = -cost

        for k = 1:endTime/Ts+1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            if k > 1
                u_00 = u_AC;
            end

            %% input from model-based controller
            u_AC = K*x;

            % Failure Confirmation
            if k > 1
                if abs(x(1)) > .5
                    reward = penalty;
                    ff = 1;
                    break
                else
                    u_00RL = u_00;
                    reward = -(x.'*Q*x+(u_00RL).'*R*(u_00RL));
                end
                rew_res = rew_res+reward;
            end
            if k == endTime/Ts+1
                break
            end

            % Calculate input
            u = u_AC;
            if u < -Saturation
                u = -Saturation;
            elseif u > Saturation
                u = Saturation;
            end

            % Calculate next state
            x = f_u(x,u);

        end %%%%%%%%%%%%%%%%%for 1:endtime/Ts+1
        rew_res_loop(:,epi) = rew_res;
    end

    average_cost = -mean(rew_res_loop);

    % Display average of the cost and set next simulation
    if ii == 1
        disp(['Cost by KAC    :',num2str(average_cost)])
        mean_cost_KAC = average_cost;
        K = K0; % Set next K
    elseif ii == 2
        disp(['Cost by K0     :',num2str(average_cost)])
        mean_cost_K0 = average_cost;
        K=Kstar; % Setv next K
    elseif ii == 3
        disp(['Cost by Kstar  :',num2str(average_cost)])
    end
end


%% Save the result
if save_num
    clear rand_ini_ang_matrix_1 rand_ini_vel_matrix_1
    save('step1_compare_K.mat')
end
