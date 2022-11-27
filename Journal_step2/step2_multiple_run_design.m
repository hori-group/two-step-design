%% Explanation
% Simulation step2 for 3500 runs


%% Clear workspace and figures and measure time
clear
close all


%%  Measure time
tic


%% Code execution settings

% Select a method in three
type='AC'; % KAC+RL
% type='RLalone';
% type='K0'; % K0+RL

% Number of simulation set
n_set = 3500; % Number of sets (design the controller n_set sets)

% Load seed for initial state
myVars = {'rand_ini_ang_matrix_1','rand_ini_vel_matrix_1'};
load('../rand_list_1.mat', myVars{:})

% Load plant and cost parameters
load('../parameter_setting')

% Save name
version='R10_ver1_test';

% Setting for a method
if strcmp(type,'AC')
    beta = beta_ini_ACRL;
    sigma2_u = sigma2_u_ACRL;
    % Setting for save
    str_Beta=ConvertForSave(beta); str_sigma=ConvertForSave(sigma2_u); 
    basic_info = append('pnd6_St',str_St,'_Ts',str_Ts,'_p',str_penalty,'_beta',str_Beta,'_sigma',str_sigma,'_epi',str_tri,'_',version);
    filename = append('data_AC+RL_',basic_info);
    % load KAC
    K_filename = '../Journal_step1/step1_design_KAC.mat';
    myVars2 = {'KAC'};
    load(K_filename,myVars2{:})
    K = KAC;
    LQR = 1; % Use input from LQR or not (1: use, 0: not use)
    RL = 1; % Use input from RL controller or not (1: use, 0: not use)
elseif strcmp(type,'RLalone')
    beta = beta_ini_RLalone;
    sigma2_u = sigma2_u_RLalone;
    % Setting for save
    str_Beta=ConvertForSave(beta); str_sigma=ConvertForSave(sigma2_u);
    basic_info = append('pnd6_St',str_St,'_Ts',str_Ts,'_p',str_penalty,'_beta',str_Beta,'_sigma',str_sigma,'_epi',str_tri,'_',version);
    filename = append('data_RLalone_',basic_info);
    LQR = 0; % use input from LQR or not (1: use, 0: not use)
    RL = 1; % Use input from RL controller or not (1: use, 0: not use)
elseif strcmp(type,'K0')
    beta = beta_ini_K0RL;
    sigma2_u = sigma2_u_K0RL;
    % Setting for save
    str_Beta=ConvertForSave(beta); str_sigma=ConvertForSave(sigma2_u);
    basic_info = append('pnd6_St',str_St,'_Ts',str_Ts,'_p',str_penalty,'_beta',str_Beta,'_sigma',str_sigma,'_epi',str_tri,'_',version);
    filename = append('data_K0RL_',basic_info);
    % load K0
    K_filename = '../Journal_step1/step1_design_KAC.mat';
    myVars2 = {'K0'};
    load(K_filename,myVars2{:})
    K = K0;
    LQR = 1; % Use input from LQR or not (1: use, 0: not use)
    RL = 1; % Use input from RL controller or not (1: use, 0: not use)
end


%% parameter settings for RL
beta_ini = beta; % learning rate


%% Design controller

for set_cnt = 1:n_set

    % List for save
    rew_res_loop = nan(1,n_trial); % For cumulative reward
    res_loop = nan(n_trial,1); % For fault

    %% Control and training
    for epi = 1:n_trial
        seed_number = (set_cnt-1)*n_trial +epi;
        rng(seed_number);
        ff=0; % Flag of fault : ff=1 if control is failed

        % Initial stete
        ini_ang = rand_ini_ang_matrix_1(epi,set_cnt);
        ini_vel = rand_ini_vel_matrix_1(epi,set_cnt);

        x = [ini_ang ; ini_vel];
        x0 = [ini_ang ; ini_vel];
        rew_res = -x0' * Q * x0;

        if epi == 1
            V_s = 0;
            V_s_kai = 0;

            theta_i = 1*ones(n_s^n_s_s,1);
            W_i = 0*ones(n_s^n_s_s,nu);
        end

        phi_i = nan(n_s^n_s_s,1);
        phi_i_kai = nan(n_s^n_s_s,1);

        for k = 1:endTime/Ts+1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            if k > 1
                u_00 = u_AC;
            end

            % input from linear controller
            if LQR == 1
                u_AC = K*x;
            else
                u_AC = 0;
            end


            %% Reinforcement learning

            if RL == 1
                xx_0 = x;

                xx = nan(n_s_s,1);
                for j = 1:n_s_s
                    xx(j,1) = (xx_0(j)-c_i_min(j))*(n_s-1)/(c_i_max(j)-c_i_min(j))+1; % normalize each state of the controlled object
                end

                % Failure confirmation
                if k > 1
                    if abs(x(1)) > .5
                        reward = penalty;
                        ff = 1;
                    else
                        u_00RL = u_00 + u_RL;
                        reward = -(x.'*Q*x+(u_00RL).'*R*(u_00RL));
                    end

                    rew_res = rew_res+reward;
                end

                % Update parameters
                if k == 1
                    z_th = zeros(length(phi_i),1);
                    z_W = zeros(length(phi_i),1);
                    I = 1;
                end


                if k > 1
                    for l = 1:n_s^n_s_s
                        phi_i_kai(l) = exp(-(xx-c_i(l,:).').'*(xx-c_i(l,:).')/(2*sigma_i^2));
                    end
                    if ff == 0
                        V_s_kai = phi_i_kai.'*theta_i;
                    else
                        V_s_kai = 0; % V(s')=0 at the time of fall
                    end

                    td_err = reward + gamma*V_s_kai-V_s; % update td error
                    z_th = gamma*lambda_th*z_th + phi_i; % update z^{\theta}

                    if et == 1
                        theta_i = theta_i+alpha*td_err*z_th; % update \theta
                    else
                        theta_i = theta_i+alpha*td_err*phi_i;
                    end

                    for j = 1:nu
                        z_W = gamma*lambda_W*z_W + I*((sigma2_u*epsilon)^-1)*(u_RL(j,1)-phi_i.'*W_i(:,j))*phi_i; % update z^{\W}
                        if et == 1
                            W_i(:,j) = W_i(:,j)+beta*td_err*z_W; % update W
                        else
                            W_i(:,j) = W_i(:,j)+beta*td_err*((sigma2_u*epsilon)^-1)*(u_RL(j,1)-phi_i.'*W_i(:,j))*phi_i;
                        end
                    end
                    phi_i = phi_i_kai;
                    I = gamma*I;
                end


                if k == 1
                    for l = 1:n_s^n_s_s
                        phi_i(l) = exp(-(xx-c_i(l,:).').'*(xx-c_i(l,:).')/(2*sigma_i^2));
                    end
                    V_s = phi_i.'*theta_i;
                else
                    V_s = V_s_kai;
                end


                if k > 1
                    if ff == 1
                        rew_res = penalty;
                        break
                    end
                elseif k == endTime/Ts+1
                    break
                end


                % generate u_RL
                mu_u = (phi_i.'*W_i).';

                epsilon = nthroot(1e-4,n_trial-1)^epi; % epsilon = 1e-4 at episode n_epi-1 (decrease exploration range as number of episode increases)
                beta = beta_ini*nthroot(1e-2,n_trial-1)^epi; % beta = beta_ini * 1e-2 at episode n_epi-1

                if epi < n_trial
                    u_RL = repmat(mu_u,1,1) +randn(nu,1)*chol(sigma2_u*epsilon);
                else
                    u_RL = repmat(mu_u,1,1); % control without exploration at the final episode
                end

            else
                u_RL = 0;
                mu_u = 0;
            end

            % Generate input
            u = u_AC + u_RL;   

            if u < -Saturation
                u = -Saturation;
            elseif u > Saturation
                u = Saturation;
            end

            % Next state
            x = f_u(x,u);

        end %%%%%%%%%%%%%%%%%for 1:endtime/dt+1

        % If the controller is not designed
        if isnan(rew_res)
            rew_res = penalty;
            ff = 1;
        end

        % Notice of Calculation Status
        if rem(epi,1000) == 1
            disp(['Trial:',num2str(epi),'  ff:',num2str(ff)])
        end

        rew_res_loop(:,epi) = rew_res;
        res_loop(epi,1) = ff;

    end

    % Save and display the results
    disp([num2str(set_cnt),'set'])

    filename1 = [filename,'_',num2str(set_cnt),'.mat'];
    save(filename1,'rew_res_loop','res_loop','phi_i','W_i','theta_i','z_W','z_th')
    if set_cnt ~= n_set
        clear rew_res_loop res_loop phi phi_i W_i theta_i z_w z_th ff rew_res
    end
end


%% Measure time
toc

