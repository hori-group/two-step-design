%% Explanation
% Simulate and obtain data for trajectory


%% Clear workspace and figures
clear
close all


%% Measure time
tic


%% Code execution settings

% Select a method in three (comment out one type that you want to simulate)
type='ACRL'; % KAC+RL
% type='ACalone';

% load KAC
myVars2 = {'KAC'};
load('../Journal_step1/step1_design_KAC.mat',myVars2{:})
K = KAC;

% load seed file
myVars = {'rand_ini_ang_matrix_1','rand_ini_vel_matrix_1'};
load('../rand_list_1.mat',myVars{:})

% Load plant and cost parameters
load('../parameter_setting')


%% Save name
version='R10_trajectory_ver1';

if strcmp(type,'ACRL')
    beta = beta_ini_ACRL;
    beta_ini = beta;
    sigma2_u = sigma2_u_ACRL;
    str_Beta=ConvertForSave(beta); str_sigma=ConvertForSave(sigma2_u);
    basic_info_for_step1 = append(pendulum,'_',option,str_St,'_Ts',str_Ts,'_p',str_penalty);
    basic_info = append(basic_info_for_step1,'_beta',str_Beta,'_sigma',str_sigma,'_epi',str_tri,'_',version);
    filename = append('data_AC+RL_',basic_info);
    RL = 1; % use input from RL controller or not (1: use, 0: not use)
elseif strcmp(type,'ACalone')
    basic_info_for_step1 = append(pendulum,'_',option,str_St,'_Ts',str_Ts,'_p',str_penalty);
    basic_info = append(basic_info_for_step1,'_epi',str_tri,'_',version);
    filename = append('data_ACalone_',basic_info);
    RL = 0;
end


%% Setting for RL
LQR = 1; % use input from LQR or not (1: use, 0: not use)


%% List for storage
rew_res_loop = nan(1,n_trial); % Storage for rew_res
res_loop = nan(n_trial,1); % Storage for ff


%%%%%%%%%%%%%%%%%%%%%%%% Design controller %%%%%%%%%%%%%%%%%%%%%%%
%%  Design nonlinear controller
if ~strcmp(type,'ACalone')
    for epi = 1:n_trial
        % Seed
        seed_number = epi;
        rng(seed_number);

        ff=0; % flag of fault : ff=1 if control is failed

        % Initial state
        ini_ang = rand_ini_ang_matrix_1(epi,1);
        ini_vel = rand_ini_vel_matrix_1(epi,1);

        x = [ini_ang ; ini_vel];
        x0 = [ini_ang ; ini_vel];
        x_res(1,:) = x;
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


            %% input from linear controller
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


                if k > 1 % Failure confirmation
                    if abs(x(1)) > .5
                        reward = penalty;
                        ff = 1;
                    else
                        u_00RL = u_00 + u_RL;
                        reward = -(x.'*Q*x+(u_00RL).'*R*(u_00RL));
                    end

                    rew_res = rew_res+reward;
                end

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
                        for i = k:endTime/Ts+1
                            x_res(i,:) = x.';
                        end
                        break
                    end
                elseif k == endTime/Ts+1
                    break
                end


                %%
                mu_u = (phi_i.'*W_i).';

                epsilon = nthroot(1e-4,n_trial-1)^epi; % epsilon = 1e-4 at episode n_epi-1 (decrease exploration range as number of episode increases)
                beta = beta_ini*nthroot(1e-2,n_trial-1)^epi; % beta = beta_ini * 1e-2 at episode n_epi-1

                if epi < n_trial
                    u_RL = repmat(mu_u,1,1) + randn(nu,1)*chol(sigma2_u*epsilon);
                else
                    u_RL = repmat(mu_u,1,1); % control without exploration at the final episode
                end


            else
                u_RL = 0;
                mu_u = 0;
            end


            %
            u = u_AC + u_RL;
            u_no_st = u_AC + u_RL;

            if u < -Saturation
                u = -Saturation;
            elseif u > Saturation
                u = Saturation;
            end

            % Next state
            x = f_u(x,u);
            x_res(k+1,:) = x.';

        end %%%%%%%%%%%%%%%%%for 1:endtime/dt+1

        if rem(epi,1000) == 1
            disp(['episode:',num2str(epi),'  ff:',num2str(ff)])
        end

        rew_res_loop(:,epi) = rew_res;
        res_loop(epi,1) = ff;
    end
end



%%%%%%%%%%%%%%%%%%%%%%%% Calculate trajectory %%%%%%%%%%%%%%%%%%%%%%%
%% Control
n_trial = 1;
ff = 0; % flag of fault : ff=1 if control is failed

% store state and inputs
x_res = nan(1+endTime/Ts,nx); 
u_res = nan(endTime/Ts,nu);
uAC_res = nan(endTime/Ts,nu);
uRL_res = nan(endTime/Ts,nu);
u_no_st_res = nan(endTime/Ts,nu);

% Initial state
ini_ang = 0.4;
ini_vel = 0;

x = [ini_ang ; ini_vel];
x_res(1,:) = x;
x0 = [ini_ang ; ini_vel];
rew_res_trj = -x0' * Q * x0;


c_i = zeros(n_s^n_s_s,n_s_s);
n_s_0 = zeros(n_s,1);
for j = 1:n_s
    n_s_0(j,1) = j;
end
for kk = 1:n_s_s
    c_i(:,kk) = kron(ones(n_s^(kk-1),1),kron(n_s_0,ones(n_s^(n_s_s-kk),1)));
end


phi_i = nan(n_s^n_s_s,1);
phi_i_kai = nan(n_s^n_s_s,1);


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
    if k > 1 % Failure confirmation
        if abs(x(1)) > .5
            reward = penalty;
            ff = 1;
        else
            u_00RL = u_00 + u_RL;
            reward = -(x.'*Q*x+(u_00RL).'*R*(u_00RL));
        end

        rew_res_trj = rew_res_trj+reward;
    end
    if RL == 1
        xx_0 = x;
        xx = nan(n_s_s,1);
        for j = 1:n_s_s
            xx(j,1) = (xx_0(j)-c_i_min(j))*(n_s-1)/(c_i_max(j)-c_i_min(j))+1; % normalize each state of the controlled object
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
            if ff == 1
                rew_res_trj = penalty;
                for i = k:endTime/Ts+1
                    x_res(i,:) = x.';
                end
                break
            end
        elseif k == endTime/Ts+1
            break
        end


        % input from nonlinear controlelr
        mu_u = (phi_i.'*W_i).';
        u_RL = repmat(mu_u,1,1); % control without exploration at the final episode

    else
        u_RL = 0;
        mu_u = 0;
    end

    u = u_AC + u_RL;
    u_no_st = u_AC + u_RL;

    if u < -Saturation
        u = -Saturation;
    elseif u > Saturation
        u = Saturation;
    end

    % next state
    x = f_u(x,u);

    % store state and inputs
    x_res(k+1,:) = x.';
    u_res(k,:) = u;
    uAC_res(k,:) = u_AC;
    uRL_res(k,:) = u_RL;
    u_no_st_res(k,:) = u_no_st;

end %%%%%%%%%%%%%%%%%for 1:endtime/dt+1



%% Save and display the results

clear rand_list_1 rand_ini_ang_matrix_1 rand_ini_vel_matrix_1
save([filename,'_',num2str(1),'.mat'])

disp(['cost :',num2str(-rew_res_trj)])


%% Measure time
toc



