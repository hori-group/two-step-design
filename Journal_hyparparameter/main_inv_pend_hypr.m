%% Explanation
% Simulate 100 sets
% Set sigma_list (line 16) and beta (line 32)


%% Clear workspace and figures
clear
close all


%% Measure time
tic


%% Code execution settings
sigma_list = [5,1,0.5,0.1,0.05,0.01]; % Store used sigma
n_set = 100;

% Change here for setting initial beta
beta=0.0001; % [0.000005,0.00001,0.00005,0.0001,0.0005,0.001,0.005]

% Change here for changing method
type = 'ACRL';
% type = 'RLalone';
% type = 'K0RL';

% Load seed
myVars = {'rand_ini_ang_matrix_1','rand_ini_vel_matrix_1'};
load('../rand_list_1.mat',myVars{:})

% Load KAC and K0
myVars2 = {'KAC','K0'};
load('../Journal_step1/step1_design_KAC.mat',myVars2{:})

% Load plant and cost parameters
load('../parameter_setting')

% Set method
if strcmp(type,'ACRL')
    K = KAC;
    LQR=1; % use input from LQR or not (1: use, 0: not use)
elseif strcmp(type,'RLalone')
    LQR=0; % use input from LQR or not (1: use, 0: not use)
elseif strcmp(type,'K0RL')
    LQR=1; % use input from LQR or not (1: use, 0: not use)
    K = K0;
end
RL=1;  % use input from RL controller or not (1: use, 0: not use)


%% Hyparmarameters
beta_ini=beta;


%% Design

for sigma2_u=sigma_list
    disp(['sigma2_u : ',num2str(sigma2_u)])
    mean_cost_runs_list = nan(1,n_set); % Store each cost
    for NN = 1:n_set
        % Storage
        u_RL_res=nan(endTime/Ts,nu);
        theta_res=nan(endTime/Ts,n_s^n_s_s);
        W_res=nan(endTime/Ts,n_s^n_s_s);
        rew_rew=zeros(endTime/Ts,n_s^n_s_s);
        rew_res_loop=nan(1,n_trial);
        res_loop=nan(n_trial,1);


        %%  Control and training
        for epi=1:n_trial
            % Seed
            seed_number = (NN-1)*n_trial + epi;
            rng(seed_number);

            ff=0; % flag of fault : ff=1 if control is failed

            % Storage
            x_res=nan(1+endTime/Ts,nx);
            u_res=nan(endTime/Ts,nu);
            mu_u_res=nan(endTime/Ts,nu);

            % Initial State
            ini_ang = rand_ini_ang_matrix_1(epi,NN);
            ini_vel = rand_ini_vel_matrix_1(epi,NN);

            x=[ini_ang;ini_vel];
            x0 = [ini_ang;ini_vel];
            rew_res = -x0' * Q * x0;

            if epi==1
                V_s=0;
                V_s_kai=0;

                theta_i=1*ones(n_s^n_s_s,1);
                W_i=0*ones(n_s^n_s_s,nu);
            end

            phi_i=zeros(n_s^n_s_s,1);
            phi_i_kai=zeros(n_s^n_s_s,1);

            for k=1:endTime/Ts+1

                if k>1
                    u_00=u_0;
                end

                %% input from linear controller
                if LQR==1
                    u_0 = K*x;
                else
                    u_0=0;
                end

                %% Reinforcement learning
                if RL==1
                    xx_0=x;

                    xx=zeros(n_s_s,1);
                    for j=1:n_s_s
                        xx(j,1)=(xx_0(j)-c_i_min(j))*(n_s-1)/(c_i_max(j)-c_i_min(j))+1; % normalize each state of the controlled object
                    end

                    if k>1
                        if abs(x(1))> .5
                            reward=penalty;
                            ff=1;
                        else
                            u_00RL=u_00+u_RL;
                            reward=-(x.'*Q*x+(u_00RL).'*R*(u_00RL));
                        end

                        rew_res=rew_res+reward;
                    end

                    if k==1
                        z_th=zeros(length(phi_i),1);
                        z_W=zeros(length(phi_i),1);
                        I=1;
                    end

                    if k>1
                        for l=1:n_s^n_s_s
                            phi_i_kai(l)=exp(-(xx-c_i(l,:).').'*(xx-c_i(l,:).')/(2*sigma_i^2));
                        end
                        if ff==0
                            V_s_kai=phi_i_kai.'*theta_i;
                        else
                            V_s_kai=0;
                        end

                        td_err=reward+gamma*V_s_kai-V_s; % update td error

                        z_th=gamma*lambda_th*z_th + phi_i; % update z^{\theta}
                        if et==1
                            theta_i=theta_i+alpha*td_err*z_th; % update \theta
                        else
                            theta_i=theta_i+alpha*td_err*phi_i;
                        end

                        for j=1:nu
                            z_W=gamma*lambda_W*z_W + I*((sigma2_u*epsilon)^-1)*(u_RL(j,1)-phi_i.'*W_i(:,j))*phi_i; % update z^W
                            if et==1
                                W_i(:,j)=W_i(:,j)+beta*td_err*z_W; % update W
                            else
                                W_i(:,j)=W_i(:,j)+beta*td_err*((sigma2_u*epsilon)^-1)*(u_RL(j,1)-phi_i.'*W_i(:,j))*phi_i;
                            end
                        end
                        phi_i=phi_i_kai;
                        I=gamma*I;
                    end


                    if k==1
                        for l=1:n_s^n_s_s
                            phi_i(l)=exp(-(xx-c_i(l,:).').'*(xx-c_i(l,:).')/(2*sigma_i^2));
                        end
                        V_s=phi_i.'*theta_i;
                    else
                        V_s=V_s_kai;
                    end


                    if k>1
                        if ff==1
                            rew_res=penalty;
                            for i=k:endTime/Ts+1
                                x_res(i,:)=x.';
                            end
                            break
                        end
                    elseif k==endTime/Ts+1
                        break
                    end


                    if k>1
                        rew_rew(k,1)=reward;
                    end

                    mu_u=(phi_i.'*W_i).';

                    epsilon=nthroot(1e-4,n_trial-1)^epi; % epsilon = 1e-4 at episode n_epi-1 (decrease exploration range as number of episode increases)
                    beta=beta_ini*nthroot(1e-2,n_trial-1)^epi; % beta = beta_ini * 1e-2 at episode n_epi-1

                    if epi<n_trial
                        u_RL = repmat(mu_u,1,1) + randn(nu,1)*chol(sigma2_u*epsilon);
                    else
                        u_RL = repmat(mu_u,1,1); % control without exploration at the final episode
                    end


                else
                    u_RL=0;
                    mu_u=0;
                end


                u=u_0+u_RL;
                u_no_saturation=u_0+u_RL;
                if u<-Saturation
                    u=-Saturation;
                elseif u>Saturation
                    u=Saturation;
                end

                % Next state
                x=f_u(x,u);

                % Storage
                mu_u_res(k,:)=mu_u;
                u_res(k,:)=u;
                u_RL_res(k,:)=u_RL;
                x_res(k+1,:)=x.';
                W_res(k,:)=W_i.';
                theta_res(k,:)=theta_i.';

            end
            rew_res_loop(:,epi)=rew_res;
            res_loop(epi,1)=ff;
        end

        disp([num2str(NN),'set'])


        %% Save
        save_items = {'filename','beta_ini','sigma2_u','ff','phi_i_kai','phi_i','sigma_i','c_i','W_i','theta_i','z_W','z_th'};
        filename=append('data_',type,'_iniR_beta',ConvertForSave(beta_ini),'_sigma',ConvertForSave(sigma2_u),'_',num2str(NN),'.mat');
        save(filename,save_items{:})

        if NN ~=n_set
            %clearvars -except  sigma2_u sets mean_cost_runs_list rand_ini_ang_matrix_1 rand_ini_vel_matrix_1 NN
            clear rew_res_loop res_loop phi phi_i W_i theta_i z_w z_th ff rew_res
        end

    end
end


%% Measure time
toc


