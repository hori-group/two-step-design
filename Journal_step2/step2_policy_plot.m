%% Explanation
% plot policy in 3D graph by ACRL (fig5,10)


%% Clear workspace and figures
clear
close all


%% Code execution settings
% Result save flag (0 or 1)
saving_num = 1; % Set 1 if you save silumation result

% Load plant and cost parameters
myVars = {'beta_ini_ACRL','sigma2_u_ACRL'};
load('../parameter_setting',myVars{:})


%% Figure settings
set(0,'defaultLineLineWidth',1)
set(0,'defaultAxesFontSize',18)
set(0,'defaultTextFontSize',24)
set(0,'defaultAxesFontName','Times New Roman')
set(0,'defaultTextFontName','TImes New Roman')
set(0,'defaultFigurePosition',[30 30 900 750])
set(0,'defaultFigureColor','white')


%% File name
% save file
beta = beta_ini_ACRL;
sigma = sigma2_u_ACRL;

% Load file
load_file = 'data_AC+RL_pnd6_St0p5_Ts0p06_p1000_beta0p0001_sigma0p1_epi4000_R10_trajectory_ver1_1.mat';
load(load_file)

% Load plant and cost parameters
load('../parameter_setting')

ver='epi4000';
Beta_str=ConvertForSave(beta); sigma_str=ConvertForSave(sigma);
basic_info2=append(pendulum,'_St',str_St,'_Ts',str_Ts,'_p',str_penalty,'_beta',Beta_str,'_sigma',sigma_str,'_',ver);


%% uRL
phi_i=zeros(n_s^n_s_s,1);
mu_u_dd=zeros(25,51);

for d1=1:51
    for d2 = 14:38
        xx=[1+(d1-1)/5;1+(d2-1)/5];

        for l=1:n_s^n_s_s
            phi_i(l)=exp(-(xx-c_i(l,:).').'*(xx-c_i(l,:).')/(2*sigma_i^2));
        end
        mu_u_dd(d2-13,d1)=(phi_i.'*W_i).';
    end
end

[X,Y] = meshgrid(-.5:.02:.5,-1.92:.16:1.92);

figure(11)
surfc(X,Y,mu_u_dd)
grid on
xlabel('Angle $$\psi_k$$ $$[\mathrm{rad}]$$','interpreter','latex');
ylabel('Angular velocity $$\xi_k$$ $$[\mathrm{rad/s}]$$','interpreter','latex');
zlabel('$$u^{\mathrm{RL}}_k$$ $$[\mathrm{N} \cdot \mathrm{m}]$$','interpreter','latex');
colorbar
lim = caxis;
view([1,-1.5,1])
hold off


%% uAC

u_opt = nan(25,51);

for d1=1:51

    for d2=14:38
        x=[-.5+.02*(d1-1);-2+.08*(d2-1)];
        u_opt(d2-13,d1)=K*x;
    end
end

figure(22)
surfc(X,Y,u_opt)
grid on
xlabel('Angle $$\psi_k$$ $$[\mathrm{rad}]$$','interpreter','latex');
ylabel('Angular velocity $$\xi_k$$ $$[\mathrm{rad/s}]$$','interpreter','latex');
zlabel('$$u^{\mathrm{AC}}_k$$ $$[\mathrm{N} \cdot \mathrm{m}]$$','interpreter','latex');
colorbar
view([1,-2,1])
hold off

%% uAC + uRL
u_ACRL=mu_u_dd+u_opt;

figure(33)
surfc(X,Y,u_ACRL)
grid on
xlabel('Angle $$\psi_k$$ $$[\mathrm{rad}]$$','interpreter','latex');
ylabel('Angular velocity $$\xi_k$$ $$[\mathrm{rad/s}]$$','interpreter','latex');
zlabel('$$u^{\mathrm{AC}}_k + u^{\mathrm{RL}}_k$$ $$[\mathrm{N} \cdot \mathrm{m}]$$','interpreter','latex');
colorbar
view([1,-2,1])
hold off


%% save
if saving_num
    save_form = 'png'; % png or epsc etc...
    filename=append(basic_info2,'_uRL');
    saveas(11,filename,save_form);
    filename=append(basic_info2,'_uAC');
    saveas(22,filename,save_form);
    filename=append(basic_info2,'_uRLuAC');
    saveas(33,filename,save_form);
end


