%% Explaination
% Plot the figure of trajectory (fig.6,7)


%% Clear workspace and figures
clear
close all


%% Control and calculate cost

% Result save flag (0 or 1)
TF_saving = 0; % Set 1 if you save silumation result

% Load plant and cost parameters
load('../parameter_setting')

%% Figure settings
set(0,'defaultLineLineWidth',1.5)
set(0,'defaultAxesFontSize',18)
set(0,'defaultTextFontSize',18)
set(0,'defaultAxesFontName','Times New Roman')
set(0,'defaultTextFontName','TImes New Roman')
set(0,'defaultFigurePosition',[0 0 700 450])
set(0,'defaultFigureColor','white')
lw=1.5;
stairs_line_width = 2;


%% Save file name
basic_info_save=append('pnd_St',str_St,'_Ts',str_Ts,'_p',str_penalty);
ACRL_file = 'data_AC+RL_pnd6_St0p5_Ts0p06_p1000_beta0p0001_sigma0p1_epi4000_R10_trajectory_ver1_1.mat';
ACalone_file = 'data_ACalone_pnd6_St0p5_Ts0p06_p1000_epi4000_R10_trajectory_ver1_1.mat';


%% Legend setting
legend_ACRL = '$$K^\mathrm{AC}$$ + RL (Proposed method)';
legend_ACalone = '$$K^{\mathrm{AC}}$$ alone';
legend_name_list = {legend_ACRL,legend_ACalone};
legend_name_list_angle = {legend_ACRL,legend_ACalone,'Angle=0'};
legend_name_list_saturation = {legend_ACRL,legend_ACalone,'Saturation'}; 


%% Trajectories of states and inputs 
%% Angle
load(ACRL_file)
tsim = 0:length(x_res(:,1))-1;
tsim2 = 0:length(u_no_st_res(:,1))-1;
figure(100)
ACRL_angle_plot = plot(tsim,x_res(:,1),'r');
hold on

load(ACalone_file)
ACalone_angle_plot = plot(tsim,x_res(:,1),'Color', [0.47 0.670 0.19]);
hold on

Angle_angle_plot = plot(tsim,zeros(length(x_res(:,1)),1),'k--','linewidth',2);
hold on
angle_plot_list = [ACRL_angle_plot, ACalone_angle_plot, Angle_angle_plot];

legend(angle_plot_list, legend_name_list_angle, 'Location', 'best', 'interpreter', 'latex')
grid on
xlim([0 30]);
xlabel('Time step')
ylabel('Angle $$\psi_k$$ $$[\mathrm{rad}]$$','interpreter','latex'); % 縦軸のラベル
xlim([0 30]);
hold off


%% Angular velocity
figure(101)
load(ACRL_file)
ACRL_velocity_plot = plot(tsim,x_res(:,2),'r');
hold on

load(ACalone_file)
ACalone_velocity_plot = plot(tsim,x_res(:,2),'Color', [0.47 0.670 0.19]);
hold on

legend_velocity_list = [ACRL_velocity_plot, ACalone_velocity_plot];
grid on
xlim([0 30]);
xlabel('Time step')
ylabel('Angular velocity $$\xi_k$$ $$[\mathrm{rad}]$$','interpreter','latex'); 
legend(legend_velocity_list,legend_name_list,'Location','best','interpreter','latex')
hold off


%% Control input
figure(102)
load(ACRL_file)
ACRL_input_plot = stairs(tsim2, u_no_st_res,'r','linewidth',lw);
ACRL_input_plot(1).LineWidth = stairs_line_width;
hold on

load(ACalone_file)
ACalone_input_plot = stairs(tsim2, u_no_st_res, 'Color', [0.47 0.670 0.19]);
ACalone_input_plot(1).LineWidth = stairs_line_width;
hold on

saturation_input_plot = plot(tsim2,0.5*ones(length(u_no_st_res),1),'k--','linewidth',2);
hold on
plot(tsim2,-0.5*ones(length(u_no_st_res),1),'k--','linewidth',2)

input_plot_list = [ACRL_input_plot, ACalone_input_plot, saturation_input_plot];
legend(input_plot_list, legend_name_list_saturation, 'Location', 'best', 'interpreter', 'latex')
grid on
xlim([0 30]);
xlabel('Time step')
ylabel('Control Input $$u_k$$ $$[\mathrm{N \cdot m}]$$','interpreter','latex')
hold off


%% Actual torque input (saturated input)
figure(103)
load(ACRL_file)
ACRL_saturated_input_plot = stairs(tsim2, u_res,'r','linewidth',lw);
ACRL_saturated_input_plot(1).LineWidth = stairs_line_width;
hold on
load(ACalone_file)
ACalone_saturated_input_plot = stairs(tsim2, u_res,'Color', [0.47 0.670 0.19]);
ACalone_saturated_input_plot(1).LineWidth = stairs_line_width;
hold on

saturation_saturated_input_plot = plot(tsim2,0.5*ones(length(u_no_st_res),1),'k--','linewidth',2);
hold on
plot(tsim2,-0.5*ones(length(u_no_st_res),1),'k--','linewidth',2)

saturated_input_plot_list = [ACRL_saturated_input_plot, ACalone_saturated_input_plot, saturation_saturated_input_plot];
legend(saturated_input_plot_list,legend_name_list_saturation,'Location','best','interpreter','latex')
grid on
xlim([0 30]);
xlabel('Time step')
ylabel('Actual torque input $$\bar{u}_k$$ $$[\mathrm{N \cdot m}]$$','interpreter','latex')
axis([0 30 -1.5 .5])
hold off


%% Save the results 
if TF_saving==1
    save_form = 'png'; % png or epsc etc...
    filename=append(basic_info_save,'_Trajectory_Angle');
    saveas(100,filename,save_form);
    filename=append(basic_info_save,'_Trajectory_Velocity');
    saveas(101,filename,save_form);
    filename=append(basic_info_save,'_Trajectory_ControlInput');
    saveas(102,filename,save_form);
    filename=append(basic_info_save,'_Trajectory_Input');
    saveas(103,filename,save_form);
end


