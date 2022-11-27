%% Explanation
% Plot fig.4


%% Clear workspace and figures
clear
close all


%% Code execution settings

% Result save flag (0 or 1)
TF_saving2 = 0; % Set 1 if you save silumation result

% Load average of the cost using ACalone
myVars = {'mean_cost_KAC'};
load('../Journal_step1/step1_compare_K.mat', myVars{:})
cost_ACalone = mean_cost_KAC;

% load file summarizing the results of step2 calculations
load_file  = 'ToFigData_R10_ver5.mat';


%% Figure settings
set(0,'defaultLineLineWidth',1)
set(0,'defaultAxesFontSize',15)
set(0,'defaultTextFontSize',15)
set(0,'defaultAxesFontName','Times New Roman')
set(0,'defaultTextFontName','TImes New Roman')
set(0,'defaultFigurePosition',[200 200 600 400])
set(0,'defaultFigureColor','white')


%% Load and plot

load(load_file)

figure(1)
plot_cost(3) = plot(1:n_trial, mean_cost_K0RL_list,'Color', 'm');
hold on
plot_cost(1) = plot(1:n_trial, mean_cost_ACRL_list,'Color', 'r');
hold on
plot_cost(2) = plot(1:n_trial, mean_cost_RLalone_list,'Color', 'b');
hold on
plot_ACalone = plot(1:n_trial, cost_ACalone*ones(n_trial,1),'--', 'Color', [0.47 0.670 0.19],'linewidth',3);
hold on
grid on
hold off
box on
axis([1 n_trial 30 60]);
xticks([1 500 1000 1500 2000 2500 3000 3500 4000])
xticklabels({'1','500','1000','1500','2000','2500','3000','3500','4000'})


f2 = figure(2);
plot_cost(3) = plot(1:n_trial, mean_cost_K0RL_list,'Color', 'm');
hold on
plot_cost(1) = plot(1:n_trial, mean_cost_ACRL_list,'Color', 'r');
hold on
plot_cost(2) = plot(1:n_trial, mean_cost_RLalone_list,'Color', 'b');
hold on
plot_ACalone = plot(1:n_trial, cost_ACalone*ones(n_trial,1),'--', 'Color', [0.47 0.670 0.19],'linewidth',3);
hold on
grid on
xlabel('Trial')
ylabel('Average of cost $$J_{\mathrm{fin}}(k_{\mathrm{fin}})$$ in each trial','interpreter','latex')
hold off
box on
legend([plot_cost,plot_ACalone],{'$$K^{\mathrm{AC}}$$ + RL (proposed method)','RL alone','$$K^{\mathrm{init}}$$ + RL','$$K^{\mathrm{AC}}$$ alone'},'Location','northoutside','interpreter','latex','NumColumns',2)
axis([0 n_trial 30 1000]);
xticks([1 500 1000 1500 2000 2500 3000 3500 4000])
xticklabels({'1','500','1000','1500','2000','2500','3000','3500','4000'})
f2.Position = [800 200 600 470];


%% Save the results 

if TF_saving2 == 1
    save_form = 'png'; % png or epsc etc...
    saveas(1,'step2_cost_fig_60',save_form)
    saveas(2,'step2_cost_fig_1000',save_form)
end




