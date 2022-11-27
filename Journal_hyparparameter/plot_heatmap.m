%% Explanation
% Plot heatmap (fig.8,9)


%% Clear workspace and figures
clear
close all


%% Code execution settings
% Result save flag (0 or 1)
TF_heatmap_save = 1; % Set 1 if you save silumation result

% Number of approachs
number_of_type = 3;

% Load files
ACRL_file='aggregation_ACRL.mat';
RLalone_file='aggregation_RLalone.mat';
K0RL_file='aggregation_K0RL.mat';


%% Graph Scale Settings
beta4='5.0\times 10^{-6}';
beta5='1.0\times 10^{-5}';
beta6='5.0\times 10^{-5}';
beta7='1.0\times 10^{-4}';
beta8='5.0\times 10^{-4}';
beta9='1.0\times 10^{-3}';
beta10='5.0\times 10^{-3}';

sigma1='1.0\times 10^{-2}';
sigma22='5.0\times 10^{-2}';
sigma3='1.0\times 10^{-1}';
sigma4='5.0\times 10^{-1}';
sigma5='1';
sigma6='5';

beta_label_list = {beta4,beta5,beta6,beta7,beta8,beta9,beta10};
sigma_label_list = {sigma6,sigma5,sigma4,sigma3,sigma22,sigma1};

shape=[6,7]; % Size of table


%% Figure settings
set(0,'defaultLineLineWidth',1.5)
set(0,'defaultAxesFontSize',18)
set(0,'defaultTextFontSize',18)
set(0,'defaultAxesFontName','Times New Roman')
set(0,'defaultTextFontName','TImes New Roman')
set(0,'defaultFigurePosition',[100 100 400 550]) 
set(0,'defaultFigureColor','white')
FS = 14;


%% Load data and summarize
shape_size=shape(1)*shape(2);
PoS_RL=zeros(1,shape_size);   PoI_RL=zeros(1,shape_size); mean_cost_RLalone = zeros(1,shape_size);
PoS_ACRL=zeros(1,shape_size); PoI_ACRL=zeros(1,shape_size); mean_cost_ACRL = zeros(1,shape_size);
PoS_K0RL=zeros(1,shape_size);   PoI_K0RL=zeros(1,shape_size); mean_cost_K0RL = zeros(1,shape_size);

load(RLalone_file)
for ii=1:shape_size
    PoS_RL(1,ii) = filename_list(ii).N_PoS;
    PoI_RL(1,ii) = filename_list(ii).N_PoI;
end
PoS_RL = reshape(PoS_RL,shape)
PoI_RL = reshape(PoI_RL,shape)

load(ACRL_file)
for ii=1:shape_size
    PoS_ACRL(1,ii) = filename_list(ii).N_PoS;
    PoI_ACRL(1,ii) = filename_list(ii).N_PoI;
end
PoS_ACRL = reshape(PoS_ACRL,shape)
PoI_ACRL = reshape(PoI_ACRL,shape)

load(K0RL_file)
for ii=1:shape_size
    PoS_K0RL(1,ii) = filename_list(ii).N_PoS;
    PoI_K0RL(1,ii) = filename_list(ii).N_PoI;
end
PoS_K0RL = reshape(PoS_K0RL,shape)
PoI_K0RL = reshape(PoI_K0RL,shape)


%% Percentage of successful learning
figure(1)
subplot(number_of_type,1,1);
h = heatmap(PoS_ACRL);
h.ColorLimits = [0,100];
h.Colormap = parula(64);
h.FontSize = 9;
h.FontName = 'Times New Roman';
h.Title = {'$$K^{\mathrm{AC}}$$ + RL (Proposed method)'};
h.NodeChildren(3).Title.Interpreter = 'latex';
h.XLabel = 'Initial learning rate $$\beta_{\mathrm{init}}$$'; 
h.YLabel = {'Initial variance $$\sigma_{\mathrm{init}}^2$$'}; 
h.NodeChildren(3).XAxis.Label.Interpreter = 'latex';
h.NodeChildren(3).YAxis.Label.Interpreter = 'latex';
h.XData = beta_label_list;
h.YData = sigma_label_list;


subplot(number_of_type,1,2);
h = heatmap(PoS_RL);
h.ColorLimits = [0,100];
h.Colormap = parula(64);
h.Title = {'RL alone'};
h.FontSize = 9; 
h.FontName = 'Times New Roman';
h.XLabel = 'Initial learning rate $$\beta_{\mathrm{init}}$$'; 
h.YLabel = {'Initial variance $$\sigma_{\mathrm{init}}^2$$'}; 
h.NodeChildren(3).XAxis.Label.Interpreter = 'latex';
h.NodeChildren(3).YAxis.Label.Interpreter = 'latex';
h.XData = beta_label_list;
h.YData = sigma_label_list;

subplot(number_of_type,1,number_of_type);
h = heatmap(PoS_K0RL);
h.Title = {'$$K^{\mathrm{init}}$$ + RL'};
h.NodeChildren(3).Title.Interpreter = 'latex';
h.ColorLimits = [0,100];
h.Colormap = parula(64);
h.FontSize = 9;
h.FontName = 'Times New Roman';
h.XLabel = 'Initial learning rate $$\beta_{\mathrm{init}}$$'; 
h.YLabel = {'Initial variance $$\sigma_{\mathrm{init}}^2$$'}; 
h.NodeChildren(3).XAxis.Label.Interpreter = 'latex';
h.NodeChildren(3).YAxis.Label.Interpreter = 'latex';
h.XData = beta_label_list;
h.YData = sigma_label_list;


%% Percentage of improvemet in performance
figure(2)
subplot(number_of_type,1,1);
h = heatmap(PoI_ACRL);
h.ColorLimits = [0,100];
h.Colormap = parula(64); 
h.FontSize = 9;
h.FontName = 'Times New Roman';
h.Title = {'$$K^{\mathrm{AC}}$$ + RL (Proposed method)'};
h.NodeChildren(3).Title.Interpreter = 'latex';
h.XLabel = 'Initial learning rate $$\beta_{\mathrm{init}}$$'; 
h.YLabel = {'Initial variance $$\sigma_{\mathrm{init}}^2$$'}; 
h.NodeChildren(3).XAxis.Label.Interpreter = 'latex';
h.NodeChildren(3).YAxis.Label.Interpreter = 'latex';
h.XData = beta_label_list;
h.YData = sigma_label_list;

subplot(number_of_type,1,2);
h = heatmap(PoI_RL);
h.ColorLimits = [0,100];
h.Colormap = parula(64);
h.Title = {'RL alone'};
h.FontSize = 9;
h.FontName = 'Times New Roman';
h.XLabel = 'Initial learning rate $$\beta_{\mathrm{init}}$$'; 
h.YLabel = {'Initial variance $$\sigma_{\mathrm{init}}^2$$'}; 
h.NodeChildren(3).XAxis.Label.Interpreter = 'latex';
h.NodeChildren(3).YAxis.Label.Interpreter = 'latex';
h.XData = beta_label_list;
h.YData = sigma_label_list;

subplot(number_of_type,1,number_of_type);
h = heatmap(PoI_K0RL);
h.Title = {'$$K^{\mathrm{init}}$$ + RL'};
h.NodeChildren(3).Title.Interpreter = 'latex';
h.ColorLimits = [0,100];
h.Colormap = parula(64); 
h.FontSize = 9;
h.FontName = 'Times New Roman';
h.XLabel = 'Initial learning rate $$\beta_{\mathrm{init}}$$'; 
h.YLabel = {'Initial variance $$\sigma_{\mathrm{init}}^2$$'}; 
h.NodeChildren(3).XAxis.Label.Interpreter = 'latex';
h.NodeChildren(3).YAxis.Label.Interpreter = 'latex';
h.XData = beta_label_list;
h.YData = sigma_label_list;


%% save
if TF_heatmap_save
    save_form = 'png'; % png or epsc etc...    
    filename = 'heatmap_PoS';
    saveas(1,filename,save_form);
    filename = 'heatmap_PoI';
    saveas(2,filename,save_form);
end



