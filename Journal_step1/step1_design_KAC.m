%% Explanation
% This is the code to design the quasi-optimal linear auxiliary controller K^AC (KAC)


%% Clear workspace and figures
clear
close all


%% Code execution settings

% Seed fixation
rng(1) 

% Result save flag (0 or 1)
save_num = 1; % Set 1 if you save silumation result

% Load plant and cost parameters
load('../parameter_setting')

%% Figure settings
set(0,'defaultLineLineWidth',1)
set(0,'defaultAxesFontSize',9)
set(0,'defaultTextFontSize',9)
set(0,'defaultAxesFontName','Times New Roman')
set(0,'defaultTextFontName','TImes New Roman')
set(0,'defaultFigurePosition',[10 278 260 170])
set(0,'defaultFigureColor','white')


%% Exploaration term parameters
num_of_prob = 30; % Time steps for exploration term (probing signal)
amp = 0.01; % Amplitude parameter of the exploration term (probing signal). Make this small enough to keep (x, u) near the origin.


%% Time settings
t0 = 0; % Start time
tl = t0 + num_of_prob*Ts; % End time


%% Intitial control law K0  
Q0 = diag([1000,1]);
R0 = 1;
[K0,P0,~] = lqr(sys,Q0,R0);
L1_0 = P0;
L2_0 = B'*P0*A;
L3_0 = B'*P0*B;
G1sol(:,:,1)=L1_0;
G2sol(:,:,1)=L2_0;
G3sol(:,:,1)=L3_0;


%% Exploration term (probing signal) used for data collection
ts = 0 : Ts : Ts*num_of_prob;
nw = 100;
w = -500 + (1000)*rand(nw,1);
mu =amp*sin(w(1)*ts)';
for ii = 2:nw
    mu = mu + amp*sin(w(ii)*ts)';
end


%% Algorithm 1: Data Collection
%%%%%% prep %%%%%%%%%%%%
x0 = [0; 0];
x = x0;
data_set_x = nan(2,num_of_prob+1);
data_set_x(:,1) = x0;
prob_collection = nan(num_of_prob+1,1);
U_collection = nan(num_of_prob,1);
Klast = [K0(1) K0(2)];
K = Klast;
%%%%%% simulation %%%%%%%%%%%%%
for ii = 1:num_of_prob
    u = mu(ii);
    U=-K*x+u;
    if U<-Saturation
        U=-Saturation;
    elseif U>Saturation
        U=Saturation;
    end
    x = f_u(x,U);
    data_set_x(:,ii+1) = x;
    U_collection(ii,1) = U;
    prob_collection(ii+1,1) = u;
end
Ksol = Klast;
u_no_saturation = (-Klast*data_set_x)'+mu;


%% Check the angle and angular velocity
% If the angle or angular velocity of the pendulum goes out of the specified range, the program is stopped by throwing an error
max_angle=max(data_set_x(1,:)); min_angle=min(data_set_x(1,:));
max_velocity=max(data_set_x(2,:)); min_velocity=min(data_set_x(2,:));
if max_angle>0.5 || min_angle<-0.5
    error('Angle is greater than 0.5 rad')
elseif max_velocity>4 || min_velocity<-4
    error('Angular velocity is greater than 4 rad/s')
end


%% Algorithm 1: Policy Evaluation and Improvement / Repetition and Termination
eps=1e-3;
F_row = 7; % 2^2+ 2 + 1^2

%%%%%%%%%%% k = 1 %%%%%%%%%%%
k = 1;
h_vector = nan(num_of_prob-1,1);
for ii = 1:num_of_prob-1
    h_i = (data_set_x(:,ii))'*Q*data_set_x(:,ii)+(data_set_x(:,ii))'*K'*R*K*data_set_x(:,ii);
    h_vector(ii,1) = h_i;
end

F_matrix = nan(num_of_prob-1,F_row);
for ii = 1:num_of_prob-1
    Fxx = kron((data_set_x(:,ii))',(data_set_x(:,ii))') - kron((data_set_x(:,ii+1))',(data_set_x(:,ii+1))');
    Fxu = 2*kron((data_set_x(:,ii))',(u_no_saturation(ii)+K*data_set_x(:,ii))');
    Fuu = -kron((K*data_set_x(:,ii)-u_no_saturation(ii))',(u_no_saturation(ii)+K*data_set_x(:,ii))');
    F_matrix(ii,:) = [Fxx Fxu Fuu];
end
g_vector = F_matrix \ h_vector;
G1sol(:,:,k+1) = [g_vector(1) g_vector(3);g_vector(2) g_vector(4)];
G2sol(:,:,k+1) = [g_vector(5) g_vector(6)];
G3sol(:,:,k+1) = g_vector(7);
Ksol(:,:,k+1) = (R+G3sol(:,:,k+1)) \ G2sol(:,:,k+1)
K = Ksol(:,:,k+1);
%%%%%%%%%%% k = 2 %%%%%%%%%%%
k = k+1;
h_vector = nan(num_of_prob-1,1);
for ii = 1:num_of_prob-1
    h_i = (data_set_x(:,ii))'*Q*data_set_x(:,ii) + (data_set_x(:,ii))'*K'*R*K*data_set_x(:,ii);
    h_vector(ii,1) = h_i;
end

F_matrix = nan(num_of_prob-1,F_row);
for ii = 1:num_of_prob-1
    Fxx = kron((data_set_x(:,ii))',(data_set_x(:,ii))')-kron((data_set_x(:,ii+1))',(data_set_x(:,ii+1))');
    Fxu = 2*kron((data_set_x(:,ii))',(u_no_saturation(ii)+K*data_set_x(:,ii))');
    Fuu = -kron((K*data_set_x(:,ii)-u_no_saturation(ii))',(u_no_saturation(ii)+K*data_set_x(:,ii))');
    F_matrix(ii,:) = [Fxx Fxu Fuu];
end
g_vector = F_matrix \ h_vector;

G1sol(:,:,k+1) = [g_vector(1) g_vector(3);g_vector(2) g_vector(4)];
G2sol(:,:,k+1) = [g_vector(5) g_vector(6)];
G3sol(:,:,k+1) = g_vector(7);
Ksol(:,:,k+1) = (R+G3sol(:,:,k+1)) \ G2sol(:,:,k+1)
K = Ksol(:,:,k+1);

if norm(Ksol(:,:,k)-Ksol(:,:,k-1))>=eps
    %%%%%%%%%%% k >= 3 %%%%%%%%%%%
    while norm(Ksol(:,:,k) - Ksol(:,:,k-1)) >= eps
        k = k+1;

        h_vector = nan(num_of_prob-1,1);
        for ii = 1:num_of_prob-1
            h_i = (data_set_x(:,ii))'*Q*data_set_x(:,ii)+(data_set_x(:,ii))'*K'*R*K*data_set_x(:,ii);
            h_vector(ii,1) = h_i;
        end

        F_matrix = nan(num_of_prob-1,F_row);
        for ii = 1:num_of_prob-1
            Fxx = kron((data_set_x(:,ii))',(data_set_x(:,ii))')-kron((data_set_x(:,ii+1))',(data_set_x(:,ii+1))');
            Fxu = 2*kron((data_set_x(:,ii))',(u_no_saturation(ii)+K*data_set_x(:,ii))');
            Fuu = -kron((K*data_set_x(:,ii)-u_no_saturation(ii))',(u_no_saturation(ii)+K*data_set_x(:,ii))');
            F_matrix(ii,:) = [Fxx Fxu Fuu];
        end
        g_vector = F_matrix\h_vector;
        G1sol(:,:,k+1) = [g_vector(1) g_vector(3);g_vector(2) g_vector(4)];
        G2sol(:,:,k+1) = [g_vector(5) g_vector(6)];
        G3sol(:,:,k+1) = g_vector(7);
        Ksol(:,:,k+1) = (R+G3sol(:,:,k+1))\G2sol(:,:,k+1)
        K = Ksol(:,:,k+1);
    end
end
Klast(:,:,2) = Ksol(:,:,k+1);
KAC = Klast(:,:,2);


%% Plot of the convergence of K^j
diff_K = [];

for gg = 1:size(Ksol,3) %size(Ksol,3) is the number of iterations
    diff_K(gg) = norm(Ksol(:,:,gg)-Kstar)/norm(Kstar);
end

iterK = 0:size(Ksol,3)-1;

figure(10)
plot(iterK,diff_K, 'r-','LineWidth', 2.5)
xlabel('Number of iterations $$j$$','interpreter','latex');
ylabel('$$\|{K}^j - {K}^\star\|/\|{K}^\star\|$$','interpreter','latex');
grid on


%% Cost during the design process (cost_design)
cost_design=0;
for jj=1:num_of_prob
    cc=data_set_x(:,jj)'*Q*data_set_x(:,jj) + (K0*data_set_x(:,jj))'*R*(K0*data_set_x(:,jj));
    cost_design=cost_design+cc;
end


%% Change the sign of control law matrices
KAC = -KAC;
K0 = -K0;


%% Save and display the results 
if save_num
    % Save name
    basic_info = append('step1_KAC_St',str_St,'_Ts',str_Ts);
    filename1=append(basic_info,'_ConvergenceK');
    saveas(10,filename1,'png') % if you want eps file, change 'png' to 'epsc'
    clear filename1
    save('step1_design_KAC.mat')
end


disp(['cost_design:    ',num2str(cost_design)])
disp(['KAC              : ',num2str(KAC)])
disp(['K*(LQR solution) : ',num2str(Kstar)])



