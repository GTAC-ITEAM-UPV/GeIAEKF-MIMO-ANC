clc
clear all

% Define acoustic system:
L = 200;     % number of coefficients of acoustic paths
fs = 8000;   % sampling rate
c = 340;     % Velocity of sound

% Define room and paths:

Ns = 3;         % Maximum number of acoustic systems
Pri=zeros(L,Ns);
Sec=zeros(L,Ns);


Sala=[9.13 4.48 2.64]; % Room dimensions
Rv=0.2; % Reverb


% Position 1

Pos_altr=[1,3,1.5];
Pos_mic=[8.4,2.44,1.5];
Pos_alts=[8.5,2.54,1.5];

% calculate primary path
Pri(:,1) = rir_generator(c, fs, Pos_altr, Pos_mic, Sala, Rv, L)';

% calculate secondary path
Sec(:,1) = rir_generator(c, fs, Pos_alts, Pos_mic, Sala, Rv, L)';


Pos1=1;



% % Physics-informed:
% Epm=mean(mean(Pri.^2));  % Average energy of primary paths
% Esm=mean(mean(Sec.^2));  % Average energy of secondary paths
% S_1 = max(Sec(1,:));     % Proposed value for the initialization of secondary paths
%
%
% Ewm=Epm/Esm;             % Estimation of the average energy of the adaptive filter

% Simulation data:
% Number of simulation samples
Ite=50000;

% % Samples in which changes occur in the system
% Ite2=50000;
% Ite3=100000;

% Variables to perform acoustic propagation
estado_s=zeros(L-1,1);
estado_p=zeros(L-1,1);


% Variables to monitor results:
% noise signal in the microphone
d=zeros(Ite,1);
% error signal at cancellation point:
error=zeros(Ite,1);
% error signal in the identification system:
% error2=zeros(Ite,1);



% Algorithm configuration:

% General parameters

% Number of adaptive filter samples
L=200;
% Number of samples of secondary path estimation
Ls=L;
% Cancellation algorithm convergence step
mu=0.25;
% Regularization factor
delta=0.0008;
% Window size
M = L+1;
d_m = zeros(Ite,1);
e_m = zeros(Ite,1);
a_m = zeros(Ite,1);
err = zeros(Ite,1);
Pd_m = zeros(Ite,1);
Pe_m = zeros(Ite,1);

smooth_factor=0.99;
tr = 100;
for i = 1:tr


    % We generate a reference signal to read it dynamically:
    rng(i-1);
    ref=randn(Ite,1);
    ref = filter(fir1(101,[0.1, 0.2]),1,ref);
    e=0; % Microphone signal initialization


    % We declare the file with the initialization and configuration of the algorithm
    sigma_x = 0.0001;
    sigma_y = 1;
    filtroF.mu = mu;
    filtroF.epsilon = delta; % if the value is less that 0.000001, the algorithm does not work
    filtroF.y = 0;
    filtroF.x = 0;
    filtroF.d = d;
    filtroF.s = zeros(L,1);
    filtroF.w = zeros(L,1);
    filtroF.state_w = zeros(L-1,1);
    filtroF.buffS = zeros(L-1,1);
    filtroF.buffL = zeros (L,1);
    filtroF.buffe = zeros(M,1);
    filtroF.P_n1= (5e-6)*eye(L); % if the vale is greater or less than (5e-6), the algorithm doen not behave well
    filtroF.F_n = eye(L);
    filtroF.Q_n = sigma_x^2*eye(L);
    filtroF.R_n = sigma_y^2;
    filtroF.M = M;             % Window size
    filtroF.C = 0;             % Variance-Covariance matrix

    % Execution sample by sample


    for n=1:Ite

        x = ref(n); % Reference signal

        % We simulate the noise signal in the microphone
        rng(1);

        [d(n),estado_p] = filter(Pri(:,Pos1)+0.0001*randn(Ls,1),1,x,estado_p);



        % We read inputs from the algorithm
        filtroF.x = x;
        filtroF.e = e;
        % We execute the control algorithm
        % eval(['data=',alg,'(data);']);
        filtroF = fx_iaekf_fQ(filtroF);

        % We read algorithm outputs
        y = filtroF.y;
        % Q_n = filtroF.Q_n;

        % We collect additional data
        % error2(n)=data.e2;



        % acoustic propagation

        [ys,estado_s]=filter(Sec(:,Pos1),1,y,estado_s);
        filtroF.s=Sec(:,Pos1);
        e=d(n)+ys;
        error(n)=e;
        err(n)=e;

    end

     Perr = filter(1-smooth_factor,[1 -smooth_factor],err.^2,mean(err(1:100).^2));
    Pd = filter(1-smooth_factor,[1 -smooth_factor],d.^2,mean(d(1:100).^2));

    e_m = e_m + err/tr;
    d_m = d_m + d/tr;

    Pe_m = Pe_m + Perr/tr;
    Pd_m = Pd_m + Pd/tr;
end
a_m = 10 * log10((Pe_m)./(Pd_m));
error = e_m;
desired = d_m;
attenuation = a_m;
figure
plot([desired error]);
save("desired_eiaekf_m1.mat","desired")
save("error_eiaekf_m1.mat","error")
save("attenuation_eiaekf_m1.mat","attenuation")