clc
clear all

% Define acoustic system:
L = 200;     % number of coefficients of acoustic paths
fs = 2000;   % sampling rate
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


% Simulation data:
% Number of simulation samples
Ite=150000;


% Variables to perform acoustic propagation
estado_s=zeros(L-1,1);
estado_p=zeros(L-1,1);


% Variables to monitor results:
% noise signal in the microphone
d=zeros(Ite,1);
% error signal at cancellation point:
error=zeros(Ite,1);



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

tr = 1;
for i = 1:tr

    % We generate a reference signal to read it dynamically:
    load("traffic.mat")
    ref = y;
    % ref = filter(fir1(101,[0.1, 0.2]),1,ref);
    e=0; % Microphone signal initialization


    % We declare the file with the initialization and configuration of the algorithm
    sigma_x = 0.003;
    sigma_y = 1;
    filtroF.epsilon = delta; % if the value is less that 0.000001, the algorithm does not work
    filtroF.mu = 1;
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
    filtroF.Q = sigma_x^2*eye(L);
    filtroF.R_n = sigma_y^2;
    filtroF.M = M;             % Window size
    filtroF.C = 0;             % Variance-Covariance matrix

    % Execution sample by sample


    for n=1:Ite

        x = ref(n); % Reference signal

        % We simulate the noise signal in the microphone

        [d(n),estado_p] = filter(Pri(:,Pos1)+0.0*randn(Ls,1),1,x,estado_p);


        % We read inputs from the algorithm
        filtroF.x = x;
        filtroF.e = e;
        % We execute the control algorithm
        filtroF = fx_eiaekf(filtroF);

        % We read algorithm outputs
        y = filtroF.y;

        % acoustic propagation

        [ys,estado_s]=filter(Sec(:,Pos1),1,y,estado_s);
        filtroF.s=Sec(:,Pos1);
        e=d(n)+ys;
        error(n)=e;
        err(n)=e;
    end
    if i>1
        er = err';
        e_com = [e_m err];
        e_com = reshape(e_com,[Ite,2]);
        e_m = mean(e_com,2);
        d_com = [d_m d];
        d_m = mean(d_com,2);
        a = 10 * log10((err.^2)./(d.^2));
        a_com = [a_m a];
        a_m = mean(a_com,2);

    else
        er = err';
        e_m = err;
        d_m = d;
        a_m = 10 * log10((err.^2)./(d.^2));
    end
end
error = e_m;
desired = d_m;
attenuation = a_m;
figure;
plot([desired error]);
% save("desired.mat","desired")
save("error_eiaekf.mat","error")
save("attenuation_eiaekf.mat","attenuation")

