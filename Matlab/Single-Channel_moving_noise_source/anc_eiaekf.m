clc
clear all

% Define acoustic system:
L = 200;     % number of coefficients of acoustic paths
fs = 2000;   % sampling rate
c = 340;     % Velocity of sound

% Define room and paths:

Ns = 7;         % Maximum number of acoustic systems
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

% Position 2 (changes noise source to 20cm left)
Pos_altr=[1.2,3,1.5];

% calculate primary path
Pri(:,2) = rir_generator(c, fs, Pos_altr, Pos_mic, Sala, Rv, L)';

% calculate secondary path
Sec(:,2) = rir_generator(c, fs, Pos_alts, Pos_mic, Sala, Rv, L)';
%Sec(:,2) = Sec(:,1);


% Position 3 (changes noise source to 40cm left)
Pos_altr=[1.4,3,1.5];


% calculate primary path
Pri(:,3) = rir_generator(c, fs, Pos_altr, Pos_mic, Sala, Rv, L)';

% calculate secondary path
Sec(:,3) = rir_generator(c, fs, Pos_alts, Pos_mic, Sala, Rv, L)';

% Position 4 (changes noise source to 60cm left)
Pos_altr=[1.6,3,1.5];


% calculate primary path
Pri(:,4) = rir_generator(c, fs, Pos_altr, Pos_mic, Sala, Rv, L)';

% calculate secondary path
Sec(:,4) = rir_generator(c, fs, Pos_alts, Pos_mic, Sala, Rv, L)';

% Position 5 (changes noise source to 10cm up)
Pos_altr=[1,3.1,1.5];


% calculate primary path
Pri(:,5) = rir_generator(c, fs, Pos_altr, Pos_mic, Sala, Rv, L)';

% calculate secondary path
Sec(:,5) = rir_generator(c, fs, Pos_alts, Pos_mic, Sala, Rv, L)';

% Position 6 (changes noise source to 20cm up)
Pos_altr=[1,3.2,1.5];


% calculate primary path
Pri(:,6) = rir_generator(c, fs, Pos_altr, Pos_mic, Sala, Rv, L)';

% calculate secondary path
Sec(:,6) = rir_generator(c, fs, Pos_alts, Pos_mic, Sala, Rv, L)';

% Position 7 (changes noise source to 30cm up)
Pos_altr=[1,3.3,1.5];


% calculate primary path
Pri(:,7) = rir_generator(c, fs, Pos_altr, Pos_mic, Sala, Rv, L)';

% calculate secondary path
Sec(:,7) = rir_generator(c, fs, Pos_alts, Pos_mic, Sala, Rv, L)';

Pos1=1;
Pos2=2;
Pos3=3;
Pos4=4;
Pos5=5;
Pos6=6;
Pos7=7;
% Pos8=8;
% Pos9=9;


% Simulation data:
% Number of simulation samples
Ite=300000;

% Samples in which changes occur in the system
Ite2=40000;
Ite3=80000;
Ite4=120000;
Ite5=160000;
Ite6=200000;
Ite7=240000;

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
M = L+30;
%M = 1;


d_m = zeros(Ite,1);
e_m = zeros(Ite,1);
a_m = zeros(Ite,1);
err = zeros(Ite,1);

tr = 100;
for i = 1:tr

    % We generate a reference signal to read it dynamically:
    rng(i-1);
    ref=randn(Ite,1);
    ref = filter(fir1(101,[0.1, 0.9]),1,ref);
    % load('traffic.mat');
    % ref = y;
    e=0; % Microphone signal initialization


    % We declare the file with the initialization and configuration of the algorithm
    sigma_x = 0.0001;
    sigma_y = 1;
    filtroF.epsilon = delta; % if the value is less that 0.000001, the algorithm does not work
    filtroF.y = 0;
    filtroF.x = 0;
    filtroF.d = d;
    filtroF.s = zeros(L,1);
    filtroF.w = zeros(L,1);
    filtroF.state_w = zeros(L-1,1);
    filtroF.buffS = zeros(L-1,1);
    filtroF.buffL = zeros (L,1);
    filtroF.buffe = zeros(1,M);
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
        if n<Ite2
            [d(n),estado_p] = filter(Pri(:,Pos1),1,x,estado_p);
        elseif n<Ite3
            [d(n),estado_p] = filter(Pri(:,Pos2),1,x,estado_p);
        elseif n<Ite4
            [d(n),estado_p] = filter(Pri(:,Pos3),1,x,estado_p);
        elseif n<Ite5
            [d(n),estado_p] = filter(Pri(:,Pos4),1,x,estado_p);
        elseif n<Ite6
            [d(n),estado_p] = filter(Pri(:,Pos5),1,x,estado_p);
        elseif n<Ite7
            [d(n),estado_p] = filter(Pri(:,Pos6),1,x,estado_p);
        else
            [d(n),estado_p] = filter(Pri(:,Pos7),1,x,estado_p);
        end


        % We read inputs from the algorithm
        filtroF.x = x;
        filtroF.e = e;
        % Control Algorithm
        filtroF = fx_eiaekf(filtroF);

        % We read algorithm outputs
        y = filtroF.y;

        % acoustic propagation
        if n<Ite2
            [ys,estado_s]=filter(Sec(:,Pos1),1,y,estado_s);
            filtroF.s=Sec(:,Pos1);
        elseif n<Ite3
            [ys,estado_s]=filter(Sec(:,Pos2),1,y,estado_s);
            filtroF.s=Sec(:,Pos1);
        elseif n<Ite4
            [ys,estado_s]=filter(Sec(:,Pos3),1,y,estado_s);
            filtroF.s=Sec(:,Pos1);
        elseif n<Ite5
            [ys,estado_s]=filter(Sec(:,Pos4),1,y,estado_s);
            filtroF.s=Sec(:,Pos1);
        elseif n<Ite6
            [ys,estado_s]=filter(Sec(:,Pos5),1,y,estado_s);
            filtroF.s=Sec(:,Pos1);
        elseif n<Ite7
            [ys,estado_s]=filter(Sec(:,Pos6),1,y,estado_s);
            filtroF.s=Sec(:,Pos1);
        else
            [ys,estado_s]=filter(Sec(:,Pos7),1,y,estado_s);
            filtroF.s=Sec(:,Pos1);
        end
        e=d(n)+ys;
        error(n)=e;
        err(n)=e;
    end
    e_m = e_m + err/tr;
    d_m = d_m + d/tr; 
    a = 10 * log10((err.^2)./(d.^2));
    a_m = a_m + a/tr;


end
error = e_m;
desired = d_m;
attenuation = a_m;
figure;
plot([desired error]);
save("desired_eiaekf.mat","desired")
save("error_eiaekf.mat","error")
save("attenuation_eiaekf.mat","attenuation")
