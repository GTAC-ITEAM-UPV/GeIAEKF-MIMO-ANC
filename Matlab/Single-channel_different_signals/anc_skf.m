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
        
            

% % Physics-informed:
% Epm=mean(mean(Pri.^2));  % Average energy of primary paths
% Esm=mean(mean(Sec.^2));  % Average energy of secondary paths
% S_1 = max(Sec(1,:));     % Proposed value for the initialization of secondary paths
% 
% 
% Ewm=Epm/Esm;             % Estimation of the average energy of the adaptive filter

% Simulation data:
    % Number of simulation samples
    Ite=18000;
    
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
    delta=1e-10;
    % Window size
    M = L+1;  



    

% We generate a reference signal to read it dynamically:
load("multi_segment_signal_Fs2000.mat");
ref = x;
e=0; % Microphone signal initialization


% We declare the file with the initialization and configuration of the algorithm
sigma_x = 0.00001;
sigma_y = 0.005;
filtroF.y = 0;
filtroF.x = 0;
filtroF.d = d;
filtroF.s = zeros(L,1);
filtroF.w = zeros(L,1);
filtroF.state_w = zeros(L-1,1);
filtroF.buffS = zeros(L-1,1);
filtroF.buffL = zeros (L,1);
filtroF.buffe = zeros(M,1);
filtroF.P_n1= (1e-7)*eye(L);
filtroF.F_n = eye(L);
filtroF.Q_n = sigma_x^2*eye(L);
filtroF.R_n = sigma_y^2;
filtroF.M = M;             % Window size
filtroF.C = 0;             % Variance-Covariance matrix 


% ini=['ini_',alg];
% 
% % We initialize the algorithm
% eval(ini);

% Execution sample by sample


for n=1:Ite
    
    x = ref(n); % Reference signal
    
    % We simulate the noise signal in the microphone

        [d(n),estado_p] = filter(Pri(:,Pos1)+0.0*randn(Ls,1),1,x,estado_p);



% We read inputs from the algorithm
    filtroF.x = x;
    filtroF.e = e;
% We execute the control algorithm
    % eval(['data=',alg,'(data);']);
    filtroF = fx_kalman(filtroF);
% We read algorithm outputs
    y = filtroF.y;

% We collect additional data
% error2(n)=data.e2;    
  
   

% acoustic propagation

[ys,estado_s]=filter(Sec(:,Pos1),1,y,estado_s);
filtroF.s=Sec(:,Pos1);
e=d(n)+ys;
error(n)=e;

end

figure
plot([d error])
title('Standard Kalman filter')
% plot(error)

%figure
%plot([error error2])

