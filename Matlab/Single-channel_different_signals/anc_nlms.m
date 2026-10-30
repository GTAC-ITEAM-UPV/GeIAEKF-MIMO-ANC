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
    Ite=18000;
    
    % Variables to perform acoustic propagation
    estado_s=zeros(L-1,1);
    estado_p=zeros(L-1,1);


% Variables to monitor results:
    % noise signal in the microphone
        d=zeros(Ite,1);
   % error signal at cancellation point:
        error=zeros(Ite,1);
    % Output signal  at cancellation point:
       output=zeros(Ite,1);
    


% Algorithm configuration:

    % General parameters

    % Number of adaptive filter samples
    L=200;
    % Number of samples of secondary path estimation
    Ls=L;
    % Cancellation algorithm convergence step
    mu=0.0005;
    % Regularization factor
    delta=1e-10;
    % Window size
    M = L+1;  



    

% We generate a reference signal to read it dynamically:
% rng(1);
% ref=randn(Ite,1);
load("multi_segment_signal_Fs2000.mat");
ref = x;
% ref = filter(fir1(101,[0.1, 0.2]),1,ref);
e=0; % Microphone signal initialization


% We declare the file with the initialization and configuration of the algorithm

filtroF.epsilon = 0.001; % if the value is less that 0.000001, the algorithm does not work
filtroF.mu = mu;
filtroF.y = 0;
filtroF.x = 0;
filtroF.d = d;
filtroF.s = zeros(L,1);
filtroF.w = zeros(L,1);
filtroF.state_w = zeros(L-1,1);
filtroF.buffS = zeros(L-1,1);
filtroF.buffL = zeros (L,1);

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
    filtroF = fx_nlms(filtroF);

% We read algorithm outputs
    y = filtroF.y;
   % Q_n = filtroF.Q_n;

% We collect additional data
% error2(n)=data.e2;    
  
   

% acoustic propagation

[ys,estado_s]=filter(Sec(:,Pos1),1,y,estado_s);
filtroF.s=Sec(:,Pos1);
e=d(n)-ys;
error(n)=e;
output(n) = y;

end

figure
plot([d error])
title('NLMS')
% plot(error)

%figure
%plot([error error2])

