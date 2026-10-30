clc;
clear all;
% 'Static' Channel Identification with near end signals
load('S1.mat');
load('S2.mat');

% Channel configuration to estimate:
% ANC system configuration
fs= 1000;                % Sampling frequency
Ts = 1/fs;               % Sampling time
B = 1;                   % No. of samples per block
len_measure = B;
C = 1;                   % No. of channels
num_targets = C;
I = 1;                   % Number of reference microphone
J = 1;                   % Number of secondary loudspeaker
K = 1;                   % Number of error microphone
M = 70;                  % Number of Secondary Path coefficients
L = 200;
N = L*I*J;               % Number of Primary path coefficients 

len_state = N;              % Number of Primary states
len_stateS = L;             % Number of Secondary states


% Definition of the state model
               
F_n = 1.0001*eye(N);  % Matrix of Transition between states
sigma_q=0.00002;       % Standard deviation of the noise in the transition model between states.
Q=sigma_q^2*eye(N);    % Covariance matrix.
Q_n=sigma_q^2*eye(N);  % Covariance matrix.

sigma_r=0.001;                       % Standard deviation of the measurement noise (the larger it is, but it will be the final error found in the Kalman filtering, but if it is very small, the algorithm could become unstable)
R_n=(sigma_r^2)*eye(K*B);           % Measurement noise covariance matrix

X_n=zeros(N,K*B);
H_n=X_n';                % Serves as the measure matrix

epsilon = 0.000001*eye(K*B);

alpha = 0.9999;

% Inicialization
m(:,1)=zeros(N,1);
P = (5e-6)*eye(N);
p = reshape (P,[N^2,1]);
% P=0.0001*eye(N);
% p=(v^2)*eye(M);                   % Primary path state covariance matrix  
p = reshape (P,N^2,1);

mu1 = 2;
mu2 = 1;
n = 1;


    
   