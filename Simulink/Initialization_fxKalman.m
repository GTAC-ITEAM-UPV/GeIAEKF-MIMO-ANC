clc;
clear all;
% Load Secondary path estimations

load('S11.mat');   % Actual Coefficients to repesent Secondary path 11
load('S12.mat');   % Actual Coefficients to repesent Secondary path 12
load('S21.mat');   % Actual Coefficients to repesent Secondary path 21
load('S22.mat');   % Actual Coefficients to repesent Secondary path 22


% ANC system configuration
fs= 1000;                % Sampling frequency
Ts = 1/fs;               % Sampling time
N =1;                    % No. of samples per block
len_measure = N;
C = 1;                   % No. of channels
num_targets = C;
I = 1;                   % Number of reference microphone
J = 2;                   % Number of secondary loudspeaker
K = 2;                   % Number of error microphone
L= 70;                   % Number of Secondary Path coefficients
M = 200;                 % Number of Primary path coefficients 

len_state = M;              % Number of Primary states
len_stateS = L;             % Number of Secondary states


% Definition of the state model of primary path

F_np = eye(M);                       % Matrix of Transition between states
sigma_q=0.00002;   
%sigma_q=0.0002;                      % Standard deviation of the noise in the transition model between states.
Q_np=sigma_q^2*eye(M);               % Covariance matrix.

sigma_r=0.001;                       % Standard deviation of the measurement noise (the larger it is, but it will be the final error found in the Kalman filtering, but if it is very small, the algorithm could become unstable)
R_np=(sigma_r^2)*eye(K*N);           % Measurement noise covariance matrix

% Initialization Primary
v = 0.01;
n=zeros(M,C);                       % Initial states of primary path
%P=0.0001*eye(M);
P = (5e-6)*eye(M);
% p=(v^2)*eye(M);                   % Primary path state covariance matrix  
p = reshape (P,M^2,1);
mu1 = 0.5;
mu2 = 0.1;
m = 4;

% mu1 = 0;
% mu2 = 0;




    
   