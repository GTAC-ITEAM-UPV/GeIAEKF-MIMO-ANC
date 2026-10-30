% This script:
%   1) Builds the acoustic scenario (refs, mics, sec-paths) from ir_map.mat
%   2) Configures block sizes, filter lengths, and Kalman parameters
%   3) Runs the sample-by-sample acoustic simulation while adapting in blocks
%   4) Calls the chosen ANC algorithm per configuration (centralized)
%   5) Collects stats and visualizes results via analizaANC(signals)

clear all
clc

MaxIte = 40000;
a_m1 = zeros(MaxIte,1);
err1 = zeros(MaxIte,1);
a_m2 = zeros(MaxIte,1);
err2 = zeros(MaxIte,1);


path_delay = 70;
d_m1 = zeros(MaxIte-path_delay,1);
e_m1 = zeros(MaxIte-path_delay,1);
d_m2 = zeros(MaxIte-path_delay,1);
e_m2 = zeros(MaxIte-path_delay,1);

Pd_m1 = zeros(MaxIte-path_delay,1);
Pe_m1 = zeros(MaxIte-path_delay,1);
Pd_m2 = zeros(MaxIte-path_delay,1);
Pe_m2 = zeros(MaxIte-path_delay,1);

tr = 100;
smooth_factor=0.99;
for i = 1:tr
% We generate a reference signal to read it dynamically:
rng(i-1);                                    % reproducible noise
noise = randn(MaxIte,1);
noise = filter(fir1(100,[0.1, 0.9]),1,noise);

% Load secondary/primary path IR database (fs = 2 kHz, FIR length = 256)
load('ir_map.mat');                           % provides ir_map
ir_map = ir_map / max(max(max(max(ir_map)))); % normalized (optional)

% Choose noise source (loudspeaker index)
src = 65;

% Build 4 reference signals at positions y=15, x=14:11
ref_idx = 14:-1:11;
x  = zeros(length(noise), numel(ref_idx));
C1 = zeros(256, numel(ref_idx));              % store IRs used for refs
for nref = 1:numel(ref_idx)
    x(:,nref)  = filter(ir_map(src,:,nref,15), 1, noise);
    C1(:,nref) = ir_map(src,:,nref,15);
end

% Build 8 monitor microphone signals (control-off noise at sensors)
mic_idx = 16:-1:9;
d = zeros(length(noise), numel(mic_idx));
for nmic = 1:numel(mic_idx)
    d(:,nmic) = filter(ir_map(src,:,mic_idx(nmic),1), 1, noise);
end

% Secondary paths: from speakers to microphones
sec_idx = 17:24;   % pick 8 possible secondaries; we'll subset later
h = [];
for nsec = 1:numel(sec_idx)
    for nmic = 1:numel(mic_idx)
        h(:,nsec,nmic) = ir_map(sec_idx(nsec),:,mic_idx(nmic),1);
    end
end

% Specific simulation size (subset I,J,K from available)
MaxIte = min([size(d,1), size(x,1)]);  % max samples
Z = 1;                                  % #nodes (Z=1 => centralized)
I = 1; J = 2; K = 2;                    % refs, secondaries, error mics

h = h(:, 1:J, 1:K);                     % (M x J x K) acoustic model
Tamh = size(h,1);
s = h;                                  % perfect estimate of the model
Tams = size(s,1);

%% 2. CONTROL CONFIG & ADAPTIVE MODEL
TamA = 1;       % update stride (samples between coefficient updates)
TamB = 1;       % block length used for filtering/measurement
L    = 200;      % adaptive filter length

%% 3. ACOUSTIC SIMULATION BUFFERS
filtro_x = x(:,1:I);      % reference signals used by the controller
filtro_d = d(:,1:K);      % baseline noise at error mics

estado_w = zeros(L-1, I, J);          % internal states of adaptive filters
estado_h = zeros(Tamh-1, J, K);       % internal states for acoustic paths
e_mic    = zeros(1, K);               % mic outputs (per sample)

%% 4. RESULTS STRUCTURE (signals)
signals.d  = d(1:MaxIte, 1:K);        % control-off noise
signals.x  = x(1:MaxIte, 1:I);        % references
signals.y  = zeros(MaxIte, J);        % loudspeaker outputs
signals.e  = zeros(MaxIte + TamB, K); % mic error signals (control-on)
signals.fs = 2;                       % kHz
signals.str= 'ANC Z=1, I=1, J=2, K=2, TamA=TamB=1, L=200, fx-Kalman.';

%% 5. FILTER OBJECT INITIALIZATION
delta    = 1e-3;          % regularization (for AP forms)
mu       = 0.002;        % reserved for NLMS variants
buffL    = zeros(L+TamB-1, I, J, K);
buffS    = zeros(Tams-1,   I, J, K);
buffLd   = zeros(L+TamB-1, Z);
buffSd   = zeros(Tams-1,   Z);
predice  = 0;             % skip prediction stage by default
w        = zeros(L, I, J);
F_n      = [];            % no state transition
sigma_x  = 0.0001;
Q_n      = 0*sigma_x^2 * eye(L*I*J);

sigma_y  = 0.005;         % measurement noise stdev (stability vs bias)
R_n      = eye(TamB*K) * sigma_y^2;

P_n1     = (5e-5) * eye(L*I*J);    % initial state covariance
P_ni     = (1e-6) * eye(L*I*J);

% Initialization for eIAEKF algorithm
M = (L*I*J);            % Window size
epsilon = 0.0000001;
%epsilon = 0.00001; % For block processing
C = zeros(TamB*K,TamB*K);                % Variance-Covariance matrix
buffC = zeros(TamB,TamB,Z);
buffe = zeros(TamB,M,Z);

%% ---- Choose ONE of the following initializations ----

% -- Centralized fx-Kalman (propagate P) --
filtrokd = creaAF_ANC('mu',0.2,'s',s,'L',L, ...
    'buffL',buffL,'buffS',buffS,'w',w, ...
    'P_n1',P_n1, 'F_n',F_n,'Q_n',Q_n,'R_n',R_n, ...
    'TamA',TamA,'predice',predice,'fxmodificada',0,'dN',zeros(TamB*K,1), ...
    'tipo',1);

% 
% %-- Centralized fx-nlms NLMS --
% filtrokd = creaAF_ANC('mu',mu,'s',s,'L',L, ...
%     'buffL',buffL,'buffS',buffS,'w',w,'TamA',TamA,'predice',predice,'fxmodificada',0,'dN',zeros(TamB*K,1),'tipo',2);

% % -- Centralized Full Adaptive fx-generalized eIAEKF --
% filtrokd = creaAF_ANC('mu',4,'s',s,'L',L, ...
%     'buffL',buffL,'buffS',buffS,'w',w, ...
%     'P_n1',P_ni , 'F_n',F_n,'Q_n',Q_n,'R_n',R_n, ...
%     'TamA',TamA,'predice',predice,'fxmodificada',0,'dN',zeros(TamB*K,1),'M', M,'C',C, ...
%     'epsilon', epsilon, 'buffe',zeros(TamB*K,M,Z),'buffC',buffC ,'tipo',3);


% If I==1, you may choose to use the exact noise as reference (more favorable)
filtro_x = noise;   % comment this if I>1

% Timing per iteration (we aggregate per-node time)
tiempo = zeros(MaxIte,1);

barra = waitbar(0, 'Iniciando proceso...'); % 1. Inicializar




%% 6. MAIN SIMULATION LOOP (sample-by-sample acoustics, block adaptation)
for n = 1:MaxIte
     waitbar(n/MaxIte, barra, sprintf('Procesando %d de %d: %d%%', i,tr,floor(n/MaxIte*100)));

    % -- 6.1 Loudspeaker drive (filter the references through w) --
    yout = zeros(1, J);
    w_now = filtrokd(1).w;  % centralized default
    for nref = 1:I
        for nsec = 1:J
            if filtrokd(1).nodos > 0      % decentralized: per-node w
                w_now = filtrokd(nsec).w;
            end
            [out, estado_w(:,nref,nsec)] = filter( ...
                w_now(:,nref,nsec), 1, filtro_x(n,nref), estado_w(:,nref,nsec));
            yout(nsec) = yout(nsec) + out;
        end
    end
    signals.y(n,:) = yout;

    % -- 6.2 Acoustic propagation + summation at error mics --
    e_mic = filtro_d(n,:);               % start from original noise
    for nsec = 1:J
        for nmic = 1:K
            [out, estado_h(:,nsec,nmic)] = filter( ...
                h(:,nsec,nmic), 1, yout(nsec), estado_h(:,nsec,nmic));
            e_mic(nmic) = e_mic(nmic) + out;
        end
    end
    signals.e(n+TamB,:) = e_mic;

    % -- 6.3 Block adaptation (every TamA samples; requires TamB history) --
    if n >= TamB
        if mod(n-1, min(TamA,TamB)) == 0 && (n-1+TamB) < MaxIte
            error_in = signals.e(1+(n-1):(n-1)+TamB, :);  % (TamB x K)

            % Multi-pass distributed compute; in centralized, only node=1 runs
            filtrokd.x = filtro_x(1+(n-1):(n-1)+TamB, :);
            % ---- Centralized path ----
            filtrokd.e = error_in(:);   % vectorized K blocks
            tic;
            % (You may switch here to fx_kalman/fx_nlms/fx_geiaekf as needed)
            filtrokd = fx_kalman(filtrokd);
            tiempo(n) = toc;
        end
    end
end
% Cerrar la barra al finalizar
close(barra);

% Post-stats
signals.t   = tiempo(1+TamA:TamA:end) / max(1,Z);
signals.tm  = median(signals.t);

    err1 = signals.e(1 + path_delay+TamB:end,1);
    err2 = signals.e(1 + path_delay+TamB:end,2);
    d1 = signals.d(1+path_delay:end,1);
    d2 = signals.d(1+path_delay:end,2);
    
    %smooth
    Perr1 = filter(1-smooth_factor,[1 -smooth_factor],err1.^2,mean(err1(1:100).^2));
    Perr2 = filter(1-smooth_factor,[1 -smooth_factor],err2.^2,mean(err2(1:100).^2));

    Pd1 = filter(1-smooth_factor,[1 -smooth_factor],d1.^2,mean(d1(1:100).^2));
    Pd2 = filter(1-smooth_factor,[1 -smooth_factor],d2.^2,mean(d2(1:100).^2));
    

    e_m1 = e_m1 + err1/tr;
    d_m1 = d_m1 + d1/tr;
    %a1 = 10 * log10((err1.^2)./(d1.^2));
    %a_m1 = a_m1 + a1/tr;
    e_m2 = e_m2 + err2/tr;
    d_m2 = d_m2 + d2/tr;
    Pe_m1 = Pe_m1 + Perr1/tr;
    Pd_m1 = Pd_m1 + Pd1/tr;
    Pe_m2 = Pe_m2 + Perr2/tr;
    Pd_m2 = Pd_m2 + Pd2/tr;

    
    %a2 = 10 * log10((err2.^2)./(d2.^2));
    %a_m2 = a_m2 + a2/tr;
end

a_m1 = 10 * log10((Pe_m1)./(Pd_m1));
a_m2 = 10 * log10((Pe_m2)./(Pd_m2));

figure,plot([a_m1 a_m2])
title('Atenuation (dB)')

signals.e = [e_m1, e_m2];
signals.d = [d_m1, d_m2];
signals.a = [a_m1, a_m2];

static_skf = signals;
save('static_skf2.mat','static_skf');

% Visualize (user function)
analizaANC(signals);