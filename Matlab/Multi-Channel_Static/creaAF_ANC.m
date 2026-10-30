function out = creaAF_ANC(varargin)
% creaAF_ANC  Build a configuration/state struct for ANC algorithms
%
%   out = creaAF_ANC('FieldName1', value1, 'FieldName2', value2, ...)
%
% The returned struct is used by different ANC algorithms:
%   fx_melms, fx_kalman, fx_kalman2, fx_kalmandec2p, fx_kalmandec3p, ...
%
% Notes
% - All fields are defined below with sensible defaults.
% - You can override any field by passing name/value pairs.
% - For decentralized algorithms, set out.nodos>0. The function will then
%   replicate the struct into a vector (one per node) and assign id_nodo.

% ----------------------------
% Defaults (can be overridden)
% ----------------------------

% Core adaptive filter settings
out.L        = 100;                 % Number of adaptive filter taps
out.s        = [0 0 0 1];           % Secondary-path IR estimates (M x J x K or simple vector)
out.x        = randn(1000,1);       % Reference signal buffer (N x I)
out.d        = filter([0 1],1,out.x); % Noise signal buffer (N x K) — usually unused by ANC algorithms
out.mu       = 1;                   % Step-size (for LMS-like algorithms)

% Time-domain buffers (fx structure)
out.buffL    = zeros(out.L,1);      % Buffer of last L reference samples filtered by s
out.buffS    = zeros(4,1);          % Filter states for reference filtered by s
out.w        = zeros(out.L,1);      % Adaptive filter coefficients (L x I x J typically reshaped)
out.e        = [];                  % Error signal(s) at microphones (N x K)
out.tipo     = 0;                   % Algorithm family tag (see your code for meanings)

% Frequency-domain (not used by the provided files, kept for completeness)
out.SP       = [];
out.W        = [];
out.X        = [];
out.in2      = [];
out.in2fx    = [];
out.BuffL    = [];
out.beta     = 0.9;                 % Forgetting factor for power estimation
out.Pot0     = [];
out.Potx     = [];
out.npromedia= 1;
out.delta    = 1e-6;                % Regularization for matrix inversions in step-size normalization

% Update stride
out.TamA     = 1;                   % Samples between updates

% Kalman-related fields
out.P_n1     = [];                  % State covariance (or its inverse, depending on variant)
out.F_n      = [];                  % State transition matrix
out.Q_n      = [];                  % Process noise covariance
out.R_n      = [];                  % Measurement noise covariance (or its inverse)
out.predice  = 0;                   % Whether to perform prediction stage
out.fxmodificada = 0;               % 1: use e to reconstruct d internally (fx-modified)
out.dN       = [];                  % Buffer for reconstructed noise (when fxmodificada=1)
out.AP       = 0;                   % 1: Affine-Projection specialization
out.lambda   = 1;                   % Forgetting factor (if used)

% Decentralized system settings
out.nodos    = 0;                   % # of nodes (0/1 = centralized)
out.id_nodo  = 0;                   % Node id (1..nodos)
out.buffLd   = [];                  % L+TamB-1 x Z filtered references needed by current node
out.buffSd   = [];                  % States for the s-filtering at each node
out.pasada   = 1;                   % Current pass index (for multi-pass decentralized)
out.numpass  = 1;                   % Total passes (2 or 3)
out.pasadatos= [];                  % Payload communicated between nodes per pass
out.w_p = [];
out.P_p = [];
out.M = 0;
out.epsilon = 0;
out.C = [];       % Variance-Covariance matrix 
out.buffe = [];
out.buffC = [];


% Inverse-P propagation helpers
out.Lema     = 0;                   % Use matrix inversion lemma for covariance (when propagating invP)
out.Neumann  = -1;                  % Neumann recursion approx for inverses (<0: off; 0: order0; >0: order1)
out.P0       = 1;                   % Initial diag value for inverse approx (lemma/Neumann)

% ----------------------------
% Parse name/value overrides
% ----------------------------
if mod(nargin,2) ~= 0
    error('Arguments must be name/value pairs.');
end

for k = 1:2:nargin
    key = varargin{k};
    val = varargin{k+1};
    if isfield(out, key)
        out.(key) = val;                   % modern dynamic field assignment
    else
        error('%s is not a valid field of AF_ANC.', key);
    end
end

% ---------------------------------------------
% Replicate for decentralized systems if needed
% ---------------------------------------------
if out.nodos > 0
    numnodos = out.nodos;
    base = out;
    out = repmat(base, numnodos, 1);
    for nodo = 1:numnodos
        out(nodo).id_nodo = nodo;
    end
end
end
