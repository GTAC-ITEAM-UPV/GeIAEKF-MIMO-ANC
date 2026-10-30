function Mult = estimaMult(filtro)
% estimaMult  Estimate multiply counts for different ANC-Kalman variants
%
%   Mult = estimaMult(filtro)
%
% INPUT
%   filtro : struct or struct array; only filtro(1) is read for config:
%       x (N x I), s (M x J x K), w (L x ...), TamA, tipo,
%       fxmodificada (0: fx, 1: fx-modified), predice (0/1), AP (0/1)
%
% OUTPUT
%   Mult : scalar − estimated multiplies per iteration (per node if decentralized).
%
% tipo mapping
%   0 : LMS baseline
%   1 : Centralized Kalman, propagate P directly (AP via AP flag)
%   2 : Centralized Kalman, propagate P directly (general)
%   3 : Centralized Kalman, propagate inv(P)
%   4 : Decentralized (2 passes), fx/fx-modified
%   5 : Decentralized (3 passes), fx/fx-modified
%   6 : Centralized Riccati
%   7 : Decentralized Riccati
%
% Notes: The expressions are preserved from your original model; only
% formatting/comments and a few small guards were added.

cfg = filtro(1);

% Dimensions
[N, I]   = size(cfg.x);     %#ok<NASGU>
[M, J, K]= size(cfg.s);     %#ok<ASGLU> % Only to read M,J,K
[L, ~, ~]= size(cfg.w);

TamA     = min(getOrDefault(cfg,'TamA',N), N);
tipo     = getOrDefault(cfg,'tipo',0);
fxm      = getOrDefault(cfg,'fxmodificada',0) ~= 0;   % boolean
pred     = getOrDefault(cfg,'predice',0) ~= 0;        % boolean
AP       = getOrDefault(cfg,'AP',0) ~= 0;             % boolean

% Centralized filtering costs
Fc          = I .* TamA .* K .* J .* M;           % sec-path filtering
Pred        = (L.*I.*J).^2 + 2 .* (L.*I.*J).^3;   % prediction stage
filtrados   = I .* J .* TamA .* L;                % adaptive filtering

% fx/fxm centralized
fx_d  = 0;
fxm_d = K .* N .* L .* I .* J + K .* TamA .* L .* I .* J; % fx-modified direct P
fx_i  = K .* N .* L .* I .* J;                             % fx direct inv(P)
fxm_i = K .* TamA .* L .* I .* J;                          % fx-modified inv(P)

% Kalman centralized cores
nucleo_kalman_d = 2.*(L.*I.*J).^2 .* K.*N + 2.*L.*I.*J.*(K.*N).^2 + ...
                  L.*I.*J.*K.*N + (L.*I.*J).^3 + 2.*(K.*N).^2;
nucleo_kalman_i = K.*N.*(L.*I.*J).^2 + 2.*L.*I.*J.*(K.*N).^2 + ...
                  2.*(L.*I.*J).^2 + (L.*I.*J.*K.*N) + 2.*(L.*I.*J).^3;

% Decentralized
Z = K;  % number of nodes often equals #error sensors in your configuration

% Kalman decentralized cores
nucleo_kalman_2p = 3 .* L .* Z .* N .* N + (L.*Z).^2;
nucleo_kalman_3p = L .* Z .* N .* N + L .* (Z.^2) .* N + ...
                   2 .* Z .* N.^2 + Z .* L.^2 + L .* Z .* N + Z .* N;

% Filtering costs (decentralized)
filtradosdc = TamA .* L;
Fc2p        = Z .* M .* TamA;
Fc3p        = (2.*Z - 1) .* M .* TamA;

% fx/fxm (decentralized)
fx  = L .* Z .* N;
fxm = L .* Z .* TamA;

% Matrix inversion choices (kept as in your model)
cinv1 = 2 .* (L.*Z).^3; % direct inverse
% cinv2: inversion lemma route (kept commented in your source)
cinv2 = 2.*Z.*N.*Z.*L.*Z.*L + 2.*Z.*N.*Z.*N.*Z.*L + 2.*(N.*Z).^3; %#ok<NASGU>

% ---------------------------
% Switch by algorithm "tipo"
% ---------------------------
switch tipo
    case 0
        % LMS (reference baseline)
        Mult = TamA.*L.*I.*J + M.*I.*J.*K + N.*L.*I.*J + K;

    case 6
        % Centralized Riccati
        Mult = Fc + fxm_d*fxm + K.*( (L.*J).^3 + 2.*N.*(L.*J).^2 + L.*J.*N.*N + ...
                L.*J.*N + 2.*(N).^3 - AP*(L.*J).^3 + N.*(L.*J).^2 );

    case 7
        % Decentralized Riccati
        Mult = Fc2p + fx*(~fxm) + fxm*fxm + ...
               ( (L.*J).^3 + 2.*N.*(L.*J).^2 + L.*J.*N.*N + L.*J.*N + 2.*(N).^3 ...
                 - AP*(L.*J).^3 + N.*(L.*J).^2 );

    case 1
        % Centralized Kalman, propagate P directly (AP via AP flag)
        Mult = Fc + nucleo_kalman_d + filtrados + fxm_d*fxm + Pred*pred ...
               - (L.*I.*J).^3 - K.*N.*(L.*I.*J).^2 + K.*N;

    case 2
        % Centralized Kalman, propagate P directly (general)
        Mult = Fc + nucleo_kalman_d + filtrados + fxm_d*fxm + Pred*pred ...
               - AP*(L.*I.*J).^3 - K.*N.*(L.*I.*J).^2 + K.*N;

    case 3
        % Centralized Kalman, propagate inv(P)
        Mult = Fc + nucleo_kalman_i + filtrados + fx_i*(~fxm) + fxm_d*fxm + ...
               Pred*pred + AP*(L.*J.*K.*N);

    case 4
        % Decentralized (2 passes), fx/fx-modified
        Z = K;
        Mult = nucleo_kalman_2p + Fc2p + filtradosdc + ...
               fx*(~fxm) + fxm*fxm + Pred*pred + cinv1 + (Z.*L).^2;

    case 5
        % Decentralized (3 passes), fx/fx-modified
        Z = K;
        Mult = nucleo_kalman_3p + Fc3p + filtradosdc + ...
               fx*(~fxm) + fxm*fxm + Pred*pred + cinv1;

    otherwise
        error('Unknown tipo = %d', tipo);
end
end

% ---------- helper ----------
function v = getOrDefault(s, name, def)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = def;
end
end
