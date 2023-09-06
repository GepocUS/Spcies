%% example_OscMass - Benchmark for the oscillating masses system
% 
% This function returns a specific example of the oscillating masses
% system, which is used in several of the tutorial and examples provided
% in the toolbox.
% It also returns a choice of MPC ingredients that work well for this system,
% which is also used in several of the examples of the toolbox.
%
% OUTPUTS:
% - sys: Structure containing the system information (in spcies_gen_controller format)
% - param: Structure containing the MPC ingredients (in spcies_gen_controller format)
% 

function [sys, param] = example_OscMass()

    % Parameters of the system
    p = 3; % Number of objects
    M = [1; 0.5; 1]; % Mass of each object
    K = 2*ones(p+1, 1); % Spring constant of each spring
    F = [1; zeros(p-2, 1); 1]; % Objects in which an external force is applied

    % Generate continuous-time model
    sysC = utils.gen_oscillating_masses(M, K, F);

    n = size(sysC.A, 1); % For convenience, we save the size of the state dimension
    m = size(sysC.B, 2); % and of the input dimension

    % We obtain a discrete-time state space model
    Ts = 0.2; % Select the sample time
    sysD = c2d(sysC, Ts); % Discrete-time state space model

    % We set some bounds for the state and input
    LBx = -[ones(p, 1); 1000*ones(p, 1)]; % Lower bound for the state
    UBx = [0.3; 0.3; 0.3; 1000*ones(p, 1)]; % Upper bound for the state
    LBu = -[0.8; 0.8]; % Lower bound for the input
    UBu = [0.8; 0.8]; % Upper bound for the input

    % Scaling matrices
    Nx = ones(n, 1);
    Nu = ones(m, 1);

    % Operating point
    x0 = zeros(n, 1);
    u0 = zeros(m, 1);

    sys = struct('A', sysD.A, 'B', sysD.B, 'LBx', LBx, 'UBx', UBx, 'LBu', LBu, 'UBu', UBu,...
                 'x0', x0, 'u0', u0, 'Nx', Nx, 'Nu', Nu);

    % MPC ingredients
    Q = blkdiag(15*eye(p), 1*eye(p)); % Remember that Q and R must be positive definite
    R = 0.1*eye(m);
    [~, T] = dlqr(sys.A, sys.B, Q, R); % This is the typical choice of T in MPC
    N = 10;

    param = struct('Q', Q, 'R', R, 'T', T, 'N', N);

end

