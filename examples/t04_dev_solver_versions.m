%% Tutorial: The non-sparse Matlab solvers
%
% This tutorial covers the basics of the non-sparse version of the solvers
% available in the SPCIES toolbox.
%
% Before reading through this tutorial we recommend you first take a look
% at the basic tutorial (basic_tutorial.m).
%
% The main objective of the SPCIES toolbox y to generate sparse tailored
% solvers for different MPC formulations. These solvers are based on 
% optimized code that is difficult to understand, to change or to extract
% additional information from (beyond the outputs of the function, that is).
%
% To alleviate this issue, the SPCIES toolbox includes non-sparse versions of
% all its solvers written in plan Matlab code. These solvers, which are 
% stored in the folder $SPCIES$/platforms/Matlab, where $SPCIES$ is the root
% directory of the toolbox (call spcies_get_root_directory), provide several
% advantages and disadvantages with respect to the sparse solvers generated
% using the spcies_gen_controller function:
%
% ADVANTAGES:
%   - The non-sparse solvers return more information than the sparse versions.
%     For instance, they can be asked to return the historic of the values
%     of the decision variables during its iterations (see below for how
%     to do this).
%   - The Matlab code is easier to follow and understand than the sparse code.
%     This allows the user to follow along with what the algorithm is doing,
%     especially if it is compared to the articles and other documentation
%     explaining the solvers. It also helps the user to understand what the
%     sparse code is doing.
%     This is only useful if your are interested in understanding the solver.
%   - It facilitates making changes to the algorithm to test new ideas or suit
%     your particular needs. This is interesting if you are doing research or
%     need access to data that is hard to extract from the sparse solver.
%   - It allows for debugging using Matlab's debugging tools.
%   - It does not require a compiler to generate the solver. It directly runs
%     in Matlab.
%
% DISADVANTAGES:
%   - The non-sparse solver is significantly slower that the sparse version.
%     Especially so if repeatedly called, since it performs all the offline 
%     computations each times it is called, whereas the sparse version only
%     performs them when the spcies_gen_controller function is called.
%   - It can only be executed in Matlab, whereas the sparse solvers can be
%     generated for different programming languages and target platforms.
%
% In short, these functions are useful if you want to be able to debug, make
% changes to the algorithm or need data not returned by the sparse version.
%
% This tutorial will show an example of how to use one of these functions.
% The usage of the others is very similar.
% The reader will not that the procedure is very similar to the one used for
% the sparse versions (see basic_tutorial.m).
%
% In this tutorial we will use the solver spcies_MPCT_EADMM_solver, which
% is a solver, based on the EADMM algorithm, for the MPC formulation:
%
% min_{x, u, xs, us} sum_{i=0}^{N-1} \| x_i - xs \|^2_Q + \| u_i - us \|^2_R +
%                    + \| xs - xr \|^2_T + \| us - ur \\^2_S
%
%   s.t. x_0 = x(k)
%        x_{i+1} = A x_i + B u_i, i = 0...N-1
%        LBx <= x_i <= UBx, i = 1...N-1
%        LBu <= u_i <= UBu, i = 0...N-1
%        xs = A xs + B us
%        LBx + epsilon_x <= xs <= UBx - epsilon_x
%        LBu + epsilon_u <= us <= UBu - epsilon_u
%        x_N = xs
%
% where x_i and u_i are the predicted states and control actions, respectively;
% x_r and u_r are the state and input reference, respectively; xs and us are
% artificial reference, Q, R, T and S, the positive definite cost function matrices;
% LBx, LBu are the lower bounds for the state and input, respectively; UBx, UBu the
% upper bounds; epsilon_x and epsilon_u vectors with arbitrarily small components
% and N is the prediction horizon.
%
% For additional information about the formulation, which we label MPCT, we refer
% the reader to (and the references therein):
%
% "Implementation of model predictive control for tracking in embedded systems
% using a sparse extended ADMM algorithm", by P. Krupa, I. Alvarado, D. Limon
% and T. Alamo, arXiv preprint: 2008:09071v2, 2020.
% 
clear; clc;

%% STEP 1: Problem definition.
% We consider the same system of connected masses that we used in the basic_tutorial.

p = 3; % Number of objects
M = [1; 0.5; 1]; % Mass of each object
K = 2*ones(p+1, 1); % Spring constant of each spring
F = [1; zeros(p-2, 1); 1]; % Objects in which an external force is applied

sysC = sp_utils.gen_oscillating_masses(M, K, F); % Generate the continuous-time model

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

% The ingredients that define the state space model have to be saved into a structure.
% The name of the fields of the structure have to be the ones shown here.

sys = struct('A', sysD.A, 'B', sysD.B, 'LBx', LBx, 'UBx', UBx, 'LBu', LBu, 'UBu', UBu);

%% STEP 2: Design the MPC controller.
% We need to determine the values of the ingredients of the MPC controller.

Q = blkdiag(15*eye(p), 1*eye(p)); % Remember that Q and R must be positive definite
R = 0.1*eye(m);
T = 10*Q; % In this formulation is it typically beneficial to make T larger than Q
S = R;
N = 10;

% The ingredients of the controller have to be saved into a structure.
% Once again, the name of the fields must be as shown.
param = struct('Q', Q, 'R', R, 'T', T, 'S', S, 'N', N);


%% STEP 3: Select options for the solver (optional).

% Set the options for the solver
options.rho_base = 2; % Base penalty parameter of the EADMM algorithm
options.rho_mult = 20; % Multiplication factor for the base penalty parameter
options.k_max = 5000; % Maximum number of iterations of the solver
options.tol = 1e-3; % Exit tolerance of the solver

% Notice that in this case there are no options for the toolbox. Only for the solver.

%% NOTICE: Steps 1 and 2 are exactly the same as for the sparse solver.
%          The selection of the options is also the same.
%          The only difference is that you do not have to generate the
%          solver, you only need to call it, as you will now see.

%% STEP 4: Use the solver.
% We now show how to use the solver. We will only show a single call, but
% its use for a closed-loop simulation follows similarly.

% We need to determine the current state of the system and the reference.
x = zeros(n, 1); % Current state of the system
ur = 0.5*ones(m, 1); % We select a reference ur
xr = (sys.A - eye(n))\(-sys.B*ur); % We compute a state reference xr that is a steady state

% We call the solver
[u, k, e_flag, Hist] = spcies_MPCT_EADMM_solver(x, xr, ur, 'sys', sys, 'param', param,...
                                                'options', options, 'genHist', 2);

% All the non-sparse solvers require the three inputs (x, xr, ur), the 'sys' structure
% and the 'param' structure. The 'options' structure is optional.
% They also accept other additional arguments. In particular, an important one to know 
% about is 'genHist'. This argument can take the values 0, 1 or 2. Its value determines
% the amount of information that is saved and returned inside the structure 'Hist'.
% Lower values of 'genHist' return less information, but may have shorter execution times.

% The outputs of the non-sparse solvers are always the same:
%   - u: Control input to apply to the system.
%   - k: Number of iterations of the algorithm.
%   - e_flag: Exit flag of the algorithm. Positive integers indicate success and negative
%             numbers indicate that some problem has occurred. For the exact flags and their
%             meaning we refer the reader to the help of each solver.
%   - Hist: structure containing information about the solution and iterates of the solver.
%           The extent of the information gathered is determined by the input 'genHist'.
%           For an exact list of its fields we refer the reader to the help of each solver.
