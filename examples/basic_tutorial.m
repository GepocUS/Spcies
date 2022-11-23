%% Welcome to SPCIES: Suite of Predictive Controllers for Industrial Embedded systems.
% 
% This is a basic introductory tutorial to the main use of SPCIES: the generation
% of tailored solvers for different MPC formulations.
% 
% The toolbox can create solvers in various programming languages.
% This tutorial will guide you through the creation of a MEX function for Matlab
% for a standard MPC formulation that we label 'laxMPC', which is given by:
% 
% min_{x, u} sum_{i = 0}^{N-1} \| x_i - xr \|^2_Q + \| u_i - ur \|^2_R + \| x_N - xr \|^2_T
%
%   s.t. x_0 = x(k)
%        x_{i+1} = A x_i + B u_i, i = 0...N-1
%        LBx <= x_i <= UBx, i = 1...N-1
%        LBu <= u_i <= UBu, i = 0...N-1
% 
% where x_i and u_i are the predicted states and control actions, respectively;
% x_r and u_r are the state and input reference, respectively; Q, R and T are
% the positive definite cost function matrices; LBx, LBu are the lower bounds
% for the state and input, respectively; UBx, UBu the upper bounds; and
% N is the prediction horizon.
% 
% For additional information about the formulation, we refer the user to:
% 
% P. Krupa, D. Limon, T. Alamo, "Implementation of model predictive control in
% programmable logic controllers", Transactions on Control Systems Technology, 2020.
% 
% Specifically, this formulation is given in equation (9) of the above reference.
% 
clear; clc;

%% STEP 1: Problem definition.
% Let us consider a system composed of three masses connected by springs.
% The continuous state space model of the system is generated by the 
% function gen_oscillating_masses, located in /+utils.
% First, we must define the system: we consider three objects, where external
% force can be applied to the two outer-most ones.

p = 3; % Number of objects
M = [1; 0.5; 1]; % Mass of each object
K = 2*ones(p+1, 1); % Spring constant of each spring
F = [1; zeros(p-2, 1); 1]; % Objects in which an external force is applied

sysC = utils.gen_oscillating_masses(M, K, F); % Generate the continuous-time model

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
[~, T] = dlqr(sys.A, sys.B, Q, R); % This is the typical choice of T in MPC
N = 10;

% The ingredients of the controller have to be saved into a structure.
% Once again, the name of the fields must be as shown.
param = struct('Q', Q, 'R', R, 'T', T, 'N', N);

%% STEP 3: Generate the solver.
% In this step, we will call the main function of SPCIES to generate a 
% MEX file containing the sparse solver.
% In this tutorial we will create an ADMM-based solver for the laxMPC formulation.

% Before we generate the solver, we must sets its options. This is not strictly
% necessary, since default values are provided, but it is recommended. Especially
% for the penalty parameter 'rho' of ADMM, since performance is highly dependent
% on its value.
% To set the options we must create a structure with the appropriate fields.
% We will set some options. For a full list of the available options, please
% consult the laxMPC documentation.

solver_options.rho = 15; % Value of the penalty parameter of the ADMM algorithm
solver_options.k_max = 5000; % Maximum number of iterations of the solver
solver_options.tol = 1e-3; % Exit tolerance of the solver
solver_options.time_varying = false;

% Next, we can set some of the options of the toolbox, such as the name of the
% MEX file that will be generated, or the directory where the solver is saved.
options.save_name = 'lax_solver';
options.directory = '';


% If an empty field is provided (as in options.directory = ''), then
% it is given its default value.
% The default directory where the solver is saved is in the folder
% $SPCIES$/generated_solvers, where $SPCIES$ is the root directory of the
% toolbox (the function spcies_get_root_directory.m returns this directory)
% To save the files into the current working directory use options.directory = './';

% Since we are saving the solver into the default directory $SPCIES$/generated_solvers,
% we recommend that you clear the directory of all previous controllers.
% Controllers are overwritten by default, but it is best to be safe that sorry.
% You can clear $SPCIES$/generated_solvers by running:
spcies_clear;

% We now generate the mex file by calling the following function:
spcies_gen_controller('sys', sys, 'param', param, 'solver_options', solver_options,...
    'options', options, 'platform', 'Matlab', 'type', 'laxMPC');

% Notice that we are indicating the 'platform' as 'Matlab', which is telling
% the function to generate a mex file. This was not strictly needed, since
% the default value for the 'platform' is set to Matlab.
% For plain C code, set the 'platform' argument to 'C'.
% Another way of passing the target to the function would have been to include
% it in the options structure in the field 'platform'.

% Notice also that we are telling the function that the 'type' of controller 
% I want it to generate a solver for is the laxMPC formulation.
% For a full list of supported MPC formulations and solvers for them please
% refer to the documentation of the toolbox and the other examples.

% A MEX file should have been saved into $SPCIES$/generated_solvers if a Matlab
% compatible compiler is correctly installed and interfaced with Matlab.
% If not, an error should have occurred at this point.
% If using the Linux version of Matlab, SPCIES currently assumes that the
% gcc compiler is being used.

%% STEP 4: Use the MEX file.
% To show how to use the generated MEX file, we will perform a closed-loop test.

% First, we set the conditions of the test
num_iter = 50; % Number of sample times
x0 = zeros(n, 1); % Initial condition

ur = 0.5*ones(m, 1); % We select a reference ur
xr = (sys.A - eye(n))\(-sys.B*ur); % We compute a state reference xr that is a steady state

% Let us define variables to store the results of the test
hX = zeros(n, num_iter); % Evolution of the state
hU = zeros(m, num_iter); % Control actions applied
hT = zeros(1, num_iter); % Computation time of the solver
hK = zeros(1, num_iter); % Number of iterations of the solver
hE = zeros(1, num_iter); % Exit flag of the solver at each iteration

% We now start the simulation
x = x0; % Set the state to the initial state

for i = 1:num_iter
    
    % Call the laxMPC solver.
    % The solver requires the current state 'x', and the reference
    % 'xr' and 'ur'. It returns the control action to be applied
    % to the system 'u', the number of iterations and an exit
    % flag (1: solution found, -1: maximum iterations).
    % The name of the function must match the string in 'save_name'.
    tic;
    [u, hK(i), hE(i)] = lax_solver(x, xr, ur);
    hT(i) = toc;
    
    % Simulate the system
    x = sys.A*x + sys.B*u;
    
    % Save the values of x and u
    hX(:, i) = x;
    hU(:, i) = u;
    
end

% We can now plot the results
figure(1); clf(1);

% State
subplot(2, 2, 1);
object_num = 2; % Determine which state to plot
plot(0:num_iter, [x0(object_num) hX(object_num,:)], 'b');
hold on;
plot(0:num_iter, xr(object_num)*ones(1, num_iter+1), 'r:'); % Plot the reference
xlabel('Sample time');
ylabel(['Position of object ' num2str(object_num)]);
grid on;

% Input
subplot(2, 2, 2);
force_num = 1; % Determine which input to plot
plot(0:num_iter-1, hU(force_num, :), 'b');
hold on;
plot(0:num_iter-1, ur(force_num)*ones(1, num_iter), 'r:'); % Plot the reference
xlabel('Sample time');
ylabel(['Control input #' num2str(force_num)]);
grid on;

% Computation time
subplot(2, 2, 3);
bar(0:num_iter-1, hT*1000);
xlabel('Sample time');
ylabel('Computation time [ms]');
grid on;

% Number of iterations
subplot(2, 2, 4);
bar(0:num_iter-1, hK);
xlabel('Sample time');
ylabel('Number of iterations');
grid on;

