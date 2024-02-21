%% Tutorial: using the time-varying MPC formulations
%
% Some MPC formulations and solvers provide a time-varying mode in which the
% main ingredients of the MPC formulation can be updated online at a small
% computational cost thanks to tailored algorithms for recomputing the main
% computationally expensive solver ingredients.
%
% Using these solvers, you can update the prediction model of the MPC controller,
% its cost function matrices, or the constraints of the system.
% Again, enabling this feature leads to a solver with slightly larger computation
% times (the time required to update the solver ingredients), so you should only
% enable it if you are going to make use of this feature.
% However, we find that the update times are very small compared with the solve
% times, so enabling this feature should not be a problem in most cases.
%
% We note that not all combinations of MPC formulations and solvers provide a
% time-varying alternative.
% Currently, only the 'laxMPC' and 'equMPC' solvers have this feature.
%
% This tutorial shows you how to enable this feature and how to call the generated
% solvers from Matlab.
clear; clc

%% STEP 1: Construct the system
[sys, param] = sp_utils.example_OscMass();
n = size(sys.A, 1); % For convenience, we save the size of the state dimension
m = size(sys.B, 2); % and of the input dimension

%% STEP 2: Generate the solver for its use in Matlab
% The important option in this tutorial is the option 'time_varying', which, if
% true, indicates that a "time-varying" solver should be generated.
% By default this option is set to 'false', so don't forget to set it to 'true' if you
% what to be able to change the MPC ingredients online.

% Select the solver options
options.rho = 15; % Value of the penalty parameter of the ADMM algorithm
options.k_max = 5000; % Maximum number of iterations of the solver
options.tol = 1e-4; % Exit tolerance of the solver
options.debug = true; % To get additional information from the solver
options.time_varying = true; % To select the time-varying solver

options.save_name = 'myMPCsolver';
options.timing = true; % To measure computation times
options.formulation = 'equMPC'; % The 'laxMPC' formulation also has time-varying solvers. Check it out!
% In this tutorial we use the ADMM solver, as selected below, but a time-varying version
% of the FISTA solver is also available.
options.method = 'ADMM'; % Try the FISTA version by changing this to 'FISTA'

if strcmp(options.method, 'FISTA')
    param.T = diag(diag(param.T)); % We include this because T must be diagonal in the FISTA solver
end

spcies('clear');
spcies('gen', 'sys', sys, 'param', param, ...
       'options', options, 'platform', 'Matlab');

%% STEP 3: Closed-loop simulation
num_iter = 60; % Number of sample times
x0 = zeros(n, 1); % Initial condition
ur = 0.5*ones(m, 1); % We select a reference ur
xr = (sys.A - eye(n))\(-sys.B*ur); % We compute a state reference xr that is a steady state

hX = zeros(n, num_iter); % Evolution of the state
hU = zeros(m, num_iter); % Control actions applied
hT_update = zeros(1, num_iter); % Computation time required to update the solver ingredients
hT_run = zeros(1, num_iter); % Total computation time used by the solver
hK = zeros(1, num_iter); % Number of iterations of the solver
hE = zeros(1, num_iter); % Exit flag of the solver at each iteration

% To show that the update procedure is working, let us modify matrix A of the prediction model
% to an incorrect value of the system's A matrix.
% Using this matrix in the MPC solver will result in it not being able to track the reference 
% due to the discrepancy between the prediction model and the real system.
A_pred = sys.A;
A_pred(1, 1) = sys.A(1, 1) + 0.1;

x = x0; % Set the state to the initial state
for i = 1:num_iter

    % Call the time-varying solver
    % Note that we must pass the following arguments:
    %   - x, xr, ur: same as the time-invariant case
    %   - A, B: Matrices of the prediction model
    %   - Q, R: Cost-function matrices of the MPC formulation
    %   - LB, UB: upper and lower bounds of the system constraints, LB = [LBx; LBu], UB = [UBx; UBu]
    % Note that we use the 'A_pred' variable defined above
    [u, hK(i), hE(i), info] = myMPCsolver(x, xr, ur, A_pred, sys.B, diag(param.Q), diag(param.R), [sys.LBx; sys.LBu], [sys.UBx; sys.UBu]);
    hT_update(i) = info.update_time; % This time includes the time required to update the solver ingredients
    hT_run(i) = info.run_time;

    % After 30 iterations we change A_pred so that it is now correct.
    % As you can see in the results, this results in the MPC controller steering the system to the reference
    if i == 30
        A_pred = sys.A;
    end

    % Simulate the system
    x = sys.A*x + sys.B*u;

    % Save the values of x and u
    hX(:, i) = x;
    hU(:, i) = u;
    
end

%% STEP 4: Plot the results
figure(1); clf(1);

% As you can see, the update time is very small. In fact, when using ADMM it is negligible compared with the total
% computation time. When using FISTA it is a larger percentage because FISTA has a very small solve time in this example.

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
bar(0:num_iter-1, hT_run); % Plot the total computation times
hold on;
bar(0:num_iter-1, hT_update); % Plot the update times
xlabel('Sample time');
ylabel('Computation time [ms]');
legend({"Total time", "Update time"});
grid on;

% Number of iterations
subplot(2, 2, 4);
bar(0:num_iter-1, hK);
xlabel('Sample time');
ylabel('Number of iterations');
grid on;

