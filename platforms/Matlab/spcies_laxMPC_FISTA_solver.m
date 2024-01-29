%% spcies_laxMPC_FISTA_solver - Solver for the lax MPC formulation using FISTA
%
% This is a non-sparse solver of the FISTA-based lax MPC solver from the Spcies toolbox.
%
% Information about this formulation and the solver can be found at:
% 
% P. Krupa, D. Limon, T. Alamo, "Implementation of model predictive control in
% programmable logic controllers", Transactions on Control Systems Technology, 2020.
% 
% Specifically, this formulation is given in equation (9) of the above reference.
%
% [u, k, e_flag, Hist] = spcies_laxMPC_FISTA_solver(x0, xr, ur, 'name', value, 'name', ...) 
%
% INPUTS:
%   - x0: Current system state.
%   - xr: State reference.
%   - ur: Input reference.
%   - lambda: (optional) Initial value of the dual variables.
% 
% NAME-VALUE INPUTS (optional):
%   - sys: State space model of the system. It should either be an
%          instance of the ssModel class of the GepocToolbox or a
%          structure containing:
%          - .A: matrix A of the state space model.
%          - .B: matrix B of the state space model.
%          It can optionally also contain the following fields:
%          - .xOptPoint: operating point for the system state.
%          - .uOptPoint: operating point for the system input.
%          - .LBx: Lower bound for the system state.
%          - .UBx: Upper bound for the system state.
%          - .LBu: Upper bound for the system input.
%          - .UBu: Upper bound for the system input.
%          - .Nx: Vector defining the scaling of the system state.
%          - .Nu: Vector defining the scaling of the system input.
%   - param: Structure containing the ingredients of the MPC controller.
%          - .Q: Cost function matrix Q.
%          - .R: Cost function matrix R.
%          - .T: Cost function matrix T.
%          - .N: Prediction horizon.
%   - controller: Alternatively, the sys and param arguments can be omitted 
%                 and instead substituted by an instance of the LaxMPC
%                 class of the GepocToolbox (https://github.com/GepocUS/GepocToolbox).
%   - options: Structure containing options of the FISTA solver.
%              - .tol: Exit tolerance of the solver. Defaults to 1e-4.
%              - .k_max: Maximum number of iterations of the solver. Defaults to 1000.
%              - .in_engineering: Boolean that determines if the arguments of the solver are given in
%                                 engineering units (true) or incremental ones (false - default).
%   - verbose: Controlls the amount of information printed in the console.
%              Integer from 0 (no information printed) to 3 (print all information).
%   - genHist: Controlls the amount of information saved and returned in the output Hist.
%              Integer from 0 (save minimum amount of data) to 2 (save all data).
% 
% OUTPUTS:
%   - u: Control action to be applied to the system.
%   - k: Number of iterations of the algorithm.
%   - e_flag: Exit flag of the algorithm.
%       - 1: Optimal solution found.
%       - -1: Maximum iterations reaches. Returns last iterate.
%   - Hist: Structure containing information of the iterations and final outcome of the solver.
%       - sol: Structure containing the attained solution.
%           - .z: Primal variables
%           - .lambda: Dual variables.
%           - .res: Value of the infinity norm of the residual.
%       - hZ: Historic of z (only saved in genHist > 1).
%       - hLambda: Historic of lambda (only saved in genHist > 1).
%       - hRes: Historic of the infinity norm of the esidual (only saved in genHist > 0).
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function [u, k, e_flag, Hist] = spcies_laxMPC_FISTA_solver(x0, xr, ur, lambda, varargin)

    %% Default options
    def_sys = []; % Default value for the sys argument
    def_param = []; % Default value for the param argument
    def_controller = []; % Default value for the controller argument
    def_genHist = 0; % Default amount of data generated for Hist
    def_verbose = 1; % Default amount of information displayed
    def_options = laxMPC.def_options_laxMPC_FISTA(); % Default values of the options of the solver
    
    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'spcies_laxMPC_FISTA_solver';
    
    % Name-value parameters
    addOptional(par, 'lambda', [], @(x) (isnumeric(x) && (min(size(x))==1)) || isempty(x));
    addParameter(par, 'sys', def_sys, @(x) isa(x, 'ss') || isa(x, 'ssModel') || isstruct(x));
    addParameter(par, 'param', def_param, @(x) isstruct(x));
    addParameter(par, 'controller', def_controller, @(x) isa(x, 'ssMPC'));
    addParameter(par, 'options', def_options, @(x) isstruct(x));
    addParameter(par, 'genHist', def_genHist, @(x) isnumeric(x) && (x>=0));
    addParameter(par, 'verbose', def_verbose, @(x) isnumeric(x) && (x>=0));
    
    % Parse
    parse(par, lambda, varargin{:})
    
    % Set default options if options is empty
    if isempty(par.Results.options)
        options = def_options;
    else
        options = par.Results.options;
    end
    options = Spcies_options('formulation', 'laxMPC', 'method', 'FISTA', 'options', options);
    
    % Create the controller structure
    if isempty(par.Results.controller)
        controller.sys = par.Results.sys;
        controller.param = par.Results.param;
    else
        controller = par.Results.controller;
        if ~isa(controller, 'LaxMPC')
            error('Controller object must be of the LaxMPC class');
        end
    end
    
    % Rename and check other arguments
    genHist = par.Results.genHist;
    if genHist > 2; genHist = 2; end
    if genHist < 0; genHist = 0; end
    verbose = par.Results.verbose;
    if verbose > 3; verbose = 3; end
    if verbose < 0; verbose = 0; end
    
    %% Generate ingredients of the solver
    
    % Extract from controller
    if isa(controller, 'LaxMPC')
        A = controller.model.A;
        B = controller.model.Bu;
        n = controller.model.n_x;
        m = controller.model.n_u;
        N = controller.N;
        Q = controller.Q;
        R = controller.R;
        T = controller.P;
        LBx = controller.model.LBx;
        UBx = controller.model.UBx;
        LBu = controller.model.LBu;
        UBu = controller.model.UBu;
    else
        A = controller.sys.A;
        if isa(controller.sys, 'ssModel')
            B = controller.sys.Bu;
        else
            B = controller.sys.B;
        end
        n = size(A, 1);
        m = size(B, 2);
        N = controller.param.N;
        Q = controller.param.Q;
        R = controller.param.R;
        T = controller.param.T;
        LBx = controller.sys.LBx;
        UBx = controller.sys.UBx;
        LBu = controller.sys.LBu;
        UBu = controller.sys.UBu;
    end
        
    % Get lambda
    lambda = par.Results.lambda;
    if isempty(lambda)
        lambda = zeros(N*n, 1);
    end
    
    % Compute Hessian
    H = blkdiag(R, kron(eye(N-1), blkdiag(Q, R)), T);
    
    % Compute the matrix Aeq (equality constraints)
    
    % Compute the top part of matrix Az, which corresponds to the prediction model constraints
    Aeq = kron(eye(N-1), [A B]); % Diagonal of the matrix
    j = 0;
    for i=1:n:n*N-n % Insert matrices -I in Az
        j = j+1;
        Aeq(i:i+n-1,((j-1)*(n+m)+(m+n+1)):((j-1)*(n+m)+(n+m+1)+n-1)) = -eye(n);
    end
    Aeq = [B -eye(n) zeros(n, size(Aeq, 2) - n); zeros(size(Aeq, 1), m) Aeq]; % Initial condition
    
    % Compute matrix W
    Hinv = inv(H);
    W = Aeq*Hinv*Aeq';
    
    % Compute the constraints
    if min(size(LBx)) == 1
        LB = [LBu; kron(ones(N-1,1), [LBx; LBu]); LBx]; 
        UB = [UBu; kron(ones(N-1,1), [UBx; UBu]); UBx];
    else
        if size(LBx, 2) ~= N+1 || size(LBu, 2) ~= N+1 || size(UBx, 2) ~= N+1 || size(UBu, 2) ~= N+1
            error('Matrices LBx, UBx, LBu and UBu must have N+1 columns');
        end
        LB = [LBu(:, 1); reshape([LBx(:, 2:N); LBu(:, 2:N)], (N-1)*(n+m), 1); LBx(:, N+1)];
        UB = [UBu(:, 1); reshape([UBx(:, 2:N); UBu(:, 2:N)], (N-1)*(n+m), 1); UBx(:, N+1)];
    end
   
    % Scaling and operation point
    if isa(controller, 'LaxMPC')
        scaling_x = controller.model.Nx;
        scaling_u = controller.model.Nu;
        OpPoint_x = controller.model.x0;
        OpPoint_u = controller.model.u0;
    else
        if isfield(controller.sys, 'Nx')
            scaling_x = controller.sys.Nx;
        else
            scaling_x = ones(n, 1);
        end
        if isfield(controller.sys, 'Nu')
            scaling_u = controller.sys.Nu;
        else
            scaling_u = ones(m, 1);
        end
        if isfield(controller.sys, 'x0')
            OpPoint_x = controller.sys.x0;
        else
            OpPoint_x = zeros(n, 1);
        end
        if isfield(controller.sys, 'u0')
            OpPoint_u = controller.sys.u0;
        else
            OpPoint_u = zeros(m, 1);
        end
    end
    
    % Compute variables used by the solver
    Hinv_diag = diag(Hinv);
    %Hinv = sparse(Hinv);
    Aeq = sparse(Aeq);
    W = sparse(W);
    
    %% Algorithm
    
    % Initialize
    done = false;
    e_flag = 0; 
    k = 0;
    z_k = zeros(N*(n+m), 1);
    y_k = zeros(N*n, 1);
    lambda_k = zeros(N*n, 1);
    lambda_km1 = lambda_k;
    t_k = 1;
    t_km1 = 1;
    
    % Historics
    if genHist > 0
        hRes = zeros(1, options.solver.k_max+1);
    end
    if genHist > 1
        hZ = zeros(N*(n+m), options.solver.k_max+1);
        hLambda = zeros(N*n, options.solver.k_max+1);
    end
    
    % Obtain x0, xr and ur
    if options.in_engineering
        x0 = scaling_x*(x0 - OpPoint_x);
        xr = scaling_x*(xr - OpPoint_x);
        ur = scaling_u*(ur - OpPoint_u);
    end
    
    % Update b
    b = zeros(N*n, 1);
    b(1:n) = -A*x0;
    
    % Update q
    q = -[R*ur; kron(ones(N-1, 1), [Q*xr; R*ur]); T*xr];
    
    % Compute q_0
    q_k = q - Aeq'*lambda;
    
    % Compute z_0
    z_k = solve_boxQP(q_k, Hinv_diag, LB, UB);
    
    % Compute r_0
    r_k = -Aeq*z_k + b;
    res_k = norm(r_k, Inf);
    
    % Compute delta_lambda_0
    d_lambda_k = W\r_k;
    
    % Compute y_0
    y_k = lambda + d_lambda_k;
    
    % Compute lambda_0
    lambda_k = y_k;
    
    % Save historics
    if genHist > 0
        hRes(k+1) = res_k;
    end
    if genHist > 1
        hZ(:, k+1) = z_k;
        hLambda(:, k+1) = lambda_k;
    end
    
    while ~done
        k = k + 1;
        
        % Update values of the previous iteration
        t_km1 = t_k;
        lambda_km1 = lambda_k;

        % Uptade q_k
        q_k = q - Aeq'*y_k;
        
        % Update z_k
        z_k = solve_boxQP(q_k, Hinv_diag, LB, UB);
        
        % Compute r_K
        r_k = -Aeq*z_k + b;
        res_k = norm(r_k, Inf);
        
        % Check exit condition
        if res_k <= options.solver.tol
            done = true;
            e_flag = 1;
        elseif k >= options.solver.k_max
            done = true;
            e_flag = -1;
        end
        
        if done == 0
        
            % Compute delta_lambda_K
            d_lambda_k = W\r_k; 

            % Update lambda_k
            lambda_k = d_lambda_k + y_k;

            % Update t_k
            t_k = 0.5*( 1 + sqrt( 1 + 4*t_km1^2));

            % Update y_k
            y_k = lambda_k + ((t_km1 - 1)/t_k)*(lambda_k - lambda_km1);
        
        end
        
        % Save Historics
        if genHist > 0
            hRes(k+1) = res_k;
        end
        if genHist > 1
            hZ(:, k+1) = z_k;
            hLambda(:, k+1) = lambda_k;
        end
        
    end
    
    %% Return results
    
    % Control action
    if options.in_engineering
        u = z_k(1:m)./scaling_u + OpPoint_u;
    else
        u = z_k(1:m);
    end
    
    % Hist
        % Solution
    Hist.sol.z = z_k;
    Hist.sol.lambda = y_k;
    Hist.sol.res = res_k;
    Hist.k = k;
        % Historics
    if genHist > 0
        Hist.hRes = hRes(1:k+1);
    end
    if genHist > 1
        Hist.hZ = hZ(:, 1:k+1);
        Hist.hLambda = hLambda(:, 1:k+1);
    end

end
