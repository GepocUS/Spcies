%% spcies_equMPC_FISTA_solver - Solver for the equaluty MPC formulation using FISTA
%
% This is a non-sparse solver of the FISTA-based equality MPC solver from the Spcies toolbox.
%
% Information about this formulation and the solver can be found at:
% 
% P. Krupa, D. Limon, T. Alamo, "Implementation of model predictive control in
% programmable logic controllers", Transactions on Control Systems Technology, 2020.
% 
% Specifically, this formulation is given in equation (8) of the above reference.
%
% [u, k, e_flag, sol] = spcies_equMPC_FISTA_solver(x0, xr, ur, 'name', value, 'name', ...) 
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
%          - .nU: Vector defining the scaling of the system input.
%   - param: Structure containing the ingredients of the MPC controller.
%          - .Q: Cost function matrix Q.
%          - .R: Cost function matrix R.
%          - .N: Prediction horizon.
%   - controller: Alternatively, the sys and param arguments can be omitted 
%                 and instead substituted by an instance of the EqualityMPC
%                 class of the GepocToolbox (https://github.com/GepocUS/GepocToolbox).
%   - options: Structure containing options of the FISTA solver.
%              - .tol: Exit tolerance of the solver. Defaults to 1e-4.
%              - .k_max: Maximum number of iterations of the solver. Defaults to 1000.
%              - .in_engineering: Boolean that determines if the arguments of the solver are given in
%                                 engineering units (true) or incremental ones (false - default).
% 
% OUTPUTS:
%   - u: Control action to be applied to the system.
%   - k: Number of iterations of the algorithm.
%   - e_flag: Exit flag of the algorithm.
%       - 1: Optimal solution found.
%       - -1: Maximum iterations reaches. Returns last iterate.
%   - sol: Structure containing the optimal solution of the decision variables and dual variables.
%       - .z: Optimal decision variables z1.
%       - .lambda: Optimal dual variables.
%       - .res: Value of the residual.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function [u, k, e_flag, sol] = spcies_equMPC_FISTA_solver(x0, xr, ur, lambda, varargin)

    %% Default options
    def_sys = []; % Default value for the sys argument
    def_param = []; % Default value for the param argument
    def_controller = []; % Default value for the controller argument
    
    % Default options
    def_options.tol = 1e-4;
    def_options.k_max = 1000;
    def_options.in_engineering = false;
    
    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'spcies_equMPC_FISTA_solver';
    
    % Name-value parameters
    addOptional(par, 'lambda', [], @(x) (isnumeric(x) && (min(size(x))==1)) || isempty(x));
    addParameter(par, 'sys', def_sys, @(x) isa(x, 'ss') || isa(x, 'ssModel') || isstruct(x));
    addParameter(par, 'param', def_param, @(x) isstruct(x));
    addParameter(par, 'controller', def_controller, @(x) isa(x, 'ssMPC'));
    addParameter(par, 'options', def_options, @(x) isstruct(x));
    
    % Parse
    parse(par, lambda, varargin{:})
    
    % Set default options if options is empty
    if isempty(par.Results.options)
        options = def_options;
    else
        options = par.Results.options;
    end
    
    % Add default values
    options = utils.add_default_options_to_struct(options, def_options);
    
    % Create the controller structure
    if isempty(par.Results.controller)
        controller.sys = par.Results.sys;
        controller.param = par.Results.param;
    else
        controller = par.Results.controller;
        if ~isa(controller, 'EqualityMPC')
            error('Controller object must be of the EqualityMPC class');
        end
    end
    
    %% Generate ingredients of the solver
    
    % Extract from controller
    if isa(controller, 'EqualityMPC')
        A = controller.model.A;
        B = controller.model.Bu;
        n = controller.model.n_x;
        m = controller.model.n_u;
        N = controller.N;
        Q = controller.Q;
        R = controller.R;
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
    H = blkdiag(R, kron(eye(N-1), blkdiag(Q, R)));
    
    % Compute the top part of matrix Az, which corresponds to the prediction model constraints
    Aeq = kron(eye(N-1), [A B]); % Diagonal of the matrix
    j = 0;
    for i=1:n:n*N-n % Insert matrices -I in Az
        j = j+1;
        Aeq(i:i+n-1,((j-1)*(n+m)+(m+n+1)):((j-1)*(n+m)+(n+m+1)+n-1)) = -eye(n);
    end
    Aeq = [B -eye(n) zeros(n, size(Aeq, 2) - n); zeros(size(Aeq, 1), m) Aeq]; % Initial condition
    Aeq = Aeq(:,1:end-n);
    
    % Compute matrix W
    Hinv = inv(H);
    W = Aeq*Hinv*Aeq';
    
    % Compute the constraints
    LB = [LBu; kron(ones(N-1,1), [LBx; LBu])]; 
    UB = [UBu; kron(ones(N-1,1), [UBx; UBu])];
    
    % Scaling and operation point
    if isa(controller, 'LaxMPC')
        scaling_x = controller.model.Nx;
        scaling_u = controller.model.Nu;
        OpPoint_x = controller.model.x0;
        OpPoint_u = controllr.model.u0;
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
    z_k = zeros((N-1)*(n+m)+m, 1);
    y_k = zeros(N*n, 1);
    lambda_k = zeros(N*n, 1);
    lambda_km1 = lambda_k;
    t_k = 1;
    t_km1 = 1;
    
    % Obtain x0, xr and ur
    if options.in_engineering
        x0 = scaling_x*(x0 - OpPoint_x);
        xr = scaling_x*(xr - OpPoint_x);
        ur = scaling_u*(ur - OpPoint_u);
    end
    
    % Update b
    b = zeros(N*n, 1);
    b(1:n) = -A*x0;
    b(end-n+1:end) = xr;
    
    % Update q
    q = -[R*ur; kron(ones(N-1, 1), [Q*xr; R*ur])];
    
    % Compute q_0
    q_k = q - Aeq'*lambda;
    
    % Compute z_0
    z_k = solve_boxQP(q_k, Hinv_diag, LB, UB);
    
    % Compute r_0
    r_k = -Aeq*z_k + b;
    
    % Compute delta_lambda_0
    d_lambda_k = W\r_k; 
    
    % Compute y_0
    y_k = lambda + d_lambda_k;
    
    % Compute lambda_0
    lambda_k = y_k;
    
    while~done
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
        
        % Check exit condition
        if norm(r_k, Inf) <= options.tol
            done = true;
            e_flag = 1;
        elseif k >= options.k_max
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
        
    end
    
    %% Return results
    
    % Control action
    if options.in_engineering
        u = z_k(1:m)./var.scaling_u + var.OpPoint_u;
    else
        u = z_k(1:m);
    end
    
    % Optimal decision variables
    sol.z = z_k;
    sol.lambda = y_k;
    sol.res = r_k;

end

