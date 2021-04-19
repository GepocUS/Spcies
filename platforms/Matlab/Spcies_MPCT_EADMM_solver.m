%% Spcies_MPCT_EADMM_solver - Solver for the MPCT formulation using the EADMM algorithm
%
% This is a non-sparse solver of the MPC for Tracking formulation from the Spcies toolbox.
%
% Information about this formulation and the solver can be found at:
% 
% "Implementation of model predictive control for tracking in embedded systems
% using a sparse extended ADMM algorithm", by P. Krupa, I. Alvarado, D. Limon
% and T. Alamo, arXiv preprint: 2008:09071v2, 2020.
%
% [u, k, e_flag, sol] = Spcies_MPCT_EADMM_solver(x0, xr, ur, 'name', value, 'name', ...) 
%
% INPUTS:
%   - x0: Current system state
%   - xr: State reference
%   - ur: Input reference
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
%   - param: Structure containing the ingredients of the MPCT controller.
%          - .Q: Cost function matrix Q.
%          - .R: Cost function matrix R.
%          - .T: Cost function matrix T.
%          - .S: Cost function matrix S.
%          - .N: Prediction horizon.
%   - controller: Alternatively, the sys and param arguments can be omitted 
%                 and instead substituted by an instance of the TrackingMPC
%                 class of the GepocToolbox (https://github.com/GepocUS/GepocToolbox).
%   - options: Structure containing options of the EADMM solver.
%              - .rho_base: Scalar. Base value of the penalty parameter.
%              - .rho_mult: Scalar. Multiplication factor of the base value.
%              - .epsilon_x: Vector by which the bound for x_s are reduced.
%              - .epsilon_u: Vector by which the bound for u_s are reduced.
%              - .inf_bound: Scalar. Determines the value given to components without bound.
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
%       - .z1: Optimal decision variables z1.
%       - .z2: Optimal decision variables z2.
%       - .z3: Optimal decision variables z3.
%       - .lambda: Optimal dual variables.
%       - .res: Value of the residual.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function [u, k, e_flag, sol] = Spcies_MPCT_EADMM_solver(x0, xr, ur, varargin)
    import MPCT.Spcies_compute_MPCT_EADMM_ingredients;
    
    %% Default values
    def_sys = []; % Default value for the sys argument
    def_param = []; % Default value for the param argument
    def_controller = []; % Default value for the controller argument
    def_rho_base = 3;
    def_rho_mult = 20;
    def_epsilon_x = 1e-6;
    def_epsilon_u = 1e-6;
    def_inf_bound = 1e6;
    def_tol = 1e-4;
    def_k_max = 1000;
    def_in_engineering = false;
    def_options = struct('rho_base', def_rho_base, 'rho_mult', def_rho_mult, 'epsilon_x', def_epsilon_x,...
                         'epsilon_u', def_epsilon_u, 'tol', def_tol, 'k_max', def_k_max,...
                         'in_engineering', def_in_engineering, 'inf_bound', def_inf_bound);
    
    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'Spcies_gen_controller';
    
    % Name-value parameters
    addParameter(par, 'sys', def_sys, @(x) isa(x, 'ss') || isa(x, 'ssModel') || isstruct(x));
    addParameter(par, 'param', def_param, @(x) isstruct(x));
    addParameter(par, 'controller', def_controller, @(x) isa(x, 'ssMPC'));
    addParameter(par, 'options', def_options, @(x) isstruct(x));
    
    % Parse
    parse(par, varargin{:})
    
    % Set default options if options is empty
    if isempty(par.Results.options)
        options = def_options;
    else
        options = par.Results.options;
    end
    if ~isfield(options, 'rho_base'); options.rho_base = def_rho_base; end
    if ~isfield(options, 'rho_mult'); options.rho_mult = def_rho_mult; end
    if ~isfield(options, 'epsilon_x'); options.epsilon_x = def_epsilon_x; end
    if ~isfield(options, 'epsilon_u'); options.epsilon_u = def_epsilon_u; end
    if ~isfield(options, 'inf_bound'); options.inf_bound = def_inf_bound; end
    if ~isfield(options, 'tol'); options.tol = def_tol; end
    if ~isfield(options, 'k_max'); options.k_max = def_k_max; end
    if ~isfield(options, 'in_engineering'); options.in_engineering = def_in_engineering; end
    
    % Create the controller structure
    if isempty(par.Results.controller)
        controller.sys = par.Results.sys;
        controller.param = par.Results.param;
    else
        controller = par.Results.controller;
        if ~isa(controller, 'TrackingMPC')
            error('Controller object must be of the TrackingMPC class');
        end
    end
    
    %% Generate ingredients of the solver
    [~, var] = Spcies_compute_MPCT_EADMM_ingredients(controller, options, []);
    N = var.N;
    n = var.n;
    m = var.m;
    
    %% Algorithm

    % Initialize
    done = false;
    k = 0;
    z1 = zeros((N+1)*(n+m), 1);
    z2 = zeros(m+n, 1);
    z3 = zeros((N+1)*(n+m), 1);
    lambda = zeros((N+2)*(n+m) + n, 1);
    
    % Obtain x0, xr and ur
    if options.in_engineering
        x0 = var.scaling_x*(x0 - var.OpPoint_x);
        xr = var.scaling_x*(xr - var.OpPoint_x);
        ur = var.scaling_u*(ur - var.OpPoint_u);
    end
    
    % Update b
    b = var.b;
    b(1:n) = x0;
    
    while ~done
        k = k + 1;
        
        %% Problem 1: Minimize w.r.t. z1 = (xi, ui)
        
        % Compute q1
        q1 = (var.rho.*var.A1)'*var.A2*z2 + (var.rho.*var.A1)'*var.A3*z3 + var.A1'*lambda - (var.rho.*var.A1)'*b;
        
        % Compute z1
        z1 = max( min( -q1.*var.H1i, var.UB), var.LB);
        
        %% Problem 2: Minimize w.r.t. z2 = (xs, us)
        
        % Compute q2
        q2 = -[var.T*xr; var.S*ur] + (var.rho.*var.A2)'*var.A1*z1 + (var.rho.*var.A2)'*var.A3*z3 + var.A2'*lambda;
        
        % Compute z2
        z2 = var.W2*q2;
        
        %% Problem 3: Minimize w.r.t. z3 = (hat_xi, hat_ui)
        
        % Compute q3
        q3 = (var.rho.*var.A3)'*var.A1*z1 + (var.rho.*var.A3)'*var.A2*z2 + var.A3'*lambda;
        
        % Compute mu
        mu = var.W3\(-var.Az3*var.H3inv*q3);
        
        % Compute z3
        z3 = -var.H3inv*(var.Az3'*mu + q3);
        
        %% Compute residual
        res = var.A1*z1 + var.A2*z2 + var.A3*z3 - b;
        
        %% Update lambda
        lambda = lambda + var.rho.*res;
        
        %% Exit condition
        if norm(res, Inf) <= options.tol
            done = true;
            e_flag = 1;
        elseif k >= options.k_max
            done = true;
            e_flag = -1;
        end
        
    end
    
    %% Return results
    
    % Control action
    if options.in_engineering
        u = z1(n+(1:m))./var.scaling_u + var.OpPoint_u;
    else
        u = z1(n+(1:m));
    end
    
    % Optimal decision variables
    sol.z1 = z1;
    sol.z2 = z2;
    sol.z3 = z3;
    sol.lambda = lambda;
    sol.res = res;
        
end

