%% spcies_MPCT_EADMM_solver - Solver for the MPCT formulation using the EADMM algorithm
%
% This is a non-sparse solver of the MPC for Tracking formulation from the Spcies toolbox.
%
% Information about this formulation and the solver can be found at:
% 
% "Implementation of model predictive control for tracking in embedded systems
% using a sparse extended ADMM algorithm", by P. Krupa, I. Alvarado, D. Limon
% and T. Alamo, arXiv preprint: 2008:09071v2, 2020.
%
% [u, k, e_flag, Hist] = spcies_MPCT_EADMM_solver(x0, xr, ur, 'name', value, 'name', ...) 
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
%       - sol: Structure containing the returned solution of the primal and dual variables.
%           - .z1: Returned decision variables z1.
%           - .z2: Returned decision variables z2.
%           - .z3: Returned decision variables z3.
%           - .lambda: Returned dual variables.
%       - res: Structure containing information of the values of the residuals of the returned solution.
%       - hNRes_pf: Historic of the infinity norm of the residual of primal feasibility (only saved in genHist > 0).
%       - hNRes_z2: Historic of the infinity norm of the residual of fixed point for z2 (only saved in genHist > 0).
%       - hNRes_z3: Historic of the infinity norm of the residual of fixed point for z3 (only saved in genHist > 0).
%       - hZ1: Historic of z1 (only saved in genHist > 1).
%       - hZ2: Historic of z2 (only saved in genHist > 1).
%       - hZ3: Historic of z3 (only saved in genHist > 1).
%       - hLambda: Historic of lambda (only saved in genHist > 1).
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function [u, k, e_flag, Hist] = spcies_MPCT_EADMM_solver(x0, xr, ur, varargin)
    import MPCT.compute_MPCT_EADMM_ingredients
    
    %% Default values
    def_sys = []; % Default value for the sys argument
    def_param = []; % Default value for the param argument
    def_controller = []; % Default value for the controller argument
    def_genHist = 0; % Default amount of data generated for Hist
    def_verbose = 1; % Default amount of information displayed
    def_options = MPCT.def_options_MPCT_EADMM(); % Default values of the options of the solver
    
    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'spcies_MPCT_EADMM_solver';
    
    % Name-value parameters
    addParameter(par, 'sys', def_sys, @(x) isa(x, 'ss') || isa(x, 'ssModel') || isstruct(x));
    addParameter(par, 'param', def_param, @(x) isstruct(x));
    addParameter(par, 'controller', def_controller, @(x) isa(x, 'ssMPC'));
    addParameter(par, 'options', def_options, @(x) isstruct(x));
    addParameter(par, 'genHist', def_genHist, @(x) isnumeric(x) && (x>=0));
    addParameter(par, 'verbose', def_verbose, @(x) isnumeric(x) && (x>=0));
    
    % Parse
    parse(par, varargin{:})
    
    % Set default options if options is empty
    if isempty(par.Results.options)
        options = def_options;
    else
        options = par.Results.options;
    end
    options = Spcies_options('formulation', 'MPCT', 'method', 'EADMM', 'options', options);

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
    
    % Rename and check other arguments
    genHist = par.Results.genHist;
    if genHist > 2; genHist = 2; end
    if genHist < 0; genHist = 0; end
    verbose = par.Results.verbose;
    if verbose > 3; verbose = 3; end
    if verbose < 0; verbose = 0; end
    
    %% Generate ingredients of the solver
    [~, var] = compute_MPCT_EADMM_ingredients(controller, options);
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
    z2_prev = z2;
    z3_prev = z3;
    lambda = zeros((N+2)*(n+m) + n, 1);
    
    % Historics
    if genHist > 0
        hNRes_pf = zeros(1, options.solver.k_max);
        hNRes_z2 = zeros(1, options.solver.k_max);
        hNRes_z3 = zeros(1, options.solver.k_max);
    end
    if genHist > 1
        hZ1 = zeros((N+1)*(n+m), options.solver.k_max);
        hZ2 = zeros(m+n, options.solver.k_max);
        hZ3 = zeros((N+1)*(n+m), options.solver.k_max);
        hLambda = zeros((N+2)*(n+m) + n, options.solver.k_max);
    end
    
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
        
        %% Compute residuals
        res_pf = var.A1*z1 + var.A2*z2 + var.A3*z3 - b; % Residual of primal feasibility
        res_z2 = z2 - z2_prev; % Residual of fixed point of z2
        res_z3 = z3 - z3_prev; % Residual of fixed point of z3
        % Infinity norms of the residuals
        n_res_pf = norm(res_pf, Inf);
        n_res_z2 = norm(res_z2, Inf);
        n_res_z3 = norm(res_z3, Inf);
        
        %% Update lambda
        lambda = lambda + var.rho.*res_pf;
        
        %% Exit condition
        if n_res_pf <= options.solver.tol && n_res_z2 <= options.solver.tol && n_res_z3 <= options.solver.tol
            done = true;
            e_flag = 1;
        elseif k >= options.solver.k_max
            done = true;
            e_flag = -1;
        end
        
        %% Update variables and save historics
        z2_prev = z2;
        z3_prev = z3;
        
        if genHist > 0
            hNRes_pf(k) = n_res_pf;
            hNRes_z2(k) = n_res_z2;
            hNRes_z3(k) = n_res_z3;
        end
        if genHist > 1
            hZ1(:, k) = z1;
            hZ2(:, k) = z2;
            hZ3(:, k) = z3;
            hLambda(:, k) = lambda;
        end
        
    end
    
    %% Return results
    
    % Control action
    if options.in_engineering
        u = z1(n+(1:m))./var.scaling_u + var.OpPoint_u;
    else
        u = z1(n+(1:m));
    end
    
    % Hist
        % Solution
    Hist.sol.z1 = z1;
    Hist.sol.z2 = z2;
    Hist.sol.z3 = z3;
    Hist.sol.lambda = lambda;
    Hist.res.pf = res_pf;
    Hist.res.z2 = res_z2;
    Hist.res.z3 = res_z3;
    Hist.res.norm_f = n_res_pf;
    Hist.res.norm_z2 = n_res_z2;
    Hist.res.norm_z3 = n_res_z3;
    Hist.k = k;
        % Historics
    if genHist > 0
        Hist.hNRes_pf = hNRes_pf(1:k);
        Hist.hNRes_z2 = hNRes_z2(1:k);
        Hist.hNRes_z3 = hNRes_z3(1:k);
    end
    if genHist > 1
        Hist.hZ1 = hZ1(:, 1:k);
        Hist.hZ2 = hZ2(:, 1:k);
        Hist.hZ3 = hZ3(:, 1:k);
        Hist.hLambda = hLambda(:, 1:k);
    end
    
end
