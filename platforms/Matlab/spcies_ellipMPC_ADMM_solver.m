%% spcies_ellipMPC_ADMM_solver - Solver for the lax MPC formulation using ADMM
%
% This is a non-sparse solver of the ADMM-based solver for MPC with ellipsoidal terminal constraint
%
% Information about this formulation and the solver can be found at:
% 
% P. Krupa, R. Jaouani, D. Limon, and T. Alamo, “A sparse ADMM-based solver for linear MPC subject
% to terminal quadratic constraint,” arXiv:2105.08419, 2021.
% 
% Specifically, this formulation is given in equation (9) of the above reference.
%
% [u, k, e_flag, Hist] = spcies_laxMPC_ADMM_solver(x0, xr, ur, 'name', value, 'name', ...) 
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
%          - .Nu: Vector defining the scaling of the system input.
%   - param: Structure containing the ingredients of the MPCT controller.
%          - .Q: Cost function matrix Q.
%          - .R: Cost function matrix R.
%          - .T: Cost function matrix T.
%          - .N: Prediction horizon.
%          - .P: Matrix defining the geometry of the ellipsoidal terminal constraint.
%          - .c: Matrix defining the center of the ellipsoidal terminal constraint.
%                Defaults to a vector of zeros (i.e., the origin).
%          - .r: Scalar defining the size of the ellipsoidal terminal constraint.
%                Defaults to 1.
%   - options: Structure containing options of the ADMM solver.
%              - .rho: Scalar or vector. Base value of the penalty parameter.
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
%           - .z Primal decision variables z.
%           - .v: Primal decision variables v.
%           - .lambda: Dual variables.
%           - .r_p: Value of the infinity norm of the primal residual.
%           - .r_d: Value of the infinity norm of the dual residual.
%       - hRp: Historic of the infinity norm of the primal residual (only saved in genHist > 0).
%       - hRd: Historic of the infinity norm of the dual residual (only saved in genHist > 0).
%       - hZ: Historic of z (only saved in genHist > 1).
%       - hV: Historic of v (only saved in genHist > 1).
%       - hLambda: Historic of lambda (only saved in genHist > 1).
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function [u, k, e_flag, Hist] = spcies_ellipMPC_ADMM_solver(x0, xr, ur, varargin)

    %% Default options
    def_sys = []; % Default value for the sys argument
    def_param = []; % Default value for the param argument
    def_controller = []; % Default value for the controller argument
    def_genHist = 0; % Default amount of data generated for Hist
    def_verbose = 1; % Default amount of information displayed
    def_options = ellipMPC.def_options_ellipMPC_ADMM(); % Default values of the options of the solver
    
    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'spcies_ellipMPC_ADMM_solver';
    
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
    
    % Add default values
    options = utils.add_default_options_to_struct(options, def_options);
    
    % Create the controller structure
    if isempty(par.Results.controller)
        controller.sys = par.Results.sys;
        controller.param = par.Results.param;
    else
        controller = par.Results.controller;
    end
    
    % Rename and check other arguments
    genHist = par.Results.genHist;
    if genHist > 2; genHist = 2; end
    if genHist < 0; genHist = 0; end
    verbose = par.Results.verbose;
    if verbose > 3; verbose = 3; end
    if verbose < 0; verbose = 0; end
    
    %% Generate ingredients of the solver
    [~, var] = ellipMPC.compute_ellipMPC_ADMM_ingredients(controller, options, []);
    N = var.N;
    n = var.n;
    m = var.m;
    
    %% Algorithm
    
    % Initialize
    done = false;
    k = 0;
    z = zeros(N*(n+m), 1);
    v = zeros(N*(n+m), 1);
    v1 = v; % Value of z in the previous iteration
    lambda = zeros(N*(n+m), 1);
    
    % Historics
    if genHist > 0
        hRp = zeros(1, options.k_max);
        hRd = zeros(1, options.k_max);
    end
    if genHist > 1
        hZ = zeros(N*(n+m), options.k_max);
        hV = zeros(N*(n+m), options.k_max);
        hLambda = zeros(N*(n+m), options.k_max);
    end
    
    % Obtain x0, xr and ur
    if options.in_engineering
        x0 = var.scaling_x*(x0 - var.OpPoint_x);
        xr = var.scaling_x*(xr - var.OpPoint_x);
        ur = var.scaling_u*(ur - var.OpPoint_u);
    end
    
    % Update b
    b = zeros(N*n, 1);
    b(1:n) = -var.A*x0;
    
    % Update q
    q = -[var.R*ur; kron(ones(N-1, 1), [var.Q*xr; var.R*ur]); var.T*xr];
    q_hat_k = zeros(length(q), 1);
    
    while~done
        k = k + 1;
        
        % Update q_hat_k
        q_hat_k(1:end-n) = q(1:end-n) + lambda(1:end-n) - var.rho.*v(1:end-n);
        q_hat_k(end-n+1:end) = q(end-n+1:end) + var.P_half*lambda(end-n+1:end) - var.rho.*var.P*v(end-n+1:end);
        
        % Compute mu
        mu = var.W\(-var.Aeq*var.Hinv*q_hat_k - b);
        
        % Update z_k
        z = -var.Hinv*(var.Aeq'*mu + q_hat_k);
        
        % Update v_k
        v(1:end-n) = max( min( z(1:end-n) + var.rho_i.*lambda(1:end-n), var.UBz), var.LBz);
        v(end-n+1:end) = z(end-n+1:end) + var.rho_i.*var.Pinv_half*lambda(end-n+1:end);
        
        % Project to ellipsoid
        vPv = (v(end-n+1:end) - var.c)'*var.P*(v(end-n+1:end) - var.c);
        
        if vPv > var.r^2
            v(end-n+1:end) = var.r*(v(end-n+1:end) - var.c)/sqrt(vPv) + var.c;
        end
        
        % Update lambda
        lambda(1:end-n) = lambda(1:end-n) + var.rho.*(z(1:end-n) - v(1:end-n));
        lambda(end-n+1:end) = lambda(end-n+1:end) + var.rho.*var.P_half*(z(end-n+1:end) - v(end-n+1:end));
        
        % Compute residuals
        r_p = norm(z - v, Inf);
        r_d = norm(v - v1, Inf);
        
        % Check exit condition
        if r_p <= options.tol && r_d <= options.tol
            done = true;
            e_flag = 1;
        elseif k >= options.k_max
            done = true;
            e_flag = -1;
        end
        
        % Update variables and historics
        v1 = v;
        
        if genHist > 0
            hRp(k) = r_p;
            hRd(k) = r_d;
        end
        if genHist > 1
            hZ(:, k) = z;
            hV(:, k) = v;
            hLambda(:, k) = lambda;
        end
        
    end
    
    %% Return results
    
    % Control action
    if options.in_engineering
        u = v(1:m)./var.scaling_u + var.OpPoint_u;
    else
        u = v(1:m);
    end
    
    % Hist
        % Solution
    Hist.sol.z = z;
    Hist.sol.v = v;
    Hist.sol.lambda = lambda;
    Hist.sol.r_p = r_p;
    Hist.sol.r_d = r_d;
    Hist.k = k;
        % Historics
    if genHist > 0
        Hist.hRp = hRp(1:k);
        Hist.hRd = hRd(1:k);
    end
    if genHist > 1
        Hist.hZ = hZ(:, 1:k);
        Hist.hV = hV(:, 1:k);
        Hist.hLambda = hLambda(:, 1:k);
    end

end
