%% spcies_MPCT_ADMM_solver - Solver for the MPCT formulation using the ADMM algorithm
%
% This is a non-sparse solver of the MPC for Tracking formulation from the Spcies toolbox.
% The solver extends the state and control inputs by adding the artificial reference to them.
% Currently, there is no additional documentation available for the solver.
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
%   - Hist: Structure containing information of the iterations and final outcome of the solver
%       - sol: Structure containing the returned solution of the primal and dual variables.
%           - .z1: Returned decision variables z1.
%           - .z2: Returned decision variables z2.
%           - .z3: Returned decision variables z3.
%           - .lambda: Returned dual variables.
%           - .res: Value of the residual.
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

function [u, k, e_flag, Hist] = spcies_MPCT_ADMM_cs_solver(x0, xr, ur, varargin)
    import MPCT.compute_MPCT_ADMM_cs_ingredients
    
    %% Default values
    def_sys = []; % Default value for the sys argument
    def_param = []; % Default value for the param argument
    def_controller = []; % Default value for the controller argument
    def_genHist = 0; % Default amount of data generated for Hist
    def_verbose = 1; % Default amount of information displayed
    def_options = MPCT.def_options_MPCT_cs_ADMM(); % Default values of the options of the solver
    
    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'spcies_MPCT_ADMM_solver';
    
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
    options = utils.add_default_options_to_struct(options, def_options);

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
    var = compute_MPCT_ADMM_cs_ingredients(controller, options, []);
    N = var.N;
    n = var.n;
    m = var.m;
    
    %% Algorithm
    
    % Initialize
    done = false;
    k = 0;
    z = zeros(N*2*(n+m), 1);
    v = zeros(N*2*(n+m), 1);
    v1 = v; % Value of z in the previous iteration
    lambda = zeros(N*2*(n+m), 1);
    
    % Historics
    if genHist > 0
        hRp = zeros(1, options.k_max);
        hRd = zeros(1, options.k_max);
    end
    if genHist > 1
        hZ = zeros(N*2*(n+m), options.k_max);
        hV = zeros(N*2*(n+m), options.k_max);
        hLambda = zeros(N*2*(n+m), options.k_max);
    end
    
    % Obtain x0, xr and ur
    if options.in_engineering
        x0 = var.scaling_x*(x0 - var.OpPoint_x);
        xr = var.scaling_x*(xr - var.OpPoint_x);
        ur = var.scaling_u*(ur - var.OpPoint_u);
    end
    
    % Update b
    b = zeros(2*n+(2*n+m)*(N-1)+n, 1);
    b(1:n) = x0;
    
    % Update q
    q = kron(ones(N, 1), blkdiag(zeros(n), var.Tz, zeros(m), var.Sz)*[zeros(n, 1); xr; zeros(m, 1); ur]);
    
    % Make LB and UB vectors
    LB = var.LB(:);
    UB = var.UB(:);
    
    while ~done
        k = k + 1;
        
        % Compute q_hat = q + lambda - rho*v
        q_hat = q + lambda - var.rho.*v;
        
        % Compute the right-hand-side of the system of equations W*mu = rhs   
        rhs = utils.smv(var.AHi_CSR.val, var.AHi_CSR.col, var.AHi_CSR.row, q_hat); % Perform the sparse matrix vector multiplication
        rhs(1:n) = rhs(1:n) - b(1:n); % Substract b
        
        % Solve the W*mu = rhs system of equations
        mu = utils.LDLsolve(var.L_CSC.val, var.L_CSC.row, var.L_CSC.col, var.Dinv, rhs);
        
        % Update z
        z = utils.smv(var.Hi_CSR.val, var.Hi_CSR.col, var.Hi_CSR.row, q_hat)...
            + utils.smv(var.HiA_CSR.val, var.HiA_CSR.col, var.HiA_CSR.row, mu);
        
        % Update v
        v = max( min( z + var.rho_i.*lambda, UB), LB);
        
        % Update lambda
        lambda = lambda + var.rho.*(z - v);
        
        % Compute residuals
        r_p = norm(z - v, Inf);
        r_d = norm(v1 - v, Inf);
        
        % Check exit conditions
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
        u = v(2*n+(1:m))./var.scaling_u + var.OpPoint_u;
    else
        u = v(2*n+(1:m));
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
