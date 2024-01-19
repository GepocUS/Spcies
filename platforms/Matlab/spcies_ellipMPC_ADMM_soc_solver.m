%% spcies_ellipMPC_ADMM_solver - Solver for the lax MPC formulation using ADMM
%
% This is a non-sparse solver of the ADMM-based solver for MPC with ellipsoidal terminal constraint.
% This version uses a SOC constraint to deal with the quadratic terminal constraint.
%
% The ellipMPC formulation can be found at 
% 
% P. Krupa, R. Jaouani, D. Limon, and T. Alamo, “A sparse ADMM-based solver for linear MPC subject
% to terminal quadratic constraint,” arXiv:2105.08419, 2021.
% 
% However, there is currently no specific documentation on this solver.
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

function [u, k, e_flag, Hist] = spcies_ellipMPC_ADMM_soc_solver(x0, xr, ur, varargin)

    %% Default options
    def_sys = []; % Default value for the sys argument
    def_param = []; % Default value for the param argument
    def_controller = []; % Default value for the controller argument
    def_genHist = 0; % Default amount of data generated for Hist
    def_verbose = 1; % Default amount of information displayed
    def_options = ellipMPC.def_options_ellipMPC_ADMM_soc; % Get defult solver options
    
    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'spcies_ellipMPC_ADMM_soc_solver';
    
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
    options = sp_utils.add_default_options_to_struct(options, def_options);
    
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
    var = ellipMPC.compute_ellipMPC_ADMM_soc_ingredients(controller, options, []);
    N = var.N;
    n = var.n;
    m = var.m;
    dim = var.dim; % Number of decision variables
    n_eq = var.n_eq; % Number of constraints: equality + box + cone (a single cone is 3)
    n_s = var.n_s; % Number of rows of matrix C
    k_max = options.k_max;
    tol_p = options.tol_p;
    tol_d = options.tol_d;
    
    %% Algorithm
    
    % Initialize
    done = false;
    k = 0;
    z = zeros(dim, 1); % Decision variables
    s = zeros(n_s, 1); % Slack variables
    lambda = zeros(dim, 1); % Dual variabels for x
    mu = zeros(n_s, 1); % Dual varaibles for s
    z_hat = zeros(dim, 1); % Dummy variable for decision variables
    s_hat = zeros(n_s, 1); % Dummy variable for slack variables
    s_proj = zeros(n_s, 1); % Vector that is projected onto the sets to obtain s
    
    % Historics
    hZ = zeros(dim, k_max);
    hS = zeros(n_s, k_max);
    hZhat = zeros(dim, k_max);
    hShat = zeros(n_s, k_max);
    hMu =  zeros(n_s, k_max);
    hLambda = zeros(dim, k_max);
    hS_proj = zeros(n_s, k_max);
    hRp = zeros(1, k_max);
    hRd = zeros(1, k_max);
    
    z_ant = z;
    s_ant = s;
    
    % Obtain x0, xr and ur
    if options.in_engineering
        x0 = var.scaling_x*(x0 - var.OpPoint_x);
        xr = var.scaling_x*(xr - var.OpPoint_x);
        ur = var.scaling_u*(ur - var.OpPoint_u);
    end
    
    % Update bh
    bh = var.bh;
    bh(1:n) = -var.A*x0;
    bh(n_eq) = controller.param.r;
    bh(n_eq+1+(1:n)) = -var.PhiP*xr;
    
    % Update q
    q = [var.R*ur; kron(ones(N-1, 1), [var.Q*xr; var.R*ur]); var.T*xr; 0];
    
    while~done
        k = k + 1;
        
        % Step 1: Compute z_hat_{k+1} and s_{k+1}
        q_hat = [q - var.sigma*z + lambda; mu - var.rho*s]; % Compute q_hat
        
        % Solve system of equations
            % Compute r.h.s.
        rhs = sp_utils.smv(var.GhHhi_CSR.val, var.GhHhi_CSR.col, var.GhHhi_CSR.row, q_hat) - bh;
            % Solve system of equations
        psi = sp_utils.LDLsolve(var.L_CSC.val, var.L_CSC.row, var.L_CSC.col, var.Dinv, rhs);
            % Compute solution
        aux = sp_utils.smv(var.HhiGh_CSR.val, var.HhiGh_CSR.col, var.HhiGh_CSR.row, psi) +...
              sp_utils.smv(var.Hhi_CSR.val, var.Hhi_CSR.col, var.Hhi_CSR.row, q_hat);
        
        %aux = var.M1*q_hat + var.M2*bh; % aux = [z_hat; s_hat]

        z_hat = aux(1:dim);
        s_hat = aux(dim+1:end);
        
        % Step 2.1: Compute z_{k+1}
        z = z_hat + lambda/var.sigma;
        z(1:(N-1)*(n+m)+m) = max( min(z(1:(N-1)*(n+m)+m) , var.UB), var.LB);
        
        % Step 3: Compute s_{k+1}
        s_proj = s_hat + mu/var.rho;
        
        % Projection onto the SOC
        s_proj_0 = s_proj(1);
        s_proj_1 = s_proj(2:end);
        ns_proj_1 = norm(s_proj_1, 2);
        
        if ns_proj_1 <= s_proj_0
            s = s_proj;
        elseif ns_proj_1 <= -s_proj_0
            s = zeros(n+1, 1);
        else
            s = (s_proj_0 + ns_proj_1)/(2*ns_proj_1)*[ns_proj_1; s_proj_1];
        end
        
        % Step 3: Compute lambda_{k+1}
        lambda = lambda + var.sigma*(z_hat - z);
        
        % Step 4: Compute y_{k+1}
        mu = mu + var.rho.*(s_hat - s);
        
        % Step 5: Compute residuals and exit tolerance
        rp = norm([z_hat; s_hat] - [z; s], Inf);
        rd = norm([z; s] - [z_ant; s_ant], Inf);
        
        z_ant = z;
        s_ant = s;
        
        % Step 6: Exit condition
        if (rp <= tol_p) && (rd <= tol_d)
            done = true;
            e_flag = 1;
        elseif k >= k_max
            done = true;
            e_flag = -1;
        end

        % Update historics
        hZ(:, k) = z; hS(:, k) = s; hLambda(:, k) = lambda; hMu(:, k) = mu; 
        hZhat(:, k) = z_hat; hShat(:, k) = s_hat; hS_proj(:, k) = s_proj;
        hRp(k) = rp; Rd(k) = rd;
        
    end
    
    %% Return results
    
    % Control action
    if options.in_engineering
        u = z(1:m)./var.scaling_u + var.OpPoint_u;
    else
        u = z(1:m);
    end
    
    % Hist
        % Solution
    Hist.sol.z = z;
    Hist.sol.s = s;
    Hist.sol.z_hat = z_hat;
    Hist.sol.s_hat = s_hat;
    Hist.sol.lambda = lambda;
    Hist.sol.mu = mu;
    Hist.sol.rp = rp;
    Hist.sol.rd = rd;
    Hist.bh = bh;
    Hist.q_hat = q_hat;
    Hist.q = q;
    Hist.rhs = rhs;
    Hist.psi = psi;
    Hist.k = k;
        % Historics
    Hist.hZ = hZ(:,1:k); % Historic of z
    Hist.hS = hS(:,1:k); % Historic of s
    Hist.hLambda = hLambda(:,1:k); % Historic of dual variable lambda
    Hist.hMu = hMu(:,1:k); % Historic of dual variable y
    Hist.hZhat = hZhat(:,1:k); % Historic of x_hat
    Hist.hShat = hShat(:,1:k); % Historic of s_hat
    Hist.hS_proj = hS_proj(:,1:k); % Historic for the non-projected slack variables s_proj
    Hist.hRp = hRp(1:k); % Historic of the primal residual
    Hist.hRd = hRd(1:k); % Historic of the dual residual

end
