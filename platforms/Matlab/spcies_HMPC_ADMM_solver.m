%% spcies_HMPC_ADMM_solver - Solver for HMPC based on ADMM
%
% This is a non-sparse verison of the solver for the HMPC formulation based on ADMM
%
% Information about the solver canbe found in:
%
% Pablo Krupa, Daniel Limon, Alberto Bemporad, Teodoro Alamo, "Efficiently
% solving the hamonic model predictive control formulation", arXiv: 2202.06629, 2022.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function [u, k, e_flag, Hist] = spcies_HMPC_ADMM_solver(x0, xr, ur, varargin)
    
    %% Default options
    def_sys = []; % Default value for the sys argument
    def_param = []; % Default value for the param argument
    def_controller = []; % Default value for the controller argument
    def_genHist = 0; % Default amount of data generated for Hist
    def_verbose = 1; % Default amount of information displayed
    def_options = HMPC.def_options_HMPC_ADMM();
    
    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'spcies_HMPC_ADMM_solver';
    
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
    
    % Determine which solver to use: box-constrained or coupled-inputs
    if isempty(options.box_constraints)
        if isfield(controller.sys, 'E')
            options.box_constraints = false;
        else
            options.box_constraints = true;
        end
    end

    %% Generate ingredients of the solver
    var = HMPC.compute_HMPC_ADMM_ingredients(controller, options, []);
    
    N = var.N;
    n = var.n;
    m = var.m;
    n_y = var.n_y;
    n_soc = var.n_soc;
    dim = var.dim; % Number of decision variables
    n_eq = var.n_eq; % Number of constraints: equality + box + cone (a single cone is 3)
    n_s = var.n_s; % Number of rows of matrix C
    n_box = var.n_box;
    k_max = options.k_max;
    tol_p = options.tol_p;
    tol_d = options.tol_d;
    
    %% Algorithm
    
    % Initialize
    done = false;
    k = 0;
    z = zeros(dim, 1); % Decision variables
    s = zeros(n_s, 1); % Slack variables
    lambda = zeros(n_s, 1); % Dual variabels for x
    s_proj = zeros(n_s, 1); % Vector that is projected onto the sets to obtain s
    
    % Historics
    hZ = zeros(dim, k_max);
    hS = zeros(n_s, k_max);
    hLambda = zeros(n_s, k_max);
    hS_proj = zeros(n_s, k_max);
    hRp = zeros(1, k_max);
    hRd = zeros(1, k_max);
    
    s_ant = s;
    
    % Obtain x0, xr and ur
    if options.in_engineering
        x0 = var.scaling_x*(x0 - var.OpPoint_x);
        xr = var.scaling_x*(xr - var.OpPoint_x);
        ur = var.scaling_u*(ur - var.OpPoint_u);
    end
    
    % Other variables
    q = -[zeros((N-1)*(n+m)+m, 1); var.Te*xr; zeros(2*n,1); var.Se*ur; zeros(2*m,1)]; % Add reference
    b = -var.A*x0; % Add current state
    d = var.d;
    C = var.C;
    
    if strcmp(options.method, 'ADMM')
        options.alpha = 1;
    end
    
    %% Algorithm
    while ~done
        k = k + 1;
        
        %%%%%%%%%%%%% Update z %%%%%%%%%%%%%
        
        % Compute q_hat
        q_hat = q + utils.smv(var.Ct_CSR.val, var.Ct_CSR.col, var.Ct_CSR.row, var.rho*(s - d) + lambda);
        
        % Compute right hand side of the system of equations
        if ~options.sparse
            z = var.M1*q_hat + var.M2*b;
        else
            error('Only sparse for now in the reduced ADMM version');
%             rhs = utils.LDLsolve(var.L_CSC.val, var.L_CSC.row, var.L_CSC.col, var.Dinv, var.Pldl'*[-q_hat; bh]);
%             z = utils.LDLsolve(var.L_CSC.val, var.L_CSC.row, var.L_CSC.col, var.Dinv, [-q_hat; bh]);
        end
        
        % Compute C*z - d
        Czd = utils.smv(var.C_CSR.val, var.C_CSR.col, var.C_CSR.row, z) - d;
        
        %%%%%%%%%%%%% Update dual (in the SADMM variant) %%%%%%%%%%%%%
        if strcmp(options.method, 'SADMM')
            lambda = lambda + options.alpha*var.rho*(Czd + s);
        end
        
        %%%%%%%%%%%%% Update s %%%%%%%%%%%%%
        
        s_proj = -Czd - lambda/var.rho;
        
        % Projection for s subject to box constraints
        for j = 1:n_box
            s(j) = max( min( s_proj(j), var.UB(j)), var.LB(j));
        end

        % Projection for s subject to the SOC constraints
        if options.use_soc
            for j = 1:n_soc
                s(n_box + 3*(j-1) + (1:3)) = utils.proj_SOC(s_proj(n_box + 3*(j-1) + (1:3)));
            end
        else
            for j = 1:n_y
                s(n_box + 3*(j-1) + (1:3)) = utils.proj_D(s_proj(n_box + 3*(j-1) + (1:3)), var.LBy(j), var.UBy(j));
            end
        end
        
        %%%%%%%%%%%%% Add s to Czd %%%%%%%%%%%%%
        Czd = Czd + s;
        
        %%%%%%%%%%%%% Update dual %%%%%%%%%%%%%
        lambda = lambda + options.alpha*var.rho*Czd;
        
        % Step 6: Compute residuals and exit tolerance
        rp = norm(Czd, Inf);
        rd = norm(s - s_ant, Inf);
        
        s_ant = s;
        
        % Step 7: Exit condition
        if rp <= tol_p && rd <= tol_d
            done = true;
            e_flag = 1;
        elseif k >= k_max
            done = true;
            e_flag = -1;
        end

        % Update historics
        hZ(:, k) = z; hS(:, k) = s; hLambda(:, k) = lambda;
        hS_proj(:, k) = s_proj; hRp(k) = rp; hRd(k) = rd;
        
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
    Hist.sol.lambda = lambda;
    Hist.sol.rp = rp;
    Hist.sol.rd = rd;
    Hist.b = b;
    Hist.q_hat = q_hat;
    Hist.q = q;
    Hist.k = k;
        % Historics
    Hist.hZ = hZ(:,1:k); % Historic of z
    Hist.hS = hS(:,1:k); % Historic of s
    Hist.hLambda = hLambda(:,1:k); % Historic of dual variable lambda
    Hist.hS_proj = hS_proj(:,1:k); % Historic for the non-projected slack variables s_proj
    Hist.hRp = hRp(1:k); % Historic of the primal residual
    Hist.hRd = hRd(1:k); % Historic of the dual residual

end
