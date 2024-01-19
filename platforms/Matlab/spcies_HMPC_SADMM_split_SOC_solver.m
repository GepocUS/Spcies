%% Non-sparse verison of the solver for the HMPC formulation with box constraints based on SADMM
% This version projects onto SOCs, instead of the "diamond" sets.
% It also uses the ADMM splitting method (\hat z, \hat s) = (z, s).
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function [u, k, e_flag, Hist] = spcies_HMPC_SADMM_split_SOC_solver(x0, xr, ur, varargin)
    
    %% Default options
    def_sys = []; % Default value for the sys argument
    def_param = []; % Default value for the param argument
    def_controller = []; % Default value for the controller argument
    def_genHist = 0; % Default amount of data generated for Hist
    def_verbose = 1; % Default amount of information displayed
    def_options = HMPC.def_options_HMPC_SADMM();
    
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
    
    % Determine which solver to use: box-constrained or coupled-inputs
    if isempty(options.box_constraints)
        if isfield(controller.sys, 'E')
            options.box_constraints = false;
        else
            options.box_constraints = true;
        end
    end

    %% Generate ingredients of the solver
    var = HMPC.compute_HMPC_SADMM_split_SOC_ingredients(controller, options, []);
    N = var.N;
    n = var.n;
    m = var.m;
    n_y = var.n_y;
    n_soc = var.n_soc;
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
    
    % Update q and b using x0, xr and ur
    q = -[zeros((N-1)*(n+m)+m, 1); var.Te*xr; zeros(2*n,1); var.Se*ur; zeros(2*m,1)];
    bh = var.bh;
    bh(1:n) = -var.A*x0;
    
    %% Algorithm
    while ~done
        k = k + 1;
        
        % Step 1: Compute z_hat_{k+1} and s_{k+1}
        
        % Compute q_hat
        q_hat = [q - var.sigma*z + lambda; mu - var.rho*s]; % Compute q_hat
        
        % Compute right hand side of the system of equations
        if ~options.sparse
            rhs = var.M1*q_hat + var.M2*bh;
        else
            rhs = sp_utils.LDLsolve(var.L_CSC.val, var.L_CSC.row, var.L_CSC.col, var.Dinv, var.Pldl'*[-q_hat; bh]);
        end
            
        z_hat = rhs(1:dim);
        s_hat = rhs(dim+1:dim+n_s);
        
        % Step 2: Compute lambda_{k+1/2} and mu_{k+1/2}
        lambda = lambda + options.alpha*var.sigma*(z_hat - z);
        mu = mu + options.alpha*var.rho.*(s_hat - s);

        % Step 3: Compute z_{k+1}
        z = z_hat + lambda/var.sigma;
        if options.box_constraints
            z(1:(N-1)*(n+m)+m) = max( min(z(1:(N-1)*(n+m)+m) , var.UB), var.LB);
        end
        
        % Step 4: Compute s_{k+1}
        
        s_proj = s_hat + mu/var.rho;
        
        if options.box_constraints
        
            % Projection for s subject to the SOC constraints
            for j = 1:n_soc
                s(3*(j-1) + (1:3)) = sp_utils.proj_SOC(s_proj(3*(j-1) + (1:3)));
            end
            
        else
            
            % Projection for s subject to box constraints
            for j = 0:N-1
                s(j*n_y+(1:n_y)) = max( min( s_proj(j*n_y+(1:n_y)), var.UBy), var.LBy);
            end
            % Projection for s subject to the SOC constraints
            for j = 1:n_soc
                s(N*n_y + 3*(j-1) + (1:3)) = sp_utils.proj_SOC(s_proj(N*n_y + 3*(j-1) + (1:3)));
            end
            
        end
        
        % Step 5: Compute lambda_{k+1}
        lambda = lambda + options.alpha*var.sigma*(z_hat - z);
        mu = mu + options.alpha*var.rho.*(s_hat - s);
        
        % Step 6: Compute residuals and exit tolerance
        rp = norm([z_hat; s_hat] - [z; s], Inf);
        rd = norm([z; s] - [z_ant; s_ant], Inf);

        z_ant = z;
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
        hZ(:, k) = z; hS(:, k) = s; hLambda(:, k) = lambda; hMu(:, k) = mu; 
        hZhat(:, k) = z_hat; hShat(:, k) = s_hat; hS_proj(:, k) = s_proj;
        hRp(k) = rp; hRd(k) = rd;
        
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
