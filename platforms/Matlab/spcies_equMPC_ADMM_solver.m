%% spcies_equMPC_ADMM_solver - Solver for the equality MPC formulation using ADMM
%
% This is a non-sparse solver of the ADMM-based equality MPC solver from the Spcies toolbox.
%
% Information about this formulation and the solver can be found at:
% 
% P. Krupa, D. Limon, T. Alamo, "Implementation of model predictive control in
% programmable logic controllers", Transactions on Control Systems Technology, 2020.
% 
% Specifically, this formulation is given in equation (8) of the above reference.
%
% [u, k, e_flag, Hist] = spcies_equMPC_ADMM_solver(x0, xr, ur, 'name', value, 'name', ...) 
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
%          - .N: Prediction horizon.
%   - controller: Alternatively, the sys and param arguments can be omitted 
%                 and instead substituted by an instance of the EqualityMPC
%                 class of the GepocToolbox (https://github.com/GepocUS/GepocToolbox).
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

function [u, k, e_flag, Hist] = spcies_equMPC_ADMM_solver(x0, xr, ur, varargin)
    
    %% Default options
    def_sys = []; % Default value for the sys argument
    def_param = []; % Default value for the param argument
    def_controller = []; % Default value for the controller argument
    def_genHist = 0; % Default amount of data generated for Hist
    def_verbose = 1; % Default amount of information displayed
    def_options = equMPC.def_options_equMPC_ADMM(); % Default values of the options of the solver
    
    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'spcies_equMPC_ADMM_solver';
    
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
        if ~isa(controller, 'EqualityMPC')
            error('Controller object must be of the EqualityMPC class');
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
    
    % Turn rho into a vector
    if isscalar(options.rho) && options.force_vector_rho
        rho = options.rho*ones(N*(n+m) - n, 1);
    else
        rho = options.rho;
    end
    
    % Compute the Hessian H and the vector q
    
    % Hessian and q for variable z
    H = blkdiag(R, kron(eye(N-1), blkdiag(Q, R)));
    if isscalar(rho)
        Hhat = H + rho*eye(N*(n+m) - n);
    else
        Hhat = H + diag(rho);
    end
    
    % Compute the matrix Aeq (equality constraints)
    
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
    Hinv = inv(Hhat);
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
    rho_i = 1./rho;
    Hinv = sparse(Hinv);
    Aeq = sparse(Aeq);
    W = sparse(W);
    
    %% Algorithm
    
    % Initialize
    done = false;
    k = 0;
    z = zeros((N-1)*(n+m)+m, 1);
    v = zeros((N-1)*(n+m)+m, 1);
    v1 = v;
    lambda = zeros((N-1)*(n+m)+m, 1);
    
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
    
    while~done
        k = k + 1;
        
        % Update qk
        q_k = q + lambda - rho.*v;
        
        % Update z_{k+1}
        z = solve_eqQP(q_k, b, Hinv, Aeq, W);
        
        % Update q_hat_k
        q_hat_k = -rho.*z - lambda;
        
        % Update v
        v = solve_boxQP(q_hat_k, rho_i, LB, UB);
        
        % Update lambda
        lambda = lambda + rho.*(z - v);
        
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
