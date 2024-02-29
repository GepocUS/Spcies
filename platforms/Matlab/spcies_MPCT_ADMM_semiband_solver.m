%% spcies_MPCT_ADMM_semiband_solver - Solver for the MPCT formulation using the ADMM algorithm
%
% This is a non-sparse solver of the MPC for Tracking formulation from the Spcies toolbox.
%
% This solver uses the Woodbury Matrix Identity to efficiently compute the iterate z^{k+1} of ADMM when it is applied to the MPC for Tracking formulation.
% This step consists of an equality-contrained QP whose Hessian presents a semi-banded structure.
% The solver considers two possibilities:
%   1. options.solver.soft_constraints = false --> MPCT formulation with terminal equality constraint. 
%   No box constraints in outputs allowed. All box constraints are hard.
%   2. options.solver.soft_constraints = true --> MPCT formulation with terminal equality constraint.
%   Box constraints in outputs allowed. All constraints are soft except for the applied input u_0.

% Information about this formulation and the solver can be found at:
% 
% TODO: Write the name of the article here if published
%
% [u, k, e_flag, Hist] = spcies_MPCT_ADMM_semiband_solver(x0, xr, ur, 'name', value, 'name', ...)
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
%          * Only if options.solver.soft_constraints == true *
%               - .C_c: matrix C_c of the constrained output vector y_c = C_c*x + D_c*u.
%               - .D_c: matrix D_c of the constrained output vector y_c = C_c*x + D_c*u.
%          It can optionally also contain the following fields:
%          - .xOptPoint: operating point for the system state.
%          - .uOptPoint: operating point for the system input.
%          - .LBx: Lower bound for the system state.
%          - .UBx: Upper bound for the system state.
%          - .LBu: Upper bound for the system input.
%          - .UBu: Upper bound for the system input.
%          * Only if options.solver.soft_constraints == true *
%               - .LBy: Lower bound for the constrained output vector y_c.
%               - .UBy: Upper bound for the constrained output vector y_c.
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
%   - options: Structure containing options of the ADMM_semiband solver.
%              - .soft_constraints: Boolean to choose if softened box
%              constraints are considered, as well as softened box constraints in outputs.
%              - .rho: Penalty parameter of ADMM. Can be either a scalar or
%                      a vector. Defaults to 1e-2.
%              * Only if options.solver.soft_constraints == true *
%                   - .beta: Parameter to weight softened box constraints.
%                            Can be either a scalar or a vector. Defaults to 1.
%              - .inf_bound: Scalar. Determines the value given to components without bound.
%              - .tol_p: Primal exit tolerance of the solver. Defaults to 1e-4.
%              - .tol_d: Dual exit tolerance (dual) of the solver. Defaults to 1e-4.
%              - .k_max: Maximum number of iterations of the solver. Defaults to 1000.
%              - .in_engineering: Boolean that determines if the arguments of the solver are given in
%                                 engineering units (true) or incremental ones (false - default).
%              * Only if options.solver.soft_constraints == false *
%                   - .epsilon_x: Vector by which the bound for x_s are reduced when there are no soft constraints.
%                   - .epsilon_u: Vector by which the bound for u_s are reduced when there are no soft constraints.
%   - verbose: Controls the amount of information printed in the console.
%              Integer from 0 (no information printed) to 3 (print all information).
%   - genHist: Controls the amount of information saved and returned in the output Hist.
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
%           - z: Returned decision variables z.
%           - v: Returned decision variables v.
%           - .lambda: Returned dual variables.
%       - res: Structure containing information of the values of the residuals of the returned solution.
%       - hNRes_pf: Historic of the infinity norm of the residual of primal feasibility (only saved if genHist > 0).
%       - hNRes_v: Historic of the infinity norm of the residual of fixed point for v (only saved if genHist > 0).
%       - hZ: Historic of z (only saved if genHist > 1).
%       - hV: Historic of v (only saved if genHist > 1).
%       - hLambda: Historic of lambda (only saved if genHist > 1).
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
%

% TODO: Change beta from option to necessary input of the solver when options.solver.soft_constraints == true and make it deal with beta being a vector.

function [u, k, e_flag, Hist] = spcies_MPCT_ADMM_semiband_solver(x0, xr, ur, varargin)
    import MPCT.compute_MPCT_ADMM_semiband_ingredients

    %% Default values
    def_sys = []; % Default value for the sys argument
    def_param = []; % Default value for the param argument
    def_controller = []; % Default value for the controller argument
    def_genHist = 0; % Default amount of data generated for Hist
    def_verbose = 1; % Default amount of information displayed
    def_options = MPCT.def_options_MPCT_ADMM_semiband();

    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'spcies_MPCT_ADMM_semiband_solver';

    % Name-value parameters
    addParameter(par, 'sys', def_sys, @(x) isa(x, 'ss') || isa(x,'ssModel') || isstruct(x));
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
    options = Spcies_options('formulation', 'MPCT', 'method', 'ADMM', 'submethod', 'semiband', 'options', options);

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
    var = compute_MPCT_ADMM_semiband_ingredients(controller, options);
    N = var.N;
    n = var.n; % Dimension of state space
    m = var.m; % Dimension of input space
    if options.solver.constrained_output
        pp = var.p; % Dimension of constrained output
    end

    %% Algorithm

    % Initialize
    done = false;
    k = 0;
    z = zeros((N+1)*(n+m),1);
    if ~options.solver.constrained_output
        v = zeros((N+1)*(n+m),1);
        v_old = zeros((N+1)*(n+m),1); % Value of v in the previous iteration
        lambda = zeros((N+1)*(n+m),1);
    else
        v = zeros((N+1)*(n+m+pp),1);
        v_old = zeros((N+1)*(n+m+pp),1); % Value of v in the previous iteration
        lambda = zeros((N+1)*(n+m+pp),1);
    end

    % Historics
    if genHist > 0
        hRp = zeros(1, options.solver.k_max); % Primal residual
        hRd = zeros(1, options.solver.k_max); % Dual residual
    end
    if genHist > 1
        hZ = zeros((N+1)*(n+m), options.solver.k_max);
        if ~options.solver.constrained_output
            hV = zeros((N+1)*(n+m), options.solver.k_max);
            hLambda = zeros((N+1)*(n+m), options.solver.k_max);
        else
            hV = zeros((N+1)*(n+m+pp), options.solver.k_max);
            hLambda = zeros((N+1)*(n+m+pp), options.solver.k_max);
        end
    end
    
    % Obtain x0, xr and ur
    if options.in_engineering
        x0 = var.scaling_x*(x0 - var.OpPoint_x);
        xr = var.scaling_x*(xr - var.OpPoint_x);
        ur = var.scaling_u*(ur - var.OpPoint_u);
    end

    % Update beq
    beq = zeros((N+2)*n,1);
    beq(1:n,1) = x0;

    % Update q
    q = zeros((N+1)*(n+m),1);

    q(end-n-m+1:end-m,1) = -var.T*xr;
    
    q(end-m+1:end,1) = -var.S*ur;

    while ~done
        k = k + 1;

        % Equality constrained QP solve : Update z

        if ~options.solver.constrained_output
            if isscalar(var.rho)
                p = q + lambda - var.rho*v;
            else
                p = q + lambda - var.rho.*v;
            end
        else
            if isscalar(var.rho)
                p = q + var.C_tilde'*(lambda - var.rho*v);
            else
                p = q + var.C_tilde'*(lambda - var.rho.*v);
            end
        end

        % Compute xi from eq. (9a) using Alg. 2 from the article

        % Obtaining z1_a = var.Gamma_hat\p;
        
        % The following code solves a banded-diagonal system of equations.
        % In C we create a function, as we need to solve it four times.
        
        % 1st banded-diagonal system
        if isscalar(var.rho)
            for i = 1:n+m:(N)*(n+m)
                z1_a(i:i+n-1,1) = var.Q_rho_i*p(i:i+n-1);
            end
            for i = n+1:n+m:(N)*(n+m)
                z1_a(i:i+m-1,1) = var.R_rho_i*p(i:i+m-1);
            end
        else
            for i = 1:n+m:(N)*(n+m)
                z1_a(i:i+n-1,1) = var.Q_rho_i(:,:,(i-1)/(n+m)+1)*p(i:i+n-1);
            end
            for i = n+1:n+m:(N)*(n+m)
                z1_a(i:i+m-1,1) = var.R_rho_i(:,:,(i-n-1)/(n+m)+1)*p(i:i+m-1);
            end
        end
        z1_a(N*(n+m)+1:N*(n+m)+n) = var.T_rho_i*p(N*(n+m)+1:N*(n+m)+n);
        z1_a(N*(n+m)+n+1:(N+1)*(n+m)) = var.S_rho_i*p(N*(n+m)+n+1:(N+1)*(n+m));
        % End of code for banded-diagonal system of equations.

        % Obtaining z2_a = (eye(2*(n+m))+var.V_hat*var.Gamma_hat_inv*var.U_hat)\(var.V_hat*z1_a);
        z2_a = var.M_hat * z1_a;

        % Obtaining z3_a = var.Gamma_hat\(var.U_hat*z2_a);
        vec = var.U_hat*z2_a;
        
        % 2nd banded-diagonal system
        if isscalar(var.rho)
            for i = 1:n+m:(N)*(n+m)
                z3_a(i:i+n-1,1) = var.Q_rho_i*vec(i:i+n-1);
            end
            for i = n+1:n+m:(N)*(n+m)
                z3_a(i:i+m-1,1) = var.R_rho_i*vec(i:i+m-1);
            end
        else
            for i = 1:n+m:(N)*(n+m)
                z3_a(i:i+n-1,1) = var.Q_rho_i(:,:,(i-1)/(n+m)+1)*vec(i:i+n-1);
            end
            for i = n+1:n+m:(N)*(n+m)
                z3_a(i:i+m-1,1) = var.R_rho_i(:,:,(i-n-1)/(n+m)+1)*vec(i:i+m-1);
            end
        end
        z3_a(N*(n+m)+1:N*(n+m)+n) = var.T_rho_i*vec(N*(n+m)+1:N*(n+m)+n);
        z3_a(N*(n+m)+n+1:(N+1)*(n+m)) = var.S_rho_i*vec(N*(n+m)+n+1:(N+1)*(n+m));
        % End of code for banded-diagonal system of equations.
        
        % Computation of xi
        xi = z1_a - z3_a;

        % Compute mu from eq. (9b) using Alg. 2 from the article
        
        % Obtaining z1_b
        z1_b = var.Gamma_tilde\-(var.G*xi+beq); % In C, -(var.G*xi+beq) is computed online and then we use solve_banded_chol()

        % Obtaining z2_b = (eye(2*(n+m))+var.V_tilde*var.Gamma_tilde_inv*var.U_tilde_full)\(var.V_tilde*z1_b);
        z2_b = var.M_tilde_full * z1_b;
        
        % Obtaining z3_b
        z3_b = var.Gamma_tilde\(var.U_tilde_full*z2_b); % In C, (var.U_tilde*z2_b) is computed online and then we use solve_banded_chol()
    
        % Computation of mu
        mu = z1_b - z3_b;

        % Compute z^{k+1} from eq. (9c) using Alg. 2 from the article
        
        % Obtaining z1_c = var.Gamma_hat\-(var.G'*mu+p);
        vec = -(var.G'*mu+p);
        
        % 3rd banded-diagonal system
        if isscalar(var.rho)
            for i = 1:n+m:(N)*(n+m)
                z1_c(i:i+n-1,1) = var.Q_rho_i*vec(i:i+n-1);
            end
            for i = n+1:n+m:(N)*(n+m)
                z1_c(i:i+m-1,1) = var.R_rho_i*vec(i:i+m-1);
            end
        else
            for i = 1:n+m:(N)*(n+m)
                z1_c(i:i+n-1,1) = var.Q_rho_i(:,:,(i-1)/(n+m)+1)*vec(i:i+n-1);
            end
            for i = n+1:n+m:(N)*(n+m)
                z1_c(i:i+m-1,1) = var.R_rho_i(:,:,(i-n-1)/(n+m)+1)*vec(i:i+m-1);
            end
        end
        z1_c(N*(n+m)+1:N*(n+m)+n) = var.T_rho_i*vec(N*(n+m)+1:N*(n+m)+n);
        z1_c(N*(n+m)+n+1:(N+1)*(n+m)) = var.S_rho_i*vec(N*(n+m)+n+1:(N+1)*(n+m));
        % End of code for banded-diagonal system of equations.
        
        % Obtaining z2_c = (eye(2*(n+m))+var.V_hat*var.Gamma_hat_inv*var.U_hat)\(var.V_hat*z1_c);
        z2_c = var.M_hat * z1_c;

        % Obtaining z3_c = var.Gamma_hat\(var.U_hat*z2_c);
        vec = var.U_hat*z2_c;
        
        % 4th banded-diagonal system
        if isscalar(var.rho)
            for i = 1:n+m:(N)*(n+m)
                z3_c(i:i+n-1,1) = var.Q_rho_i*vec(i:i+n-1);
            end
            for i = n+1:n+m:(N)*(n+m)
                z3_c(i:i+m-1,1) = var.R_rho_i*vec(i:i+m-1);
            end
        else
            for i = 1:n+m:(N)*(n+m)
                z3_c(i:i+n-1,1) = var.Q_rho_i(:,:,(i-1)/(n+m)+1)*vec(i:i+n-1);
            end
            for i = n+1:n+m:(N)*(n+m)
                z3_c(i:i+m-1,1) = var.R_rho_i(:,:,(i-n-1)/(n+m)+1)*vec(i:i+m-1);
            end
        end
        z3_c(N*(n+m)+1:N*(n+m)+n) = var.T_rho_i*vec(N*(n+m)+1:N*(n+m)+n);
        z3_c(N*(n+m)+n+1:(N+1)*(n+m)) = var.S_rho_i*vec(N*(n+m)+n+1:(N+1)*(n+m));
        % End of code for banded-diagonal system of equations.

        % Computation of z^{k+1}
        z = z1_c - z3_c;

        % Inequality constrained QP solve: Update v
        
        if ~options.solver.constrained_output
            if isscalar(var.rho)
                v = var.rho_i*lambda + z;
            else
                v = var.rho_i.*lambda + z;
            end
        else
            if isscalar(var.rho)
                v = var.rho_i*lambda + var.C_tilde*z;
            else
                v = var.rho_i.*lambda + var.C_tilde*z;
            end
        end
        
        % Obtaining v^{k+1}

        % x_0 is unconstrained
        for i = 1:n
            v(i) = min(max(v(i),-options.inf_value),options.inf_value);
        end
        
        % u_0 is hard-constrained in both hard and soft versions of the solver
        for i = n+1:n+m
            v(i) = min(max(v(i),var.LB(i)),var.UB(i));
        end

        if ~options.solver.constrained_output

            if ~options.solver.soft_constraints
                % Rest of v is hard-constrained
                for l = 1:N-1
                    for i = l*(n+m)+1 : (l+1)*(n+m)
                        v(i) = min(max(v(i),var.LB(i-l*(n+m))),var.UB(i-l*(n+m)));
                    end
                end
        
                for i = N*(n+m)+1:N*(n+m)+n
                    v(i) = min(max(v(i),(var.LB(i-N*(n+m))+options.solver.epsilon_x)),(var.UB(i-N*(n+m))-options.solver.epsilon_x));
                end
        
                for i = N*(n+m)+n+1:(N+1)*(n+m)
                    v(i) = min(max(v(i),(var.LB(i-(N*(n+m)))+options.solver.epsilon_u)),(var.UB(i-(N*(n+m)))-options.solver.epsilon_u));
                end

            else % options.solver.soft_constraints == true
                
                % Rest of vector v soft-constrained
                for l = 1:N
                    for i = l*(n+m)+1 : (l+1)*(n+m)
                        
                        if isscalar(var.rho)
                            v1 = v(i) + var.beta_rho_i;
                            v2 = v(i);
                            v3 = v(i) - var.beta_rho_i;
                        else
                            v1 = v(i) + var.beta_rho_i(i);
                            v2 = v(i);
                            v3 = v(i) - var.beta_rho_i(i);
                        end
    
                        if (v1 <= var.LB(i-l*(n+m)))
                            v(i) = v1;
                        elseif (v2 >= var.LB(i-l*(n+m)) && v2 <= var.UB(i-l*(n+m)))
                            v(i) = v2;
                        elseif (v3 >= var.UB(i-l*(n+m)))
                            v(i) = v3;
                        elseif (v2 > var.UB(i-l*(n+m)))
                            v(i) = var.UB(i-l*(n+m));
                        elseif (v2 < var.LB(i-l*(n+m)))
                            v(i) = var.LB(i-l*(n+m));
                        end
    
                    end
                end
    
            end

        else % options.solver.constrained_output == true

            if ~options.solver.soft_constraints
                % Rest of v is hard-constrained
                
                % y_0 is hard-constrained
                for i = n+m+1:n+m+pp
                    v(i) = min(max(v(i),var.LB(i)),var.UB(i));
                end
                
                % From x_1 to y_{N-1} 
                for l = 1:N-1
                    for i = l*(n+m+pp)+1 : (l+1)*(n+m+pp)
                        v(i) = min(max(v(i),var.LB(i-l*(n+m+pp))),var.UB(i-l*(n+m+pp)));
                    end
                end
                
                % From x_s to y_s
                for i = N*(n+m+pp)+1 : N*(n+m+pp)+n
                    v(i) = min(max(v(i),var.LB(i-N*(n+m+pp))+options.solver.epsilon_x),var.UB(i-N*(n+m+pp))-options.solver.epsilon_x);
                end
                for i = N*(n+m+pp)+n+1 : N*(n+m+pp)+n+m
                    v(i) = min(max(v(i),var.LB(i-N*(n+m+pp))+options.solver.epsilon_u),var.UB(i-N*(n+m+pp))-options.solver.epsilon_u);
                end
                for i = N*(n+m+pp)+n+m+1 : (N+1)*(n+m+pp)
                    v(i) = min(max(v(i),var.LB(i-N*(n+m+pp))+options.solver.epsilon_y),var.UB(i-N*(n+m+pp))-options.solver.epsilon_y);
                end
        
            else % options.solver.soft_constraints == true
                
                % y_0 is soft-constrained
                for i = n+m+1:n+m+pp
    
                    if isscalar(var.rho)
                        v1 = v(i) + var.beta_rho_i;
                        v2 = v(i);
                        v3 = v(i) - var.beta_rho_i;
                    else
                        v1 = v(i) + var.beta_rho_i(i);
                        v2 = v(i);
                        v3 = v(i) - var.beta_rho_i(i);
                    end
    
                    if (v1 <= var.LB(i))
                        v(i) = v1;
                    elseif (v2 >= var.LB(i) && v2 <= var.UB(i))
                        v(i) = v2;
                    elseif (v3 >= var.UB(i))
                        v(i) = v3;
                    elseif (v2 > var.UB(i))
                        v(i) = var.UB(i);
                    elseif (v2 < var.LB(i))
                        v(i) = var.LB(i);
                    end
    
                end
                
                % Rest of vector v soft-constrained
                for l = 1:N
                    for i = l*(n+m+pp)+1 : (l+1)*(n+m+pp)
                        
                        if isscalar(var.rho)
                            v1 = v(i) + var.beta_rho_i;
                            v2 = v(i);
                            v3 = v(i) - var.beta_rho_i;
                        else
                            v1 = v(i) + var.beta_rho_i(i);
                            v2 = v(i);
                            v3 = v(i) - var.beta_rho_i(i);
                        end
    
                        if (v1 <= var.LB(i-l*(n+m+pp)))
                            v(i) = v1;
                        elseif (v2 >= var.LB(i-l*(n+m+pp)) && v2 <= var.UB(i-l*(n+m+pp)))
                            v(i) = v2;
                        elseif (v3 >= var.UB(i-l*(n+m+pp)))
                            v(i) = v3;
                        elseif (v2 > var.UB(i-l*(n+m+pp)))
                            v(i) = var.UB(i-l*(n+m+pp));
                        elseif (v2 < var.LB(i-l*(n+m+pp)))
                            v(i) = var.LB(i-l*(n+m+pp));
                        end
    
                    end
                end
    
            end

        end
        
        % Update lambda
        if ~options.solver.constrained_output
            if isscalar(var.rho)
                lambda = lambda + var.rho*(z - v);
            else
                lambda = lambda + var.rho.*(z - v);
            end
        else
            if isscalar(var.rho)
                lambda = lambda + var.rho*(var.C_tilde*z - v);
            else
                lambda = lambda + var.rho.*(var.C_tilde*z - v);
            end
        end

        % Compute residuals
        if ~options.solver.constrained_output
            r_p = norm(z - v,'inf'); % Primal residual
        else
            r_p = norm(var.C_tilde*z - v,'inf'); % Primal residual
        end
        r_d = norm(v - v_old,'inf'); % Dual residual

        % Check exit condition
        if (r_p <= options.solver.tol_p && r_d <= options.solver.tol_d) % Infinity norm
            done = true;
            e_flag = 1;
        elseif (k >= options.solver.k_max)
            done = true;
            e_flag = -1;
        end

        % Update variables and historics
        v_old = v;

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
        u = v(n+1:n+m)./var.scaling_u + var.OpPoint_u;
    else
        u = v(n+1:n+m);
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
