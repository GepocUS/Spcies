%% EADMM_for_MPCT_embedded - Efficient version of the EADMM solver for MPCT
% This solver contains the near-to-exact algorithm that would be run in an embedded system
% The only difference is that it collects historics and times
% 
% Inputs:
%   - var: Structure containing the necessary variables. It is computed using Spcies_gen_controller for the 'MPCT_EADMM' type and 'Matlab' target.
%   - x0: Current system state.
%   - xr, ur: State and input reference, respectively.
% 
% Optional inputs
%   - z1_0, z2_0, z3_0, lambda_0: Initial values of decision variables z1, z2, z3 and lambda, respectively. (zeros)
%   - k_max: Maximum number of iterations of the algorithm. (1000)
%   - tol: Exit tolerance. (1e-3)
%   - warmstart: Boolean that indicates if the warmstart procedure is used (requires setting inpout x_ant). (false)
%   - x_ant: Value of the system state at the previous sample time. Needed for the warmstart procedure. (zeros)
%   - in_engineering: Boolean that indicates if the inputs (x0, xr, ur) and the output u are in engineering or incremental units. (false - in incremental)
%
% Outputs:
%   - u_opt: Control action to be applied to the system (in engineering or incremental units depending value of in_engineering)
%   - opt_var: Structure containing the optimal values of z1, z2, z3 and lambda.
%   - e_flag: Flag indicating the exit condition of the algorithm. 1: Optimim found. -1: Maximum number of iterations attained.
%   - Hist: Structure containing a variety of information and historics.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function [u_opt, opt_var, e_flag, Hist] = EADMM_for_MPCT_embedded(var, x0, xr, ur, varargin)
    timer_function = tic;
    
    %% Default values
    def_z1_0 = zeros((var.N+1)*(var.n+var.m), 1); % z1 = (xi, ui)
    def_z2_0 = zeros(var.n+var.m, 1); % z2 = (xs, us)
    def_z3_0 = zeros((var.N+1)*(var.n+var.m), 1); % z3 = (hat_xi, hat_ui)
    def_lambda_0 = zeros((var.N+1)*(var.n+var.m) + 2*var.n + var.m, 1); % Dual variables
    def_k_max = 1000; % Maximum number of iterations
    def_tol = 1e-3; % Exit tolerance
    def_genHist = 0; % Default amount of data generated for Hist
    def_verbose = 1; % Default amount of information displayed: Displayes end of algorithm info only
    def_warmstart = 0; % Default value of the warmstar flag (true: perform warmstart, false: do not perform warmstart)
    def_x_ant = x0; % Default value of the system state at the previous sample time
    def_in_engineering = 0; % Default vale for flag indicating if the inputs and outputs are in engineering units
    
    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'EADMM_for_MPCT_embedded';
    % Required
    addRequired(par, 'var', @(x) isstruct(x));
    addRequired(par, 'x0', @(x) isnumeric(x) && (min(size(x))==1) || isempty(x));
    addRequired(par, 'xr', @(x) isnumeric(x) && (min(size(x))==1) || isempty(x));
    addRequired(par, 'ur', @(x) isnumeric(x) && (min(size(x))==1) || isempty(x));
    % Optional
    addOptional(par, 'z1_0', def_z1_0, @(x) isnumeric(x) && (min(size(x))==1) || isempty(x));
    addOptional(par, 'z2_0', def_z2_0, @(x) isnumeric(x) && (min(size(x))==1) || isempty(x));
    addOptional(par, 'z3_0', def_z3_0, @(x) isnumeric(x) && (min(size(x))==1) || isempty(x));
    addOptional(par, 'lambda_0', def_lambda_0, @(x) isnumeric(x) && (min(size(x))==1) || isempty(x));
    % Name-value parameters
    addParameter(par, 'k_max', def_k_max, @(x) isnumeric(x) && (x>0) && x==floor(x));
    addParameter(par, 'tol', def_tol, @(x) isnumeric(x) && (x>=0));
    addParameter(par, 'genHist', def_genHist, @(x) isnumeric(x) && (x>=0));
    addParameter(par, 'verbose', def_verbose, @(x) isnumeric(x) && (x>=0));
    addParameter(par, 'warmstart', def_warmstart, @(x) islogical(x) || x==1 || x==0);
    addParameter(par, 'x_ant', def_x_ant, @(x) isnumeric(x) && (min(size(x))==1) || isempty(x));
    addParameter(par, 'in_engineering', def_in_engineering, @(x) islogical(x) || x==1 || x==0);
    % Parse
    parse(par, var, x0, xr, ur, varargin{:});
    % Rename
    var = par.Results.var;
    x0 = par.Results.x0;
    xr = par.Results.xr;
    ur = par.Results.ur;
    z1 = par.Results.z1_0;
    z2 = par.Results.z2_0;
    z3 = par.Results.z3_0;
    lambda = par.Results.lambda_0;
    k_max = par.Results.k_max;
    tol = par.Results.tol;
    genHist = par.Results.genHist;
    verbose = par.Results.verbose;
    warmstart = par.Results.warmstart;
    x_ant = par.Results.x_ant;
    in_engineering = par.Results.in_engineering;
    % Check arguments
    if isempty(z1); z1 = def_z1_0; end % Default if an empty array is provided
    if isempty(z2); z2 = def_z2_0; end % Default if an empty array is provided
    if isempty(z3); z3 = def_z3_0; end % Default if an empty array is provided
    if isempty(lambda); lambda = def_lambda_0; end % Default if an empty array is provided
    if isempty(x_ant); x_ant = def_x_ant; end % Default if an empty array is provided
    if size(z1, 2)>1; z1 = z1'; end % Make a column vector
    if size(z2, 2)>1; z2 = z2'; end % Make a column vector
    if size(z3, 2)>1; z3 = z3'; end % Make a column vector
    if size(z3, 2)>1; z3 = z3'; end % Make a column vector
    if size(lambda, 2)>1; lambda = lambda'; end % Make a column vector
    if size(x_ant, 2)>1; x_ant = x_ant'; end % Make a column vector
    if genHist>2; genHist = 2; end
    if verbose>3; verbose = 3; end
    
    %% Variable declaration
    done = false; % Flag that indicates the end of the algorithm
    k = 0; % Counter for the number of iterations
    
    % Times
    time_QP1 = zeros(1, k_max);
    time_QP2 = zeros(1, k_max);
    time_QP3 = zeros(1, k_max);
    time_res = zeros(1, k_max);
    time_lambda = zeros(1, k_max);
    time_exit = zeros(1, k_max);
    time_iter = zeros(1, k_max);
    
    % Dimensions
    n = var.n;
    m = var.m;
    N = var.N;
    
    % Variables
    z1 = reshape(z1, m+n, N+1); % Decision variable z1 in matrix form
    z3 = reshape(z3, m+n, N+1); % Decision variable z3 in matrix form
    lambda = reshape([lambda(1:n); zeros(m, 1); lambda(n+1:end)], m+n, N+3); % Dual variables in matrix form
    res = zeros(n+m, N+3); % Residual Gamma for the equality constraints between z1, z2 and z3
    rho = var.rho; % Penalty parameter for predited horizon constraints
    rho_s = var.rho_s; % Penalty parameter for initial condition
    rho_0 = var.rho_0; % Penalty parameter for terminal condition
    H1i = var.H1i; % Inverse of diagonal of H1 in matrix form
    H3i = var.H3i; % Inverse of diagonal of H3 in matrix form
    x0 = [x0; zeros(m, 1)]; % Initial condition (with m zeros appended to it)
    x_ant = [x_ant; zeros(m, 1)]; % State at the previous sample time (with m zeros appended to it)
    AB = var.AB; % Matrices of the system model
    ref = [xr; ur]; % Reference
    q2 = zeros(n+m, 1); % Vector q2
    q3 = zeros(n+m, N+1); % Vector q3 in matrix form
    mu3 = zeros(n, N); % Vector mu3 in matrix form
    u_opt = zeros(m, 1); % Control action to be applied
    res_1 = zeros(1, k_max); % One norm of the residual
    x_dif = zeros(n, 1); % Difference between the current state and the state at the previous sample time
    
    %% Scale variables
    if in_engineering
        
        for i = 1:n+m
            ref(i) = var.scaling(i)*(ref(i) - var.OpPoint(i));
        end
        for i = 1:n
            x0(i) = var.scaling(i)*(x0(i) - OpPoint(i));
            x_ant(i) = scaling(i)*(x_ant(i) - OpPoint(i));
        end
        
    end
    
    %% Warmstart
    if warmstart
        
        % Compute diff vector
        for i = 1:n
            x_dif(i) = x0(i) - x_ant(i);
        end
        
        % z2
        for i =  1:n+m
            for j = 1:n
                z2(i) = z2(i) - var.L_z2(i, j)*x_dif(j);
            end
        end
        
        % z3
        for i = 1:n
            for j = 1:n
                z3(i) = z3(i) - var.L_z3(i, j)*x_dif(j);
            end
        end
        
        % lambda
        for i = 1:2*n
            for j = 1:n
                lambda(i) = lambda(i) - var.L_l(i, j)*x_dif(j);
            end
        end   
        
    end
    
    %% Iterations
    while ~done
        timer_iter = tic;
        k = k + 1;
        
        %% Problem 1: Minimize w.r.t. z1 = (xi, ui)
        timer_QP1 = tic;
        
        for i = 1:n+m % First n+m elements
            z1(i, 1) = max( min( (rho(i,1)*(z3(i, 1) + z2(i)) - rho_0(i)*x0(i) + lambda(i, 2) - lambda(i, 1) )*H1i(i, 1), var.UB0(i)), var.LB0(i));
        end
        for j = 2:N % All other elements except for (x_N, u_N)
            for i = 1:n+m
                z1(i, j) = max( min( (rho(i, j)*(z2(i) + z3(i, j)) + lambda(i, j+1) )*H1i(i, j), var.UB(i)), var.LB(i));
            end
        end
        for i = 1:n+m % Last n+m elements
            z1(i, end) = max( min( (rho(i, end)*z3(i, end)  + (rho(i, end) +  rho_s(i))*z2(i) + lambda(i, end-1) + lambda(i, end) )*H1i(i, end), var.UBs(i)), var.LBs(i));
        end
        
        time_QP1(k) = toc(timer_QP1);
        
        %% Problem 2: Minimize w.r.t. z2 = (xs, us)
        % TODO: Take care with the numerical conditionuing of this problem. Maybe we could divide it all by the maximum number in rho.
        timer_QP2 = tic;
        
        % Compute q2
        for i = 1:n+m
            q2(i) = rho(i, end)*z3(i, end) - (rho(i, end) + rho_s(i))*z1(i, end) + lambda(i, N+2) + lambda(i, N+3);
        end
        for i = 1:n
            for j = 1:n
                q2(i) = q2(i) + var.T(i, j)*ref(j);
            end
        end
        for i = n+1:n+m
            for j = 1:m
                q2(i) = q2(i) + var.S(i-n, j)*ref(j+n);
            end
        end
        for j = 1:N
            for i = 1:n+m
                q2(i) = q2(i) + rho(i, j)*(z3(i, j) - z1(i, j)) + lambda(i, j+1);
            end
        end
        
        % Compute z2
        for i = 1:n+m
            z2(i) = 0;
            for j = 1:n+m
                z2(i) = z2(i) + var.W2(i, j)*q2(j);
            end
        end
        
        time_QP2(k) = toc(timer_QP2);
        
        %% Problem 3: Minimize w.r.t. z3 = (hat_xi, hat_ui)
        timer_QP3 = tic;
        
        % Compute q3 (this could be  done with less loops. I keep it like this for the rho in three vectors possible change)
        for i = 1:n+m
            q3(i, 1) = rho(i, 1)*(z2(i) - z1(i, 1)) + lambda(i, 2);
        end
        for j = 2:N
            for i = 1:n+m
                q3(i, j) = rho(i, j)*(z2(i) - z1(i, j)) + lambda(i, j+1);
            end
        end
        for i = 1:n+m
            q3(i, end) = rho(i, end)*(z2(i) - z1(i, end)) + lambda(i, end-1);
        end
        
        % Compute mu: Sparce algorithm for solving W_H-systems using Cholesky factorization
        
        %%%%% Forward substitution %%%%%
            % Compute first n
        for j = 1:n
            % Compute c
            mu3(j,1) = H3i(j, 2)*q3(j, 2);
            for i = 1:n+m
                mu3(j,1) = mu3(j,1) - AB(j,i)*H3i(i, 1)*q3(i, 1);
            end
            % Forwards substitution
            for i = 1:j-1
                mu3(j,1) = mu3(j,1) - var.Beta(i, j, 1)*mu3(i,1);
            end
            mu3(j,1) = var.Beta(j, j, 1)*mu3(j,1); % Divide by diagonal element
        end
            % Compute all other n except for the last n
        for l = 2:N-1
            for j = 1:n
                % Compute c
                mu3(j,l) = H3i(j, l+1)*q3(j, l+1);
                for i = 1:n+m
                    mu3(j,l) = mu3(j,l) - AB(j,i)*H3i(i,l)*q3(i,l);
                end
                % Forwards substitution
                for i = 1:n
                    mu3(j,l) = mu3(j,l) - var.Alpha(i, j, l-1)*mu3(i, l-1);
                end
                for i = 1:j-1
                    mu3(j,l) = mu3(j,l) - var.Beta(i, j, l)*mu3(i, l);
                end
                mu3(j,l) = var.Beta(j, j, l)*mu3(j, l);
            end
        end
            % Compute the last n elements
        for j = 1:n
            % Compute c
            mu3(j,N) = H3i(j, N+1)*q3(j, N+1);
            for i = 1:n+m
                mu3(j, N) = mu3(j, N) - AB(j,i)*H3i(i,N)*q3(i,N);
            end
            % Forwards substitution
            for i = 1:n
                mu3(j,N) = mu3(j,N) - var.Alpha(i, j, N-1)*mu3(i, N-1);
            end
            for i = 1:j-1
                mu3(j,N) = mu3(j,N) - var.Beta(i, j, N)*mu3(i, N);
            end
            mu3(j,N) = var.Beta(j, j, N)*mu3(j,N);
        end
        
        %%%%% Backward substitution %%%%%
            % Compute last n
        for j = n:-1:1
            for i = n:-1:j+1
                mu3(j,N) = mu3(j,N) - var.Beta(j, i, N)*mu3(i, N);
            end
            mu3(j,N) = var.Beta(j, j, N)*mu3(j,N);
        end
            % Compute all other n except for the first n
        for l = N-1:-1:2
            for j = n:-1:1
                for i = n:-1:1
                    mu3(j,l) = mu3(j,l) - var.Alpha(j, i, l)*mu3(i, l+1);
                end
                for i = n:-1:j+1
                    mu3(j,l) = mu3(j,l) - var.Beta(j, i, l)*mu3(i, l);
                end
                mu3(j,l) = var.Beta(j, j, l)*mu3(j,l);
            end
        end
            % Compute the first n elements
        for j = n:-1:1
            for i = n:-1:1
                mu3(j,1) = mu3(j,1) - var.Alpha(j, i, 1)*mu3(i, 2);
            end
            for i = n:-1:j+1
                mu3(j,1) = mu3(j,1) - var.Beta(j, i, 1)*mu3(i, 1);
            end
            mu3(j,1) = var.Beta(j, j, 1)*mu3(j,1);
        end
        
            
        % Compute z3
            % Compute first n+m elements
        for j = 1:n+m
            z3(j,1) = q3(j, 1);
            for i = 1:n
                z3(j,1) = z3(j,1) + AB(i, j)*mu3(i,1);
            end
            z3(j,1) = -H3i(j,1)*z3(j,1);
        end
            % Compute all other elements except the last n+m
        for l = 2:N
            for j = 1:n
                z3(j,l) = q3(j, l) - mu3(j, l-1);
            end
            for j = n+1:n+m
                z3(j,l) = q3(j, l);
            end
            for j = 1:n+m
                for i = 1:n
                    z3(j,l) = z3(j,l) + AB(i, j)*mu3(i,l);
                end
                z3(j,l) = -H3i(j,l)*z3(j, l);
            end           
        end
            % Compute last n+m
        for j = 1:n
            z3(j,N+1) = H3i(j,N+1)*(mu3(j,N) - q3(j, N+1));
        end
        for j = n+1:n+m
            z3(j,N+1) = -H3i(j,N+1)*q3(j, N+1);
        end
        
        time_QP3(k) = toc(timer_QP3);
        
        %% Compute res
        timer_res = tic;
        
        for j = 1:n
            res(j,1) = z1(j, 1) - x0(j);
        end
        for l = 1:N+1
            for j = 1:n+m
                res(j, l+1) = z2(j) + z3(j, l) - z1(j, l);
            end
        end
        for j = 1:n+m
            res(j, N+3) = z2(j) - z1(j, N+1);
        end
        
        time_res(k) = toc(timer_res);
        
        %% Update lambda
        timer_lambda = tic;

        for j = 1:n
            lambda(j, 1) = lambda(j, 1) + rho_0(j)*res(j,1);
        end
        for l = 2:N+2
            for j = 1:n+m
                lambda(j, l) = lambda(j, l) + rho(j, l-1)*res(j, l);
            end
        end
        for j = 1:n+m
            lambda(j, N+3) = lambda(j, N+3) + rho_s(j)*res(j, N+3);
        end
        
        time_lambda(k) = toc(timer_lambda);
        
        %% Exit condition
        timer_exit = tic;
        
        % Compute res_1
        res_1(k) = 0;
        for l = 1:N+3
            for j = 1:n+m
                res_1(k) = max(res_1(k), abs(res(j, l)));
            end
        end
        
        if res_1(k) <= tol
            done = true;
            e_flag = 1;
        elseif k >= k_max
            done = true;
            e_flag = -1;
        end
        
        if done
            if in_engineering
                for i = 1:m
                    u_opt(i) = z1(n+i, 1)*var.scaling_inv_u(i) + var.OpPoint(n+i);
                end
            else
                for i = 1:m
                    u_opt(i) = z1(n+i, 1);
                end
            end
        end
            
        time_exit(k) = toc(timer_exit);
        
        time_iter(k) = toc(timer_iter);
        
    end
    
    %% Construct and return results
    
    % Times
    time = struct;
    time.function = toc(timer_function);
    time.QP1 = time_QP1(1:k);
    time.QP2 = time_QP2(1:k);
    time.QP3 = time_QP3(1:k);
    time.res = time_res(1:k);
    time.lambda = time_lambda(1:k);
    time.exit = time_exit(1:k);
    time.iter = time_iter(1:k);
    
    % Historics
    Hist.res_1 = res_1(1:k);
    Hist.k = k;
    Hist.time = time;
    Hist.mu3 = mu3;
    Hist.q3 = q3;
    Hist.q2 = q2;
    Hist.res = res;
    
    % Optimal values of decision variables
    opt_var.z1 = z1(:);
    opt_var.z2 = z2;
    opt_var.z3 = z3(:);
    opt_var.res = res(:);
    opt_var.res = [opt_var.res(1:n); opt_var.res(n+m+1:end)];
    opt_var.lambda = lambda(:);
    opt_var.lambda = [opt_var.lambda(1:n); opt_var.lambda(n+m+1:end)];
    
end
