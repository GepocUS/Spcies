%% This is the sparse version of the RMPC solver

function [v_0, k, e_flag, Hist] = ADMM_for_ellipMPC_embedded(var, x0, xr, ur, varargin)
    timer_function = tic;
    
    %% Default values
    def_v_0 = zeros(var.N*(var.n+var.m), 1); % Default initial value for the decision variables v
    def_lambda_0 = zeros(var.N*(var.n+var.m), 1); % Default initial value for the dual variables lamnda
    def_genHist = 0; % Default amount of data generated for Hist
    def_verbose = 1; % Default amount of information displayed: Displayes end of algorithm info only
    def_k_max = 1000; % Maximum number of iterations
    def_tol = 1e-3; % Exit tolerance
    def_in_engineering = false; % Default value of the flag that determined if the input variables are given in engineering units
    def_options = struct('tol', def_tol, 'k_max', def_k_max, 'in_engineering', def_in_engineering);
                     
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
    addOptional(par, 'v_0', def_v_0, @(x) isnumeric(x) && (min(size(x))==1) || isempty(x));
    addOptional(par, 'lambda_0', def_lambda_0, @(x) isnumeric(x) && (min(size(x))==1) || isempty(x));
    % Name-value parameters
    addParameter(par, 'genHist', def_genHist, @(x) isnumeric(x) && (x>=0));
    addParameter(par, 'verbose', def_verbose, @(x) isnumeric(x) && (x>=0));
    addParameter(par, 'options', def_options, @(x) isstruct(x) || isempty(x));
    
    % Parse
    parse(par, var, x0, xr, ur, varargin{:});
    % Rename
    var = par.Results.var;
    x0 = par.Results.x0;
    xr = par.Results.xr;
    ur = par.Results.ur;
    v = par.Results.v_0;
    lambda = par.Results.lambda_0;
    genHist = par.Results.genHist;
    verbose = par.Results.verbose;
    % Check arguments
    if isempty(v); v = def_v_0; end % Default if an empty array is provided
    if isempty(lambda); lambda = def_lambda_0; end % Default if an empty array is provided
    if size(v, 2)>1; v = v'; end % Make a column vector
    if size(lambda, 2)>1; lambda = lambda'; end % Make a column vector
    if genHist>2; genHist = 2; end
    if verbose>3; verbose = 3; end
    
    % Set default options if options is empty
    if isempty(par.Results.options)
        options = def_options;
    else
        options = par.Results.options;
    end
    if ~isfield(options, 'tol'); options.tol = def_tol; end
    if ~isfield(options, 'k_max'); options.k_max = def_k_max; end
    if ~isfield(options, 'in_engineering'); options.in_engineering = def_in_engineering; end
    
    %% Variable declaration
    done = false; % Flag that indicates the end of the algorithm
    k = 0; % Counter for the number of iterations
    
    % Dimensions
    n = var.n;
    m = var.m;
    N = var.N;
    
    % Decision variables
    z_0 = zeros(m, 1);
    z_N = zeros(n, 1);
    z = zeros(n+m, N-1);
    
    z1_0 = zeros(m, 1);
    z1_N = zeros(n, 1);
    z1 = zeros(n, 1);

    v_0 = v(1:m);
    v_N = v(end-n+1:end);
    v = reshape(v(m+1:end-n), n+m, N-1);
    
    lambda_0 = lambda(1:m);
    lambda_N = lambda(end-n+1:end);
    lambda = reshape(lambda(m+1:end-n), n+m, N-1); % Decision variable lambda in matrix form
    
    % Other variables
    aux_N = zeros(n, 1);
    mu = zeros(n, N);
    
    rho_0 = var.rho_0;
    rho = var.rho;
    rho_N = var.rho_N;
    
    AB = var.AB;
    Hi = var.Hi;
    Hi_0 = var.Hi_0;
    Hi_N = var.Hi_N;
    P_half = var.P_half;
    P = var.P;
    
    %% Obtain variables in scaled units
    timer_iter = tic;
    
    if options.in_engineering
        for j = 1:n
            x0(j) = var.scaling_x(j)*( x0(j) - var.OpPoint_x(j) );
            xr(j) = var.scaling_x(j)*( xr(j) - var.OpPoint_x(j) );
        end
        for j = 1:m
            ur(j) = var.scaling_x(j)*( ur(j) - var.OpPoint_x(j) );
        end
    end
            
    
    % Update b with the current state x0
    for j = 1:n
        b(j) = 0;
        for i = 1:n
            b(j) = b(j) -AB(j, i)*x0(i);
        end
    end
    
    % Update q with the current reference
    for j = 1:n
        q(j) = var.Q(j)*xr(j);
        qT(j) = 0;
        for i = 1:n
            qT(j) = qT(j) + var.T(j, i)*xr(i);
        end
    end
    for j = 1:m
        q(j+n) = var.R(j)*ur(j);
    end
    
    %% Iterations
    while ~done
        k = k + 1;
        
        %% Step 0: Save value of z
        
            % Compute the first m elements
        for j = 1:m
            z1_0(j) = z_0(j);
        end
            % Compute all others except for the last n
        for l = 1:N-1
            for j = 1:n+m
                z1(j, l) = z(j, l);
            end
        end
            % Compute the last n
        for j = 1:n
            z1_N(j) = z_N(j);
        end
        
        %% Step 1: Minimize w.r.t z
        
        % Compute q_hat = q + lambda - rho*v
        % I save it in z to same a bit of memory and because I also because z has to be
        % initiallized as q_hat further ahead, thus also saving some computations
        
            % Compute first n
        for j = 1:m
            z_0(j) = q(n+j) + lambda_0(j) - rho_0(j)*v_0(j);
        end
            % Compute all other elements except the last n
        for l = 1:N-1
            for j = 1:n+m
               z(j, l) = q(j) + lambda(j, l) - rho(j, l)*v(j, l);
            end
        end
            % Compute the last n
        for j = 1:n
            z_N(j) = qT(j);
            for i = 1:n
                z_N(j) = z_N(j) + P_half(j, i)*lambda_N(i) - P(j, i)*rho_N(i)*v_N(i);
            end
        end
        
        % Compute r.h.s. of Wc system of equations, i.e., -G'*H_hat^(-1)*q_hat - b
        % I store it in mu to save a bit of memory
        
            % Compute first n
        for j = 1:n
            mu(j, 1) = Hi(j, 1)*z(j, 1) - b(j);
            for i = 1:m
                mu(j, 1) = mu(j, 1) - AB(j, i+n)*Hi_0(i)*z_0(i);
            end
        end
            % Compute all other elements except the last n
        for l = 2:N-1
            for j = 1:n
                mu(j, l) = Hi(j, l)*z(j, l);
                for i = 1:n+m
                    mu(j, l) = mu(j, l) - AB(j, i)*Hi(i, l-1)*z(i, l-1);
                end
            end
        end
            % Compute the last n
        for j = 1:n
            mu(j, N) = 0;   
            for i = 1:n
                mu(j, N) = mu(j, N) + Hi_N(j, i)*z_N(i);
            end
            for i = 1:n+m
                mu(j, N) = mu(j, N) - AB(j, i)*Hi(i, N-1)*z(i, N-1);
            end
        end
        
        %%%%% Forward substitution %%%%%
            % Compute first n
        for j = 1:n
            % Forwards substitution
            for i = 1:j-1
                mu(j,1) = mu(j,1) - var.Beta(i, j, 1)*mu(i,1);
            end
            mu(j,1) = var.Beta(j, j, 1)*mu(j,1); % Divide by diagonal element
        end
            % Compute all other n except for the last n
        for l = 2:N-1
            for j = 1:n
                % Forwards substitution
                for i = 1:n
                    mu(j,l) = mu(j,l) - var.Alpha(i, j, l-1)*mu(i, l-1);
                end
                for i = 1:j-1
                    mu(j,l) = mu(j,l) - var.Beta(i, j, l)*mu(i, l);
                end
                mu(j,l) = var.Beta(j, j, l)*mu(j, l);
            end
        end
            % Compute the last n elements
        for j = 1:n
            % Forwards substitution
            for i = 1:n
                mu(j,N) = mu(j,N) - var.Alpha(i, j, N-1)*mu(i, N-1);
            end
            for i = 1:j-1
                mu(j,N) = mu(j,N) - var.Beta(i, j, N)*mu(i, N);
            end
            mu(j,N) = var.Beta(j, j, N)*mu(j,N);
        end
        
        %%%%% Backward substitution %%%%%
            % Compute last n
        for j = n:-1:1
            for i = n:-1:j+1
                mu(j,N) = mu(j,N) - var.Beta(j, i, N)*mu(i, N);
            end
            mu(j,N) = var.Beta(j, j, N)*mu(j,N);
        end
            % Compute all other n except for the first n
        for l = N-1:-1:2
            for j = n:-1:1
                for i = n:-1:1
                    mu(j,l) = mu(j,l) - var.Alpha(j, i, l)*mu(i, l+1);
                end
                for i = n:-1:j+1
                    mu(j,l) = mu(j,l) - var.Beta(j, i, l)*mu(i, l);
                end
                mu(j,l) = var.Beta(j, j, l)*mu(j,l);
            end
        end
            % Compute the first n elements
        for j = n:-1:1
            for i = n:-1:1
                mu(j,1) = mu(j,1) - var.Alpha(j, i, 1)*mu(i, 2);
            end
            for i = n:-1:j+1
                mu(j,1) = mu(j,1) - var.Beta(j, i, 1)*mu(i, 1);
            end
            mu(j,1) = var.Beta(j, j, 1)*mu(j,1);
        end
        
        % Compute z (note that at this point z = q_hat)
            % Compute the first m
        for j = 1:m
            for i = 1:n
                z_0(j) = z_0(j) + AB(i, j+n)*mu(i, 1);
            end
            z_0(j) = -Hi_0(j)*z_0(j);
        end
            % Compute all others except for the last n
        for l = 1:N-1
            for j = 1:n
                z(j, l) = z(j, l) - mu(j, l);
            end
            for j = 1:n+m
                for i = 1:n
                    z(j, l) = z(j, l) + AB(i, j)*mu(i, l+1);
                end
                z(j, l) = -Hi(j, l)*z(j, l);
            end
        end
            % Compute the last n
        for j = 1:n
            aux_N(j) = z_N(j) - mu(j, N);
        end
        for j = 1:n
            z_N(j) = 0;
            for i = 1:n
                z_N(j) = z_N(j) - Hi_N(j, i)*aux_N(i);
            end
        end
        
        %% Step 2: Minimize w.r.t. v
        
        % First m variables
        for j = 1:m
            v_0(j) = max( min( z_0(j) + var.rho_i_0(j)*lambda_0(j), var.UBu0(j)), var.LBu0(j));
        end
        
        % All the rest except for the last n
        for l = 1:N-1
            for j = 1:n+m
                v(j, l) = max( min( z(j, l) + var.rho_i(j, l)*lambda(j, l), var.UBz(j, l)), var.LBz(j, l));
            end
        end
        
        % Last n elements
        
        % Compute the vector to be projected
        for j = 1:n
            v_N(j) = z_N(j);
            for i = 1:n
                v_N(j) = v_N(j) + var.Pinv_half(j, i)*var.rho_i_N(i)*lambda_N(i);
            end
        end
        
        % Compute (v_N - c)*P*(v_N - c)
        for j = 1:n
            aux_N(j) = 0;
            for i = 1:n
                aux_N(j) = aux_N(j) + P(j, i)*(v_N(i) - var.c(i));
            end
        end
        vPv = 0;
        for j = 1:n
            vPv = vPv + (v_N(j) - var.c(j))*aux_N(j);
        end
        
        % If v_N does not belong to the ellipsoid I need to perform the projection step
        if vPv > var.r^2
            vPv = var.r/sqrt(vPv);
            for j = 1:n
                v_N(j) = vPv*(v_N(j) - var.c(j)) + var.c(j);
            end
        end
        
        %% Step 3: Update lambda
        
            % Compute the first n elements
        for j = 1:m
            lambda_0(j) = lambda_0(j) + rho_0(j)*(z_0(j) - v_0(j));
        end
            % Compute all others except for the last n
        for l = 1:N-1
            for j = 1:n+m
                lambda(j, l) = lambda(j, l) + rho(j, l)*(z(j, l) - v(j, l));
            end
        end
            % Compute the last n
        for j = 1:n
            aux_N(j) = rho_N(j)*(z_N(j) - v_N(j));
        end
        for j = 1:n
            for i = 1:n
                lambda_N(j) = lambda_N(j) + P_half(j, i)*aux_N(i);
            end
        end

        %% Step 4: Compute residual
        
        res_flag = false; % Flag used to determine if the residuals are satisfied
        res_fixed_point = 0; % Residual for fixed point
        res_primal_feas = 0; % Residual for primal feasibility
        
        % Computer the first n elements
        for j = 1:m
            res_fixed_point = abs(z1_0(j) - z_0(j));
            res_primal_feas = abs(z_0(j) - v_0(j));
            if res_fixed_point > options.tol || res_primal_feas > options.tol
                res_flag = 1;
                break;
            end
        end
        
        % Compute the last n elements
        if ~res_flag
            for j = 1:n
                res_fixed_point = abs(z1_N(j) - z_N(j));
                res_primal_feas = abs(z_N(j) - v_N(j));
                if res_fixed_point > options.tol || res_primal_feas > options.tol
                    res_flag = 1;
                    break;
                end
            end
        end
        
        % Compute all the other elements
        if ~res_flag
            for l = 1:N-1
                for j = 1:n+m 
                    res_fixed_point = abs(z1(j, l) - z(j, l));
                    res_primal_feas = abs(z(j, l) - v(j, l));
                    if res_fixed_point > options.tol || res_primal_feas > options.tol
                        res_flag = 1;
                        break;
                    end
                end
                if res_flag
                    break;
                end
            end
        end
                
        %% Step 5: Exit condition
        if ~res_flag
            done = true;
            e_flag = 1;
        elseif k >= options.k_max
            done = true;
            e_flag = -1;
        end
        
    end
    
    %% Extract variables
    time = toc(timer_function);
    
    Hist.z = [z_0; z(:); z_N];
    Hist.v = [v_0; v(:); v_N];
    Hist.lambda = [lambda_0; lambda(:); lambda_N];
    Hist.time = time;

end
