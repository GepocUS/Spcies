%% compute_MPCT_EADMM_ingredients
%
% Computes the ingredients for the MPCT controller solved using the EADMM algorithm
%
% Information about this formulaiton and the solver  can be found at:
% 
% "Implementation of model predictive control for tracking in embedded systems
% using a sparse extended ADMM algorithm", by P. Krupa, I. Alvarado, D. Limon
% and T. Alamo, arXiv preprint: 2008:09071v2, 2020.
% 
% INPUTS:
%   - controller: Contains the information of the controller.
%   - options: Structure containing options of the EADMM solver.
%   - spcies_options: Structure containing the options of the toolbox.
% 
% OUTPUTS:
%   - vars: Structure containing the ingredients required by the solver.
%   - vars_nonsparse: Structure containing the ingredients for the non-sparse solver.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function [vars, vars_nonsparse] = compute_MPCT_EADMM_ingredients(controller, options, spcies_options)

    %% Extract from controller
    if isa(controller, 'TrackingMPC')
        A = controller.model.A;
        B = controller.model.Bu;
        n = controller.model.n_x;
        m = controller.model.n_u;
        N = controller.N;
        Q = controller.Q;
        R = controller.R;
        T = controller.T;
        S = controller.S;
        LBx = controller.model.LBx;
        LBu = controller.model.LBu;
        UBx = controller.model.UBx;
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
        T = controller.param.T;
        S = controller.param.S;
        try
            LBx = controller.sys.LBx;
        catch
            LBx = -options.inf_bound*ones(n, 1);
        end
        try
            UBx = controller.sys.UBx;
        catch
            UBx = options.inf_bound*ones(n, 1);
        end
        try
            LBu = controller.sys.LBu;
        catch
            LBu = -options.inf_bound*ones(m, 1);
        end
        try
            UBu = controller.sys.UBu;
        catch
            UBu = options.inf_bound*ones(m, 1);
        end
    end

    %% Extract or compute rho
    if isfield(options, 'rho')
        options.rho_base = options.rho;
        options.rho_mult = 1;
    end

    rho_base = options.rho_base;
    rho_mult = options.rho_mult;
    rho = rho_base*ones((N+1)*(n+m) + n + 1*(n+m), 1);
    % Penalize constraints related to x
    rho(1:n) = 1*rho_mult*rho_base; % Initial constraint: x_0 = x. (6b)
    rho(n+1:2*n) = 1*rho_mult*rho_base; % Initial z1 + z2 + z3 = 0. (6i) for i = 0
    rho(end-2*(n+m)+1:end-n-m-m) = 1*rho_mult*rho_base; % Final  z1 + z2 + z3 = 0. (6i)  for i = N
    rho(end-n-m+1:end-m) = 1*rho_mult*rho_base; % x_N = x_s. (6k)
    % Penalize constraints related to u
    rho(end-2*(n+m)+n+1:end-n-m) = rho_mult*rho_base; % Final  z1 + z2 + z3 = 0. (6j)  for i = N
    rho(end-m+1:end) = rho_mult*rho_base; % u_N = u_s. (6l) 
    
    %% Matrices A1, A2 and A3
    
    % A1 : z2 = (xi, ui)
    A1 = -[ [-eye(n) zeros(n, N*(n+m)+m)]; eye((N+1)*(n+m)); [zeros(n+m, N*(n+m)) eye(n+m)]];
    
    % A2 : z1 = (xs, us)
    A2 = [zeros(n, n+m); kron(ones(N,1), eye(n+m)); kron(ones(2,1), eye(n+m))];

    % A3 : z3 = (hat_xi, hat_ui)
    A3 = [zeros(n, (N+1)*(n+m)); eye((N+1)*(n+m)); zeros(n+m, (N+1)*(n+m))];
    
    % b
    b = zeros((N+1)*(n+m) + n + 1*(n+m), 1);
    
    %% Problem 1: z1 = (xi, ui)
    
    % QP1 matrices
    H1 = zeros((N+1)*(n+m)) + (rho.*A1)'*A1;
    
    % Ingredients with minimal memory consumption
    H1i = 1./diag(H1);
    
    %% Problem 2: z2 = (xs, us)
    H2 = blkdiag(T, S) + (rho.*A2)'*A2;
    Az2 = [(A - eye(n)) B];

    
    % Matrices for explicit solution
    H2i = inv(H2);
    W2 = H2i*Az2'*inv(Az2*H2i*Az2')*Az2*H2i - H2i;
    
    %% Problem 3: z3 = (hat_xi, hat_ui)
    H3 = kron(eye(N+1), blkdiag(Q, R)) + (rho.*A3)'*A3;

    Az3 = kron(eye(N), [A B]); % Diagonal de la matriz
    j = 0;
    for i=1:n:n*N-n % Inserto las matrices -I en Az
        j = j+1;
        Az3(i:i+n-1,((j-1)*(n+m)+(m+n+1)):((j-1)*(n+m)+(n+m+1)+n-1)) = -eye(n);
    end
    Az3 = [Az3 [zeros((N-1)*n, n); -eye(n)] zeros(N*n, m)];
    
    % Matrices for explicit solution
    W3 = Az3*inv(H3)*Az3';
    W3c = chol(W3);
    
    % Save H3i to optimize algorithm efficiency 

    % Detect if matrices Q and R are diagonal
    if options.diag_QR && (isdiag(Q) && isdiag(R))
        options.diag_QR = true;
    else
        options.diag_QR = false;
    end

    if options.diag_QR == true
        H3i = 1./diag(H3);
    else
        Q_mult_inv = inv(Q + rho_mult*rho_base*eye(n));
        R_mult_inv = inv(R + rho_mult*rho_base*eye(m));
        Q_base_inv = inv(Q + rho_base*eye(n));
        R_base_inv = inv(R + rho_base*eye(m));
    end
       
    %% Warmstart
    
    % Dimensions
    dim_z1 = size(H1, 1);
    dim_z2 = size(H2, 1);
    dim_z3 = size(H3, 1);
    dim_l = size(A1, 1);
    dim_w = dim_z1 + dim_z2 + dim_z3 + dim_l;
    
    % Equality constraints
    Aeq = [A1 A2 A3];
    
    % The cost function that would result from grouping z = [z1; z2; z3; lambda] only due to the \theta_i functions
    H = blkdiag(zeros(dim_z1), blkdiag(T, S), kron(eye(N+1), blkdiag(Q, R)), zeros(dim_l));
    %q = [zeros(dim_z1, 1); [-T*xr; -S*ur]; zeros(dim_z3 + dim_l)];
    
    S_l = zeros(dim_l, dim_w); % Selects lambda from w, i.e. lambda = S_l*w
    S_l(:, dim_z1 + dim_z2 + dim_z3 + (1:dim_l)) = eye(dim_l);
    
    S_z = zeros(dim_z1 + dim_z2 + dim_z3, dim_w);
    S_z(:, 1:dim_z1 + dim_z2 + dim_z3) = eye(dim_z1 + dim_z2 + dim_z3);
    
    % Matrix B such that b = B*x
    Bz = zeros(dim_l, n);
    Bz(1:n, 1:n) = eye(n);
    
    % Matrices of the partial derivatives
    H_hat = 0.5*H - S_l'*Aeq*S_z;
    H_hat_inv = inv(H_hat + H_hat');
    
    % Prediction step matrix. w_{k+1} = w - Ln*(x_{k+1} - x_k)
    L = H_hat_inv*S_l'*Bz;
    
    % Extract from the matrix Ln
    L_z2 = L(dim_z1 + (1:dim_z2),:);
    L_z3 = L(dim_z1 + dim_z2 + (1:dim_z3), :);
    L_l = L(dim_z1 + dim_z2 + dim_z3 + (1:dim_l), :);
    
    %% Construct str
    vars.N = N; % Prediction horizon
    vars.n = n; % Dimension of state space
    vars.m = m; % Dimension of input space
    vars.H1i = reshape(H1i, m+n, N+1)'; % Inverse of diagonal of H1 in matrix form
    if options.diag_QR == true
        vars.H3i = reshape(H3i, m+n, N+1)'; % Inverse of diagonal of H3 in matrix form
    else
        vars.Q_mult_inv = Q_mult_inv;
        vars.Q_base_inv = Q_base_inv;
        vars.R_mult_inv = R_mult_inv;
        vars.R_base_inv = R_base_inv;
        vars.AB_base_inv = [A B]*blkdiag(Q_base_inv, R_base_inv);
        vars.AB_mult_inv = [A B]*blkdiag(Q_mult_inv, R_base_inv);
    end

    vars.AB = [A B]; % Matrices of the system model
    vars.W2 = W2;
    vars.T = -T;
    vars.S = -S;
    vars.LB = [LBx; LBu];
    vars.LB(isinf(vars.LB)) = -options.inf_bound;
    vars.UB = [UBx; UBu];
    vars.UB(isinf(vars.UB)) = options.inf_bound;
    vars.LBs = [LBx + options.epsilon_x*ones(n, 1); LBu + options.epsilon_u*ones(m, 1)];
    vars.LBs(isinf(vars.LBs)) = -options.inf_bound;
    vars.UBs = [UBx - options.epsilon_x*ones(n, 1); UBu - options.epsilon_u*ones(m, 1)];
    vars.UBs(isinf(vars.UBs)) = options.inf_bound;
    vars.LB0 = [-options.inf_bound*ones(n,1); LBu];
    vars.UB0 = [options.inf_bound*ones(n,1); UBu];
    
    % Penalty parameter
    vars.rho = reshape(rho(n+1:end-n-m), m+n, N+1)';
    vars.rho_0 = [rho(1:n); zeros(m,1)];
    vars.rho_s = rho(end-n-m+1:end);
    
    % Warmstart
    vars.L_z2 = L_z2;
    vars.L_z3 = L_z3(1:n, :);
    vars.L_l = L_l(1:2*n, :); 
    
    % Scaling vectors
    if isa(controller, 'TrackingMPC')
        vars.scaling_x = controller.model.Nx;
        vars.scaling_u = controller.model.Nu;
        vars.scaling_i_u = 1./controller.model.Nu;
        vars.OpPoint_x = controller.model.x0;
        vars.OpPoint_u = controller.model.u0;
    else
        if isfield(controller.sys, 'Nx')
            vars.scaling_x = controller.sys.Nx;
        else
            vars.scaling_x = ones(n, 1);
        end
        if isfield(controller.sys, 'Nu')
            vars.scaling_u = controller.sys.Nu;
        else
            vars.scaling_u = ones(m, 1);
        end
        if isfield(controller.sys, 'Nu')
            vars.scaling_i_u = 1./controller.sys.Nu;
        else
            vars.scaling_i_u = ones(m, 1);
        end
        if isfield(controller.sys, 'x0')
            vars.OpPoint_x = controller.sys.x0;
        else
            vars.OpPoint_x = zeros(n, 1);
        end
        if isfield(controller.sys, 'u0')
            vars.OpPoint_u = controller.sys.u0;
        else
            vars.OpPoint_u = zeros(m, 1);
        end
    end
    
    % Alpha and Beta
    vars.Beta = zeros(n,n,N);
    vars.Alpha = zeros(n,n,N-1);
    for i = 1:N
        vars.Beta(:,:,i) = W3c((i-1)*n+(1:n),(i-1)*n+(1:n));
        for j = 1:n
            vars.Beta(j,j,i) = 1/vars.Beta(j,j,i);
        end
    end
    for i = 1:N-1
        vars.Alpha(:,:,i) = W3c((i-1)*n+(1:n),i*n+(1:n));
    end
    
    %% Compute non-sparse variables (used by the non-sparse solver)
    vars_nonsparse.N = N;
    vars_nonsparse.n = n;
    vars_nonsparse.m = m;
    vars_nonsparse.A1 = A1;
    vars_nonsparse.A2 = A2;
    vars_nonsparse.A3 = A3;
    vars_nonsparse.rho = rho;
    
    % For P1
    vars_nonsparse.H1i = H1i; % Inverse of the diagonal of H_1
    vars_nonsparse.LB = [vars.LB0; kron(ones(N-1, 1), vars.LB); vars.LBs]; % Lower bounds
    vars_nonsparse.UB = [vars.UB0; kron(ones(N-1, 1), vars.UB); vars.UBs]; % Upper bounds
    vars_nonsparse.b = b;
    
    % For P2
    vars_nonsparse.T = T;
    vars_nonsparse.S = S;
    vars_nonsparse.W2 = W2;
    
    % For P3
    vars_nonsparse.H3inv = inv(H3);
    vars_nonsparse.Az3 = Az3;
    vars_nonsparse.W3 = W3;
    
    % Scaling
    vars_nonsparse.scaling_x = vars.scaling_x ;
    vars_nonsparse.scaling_u = vars.scaling_u ;
    vars_nonsparse.OpPoint_x = vars.OpPoint_x;
    vars_nonsparse.OpPoint_u = vars.OpPoint_u;
    
end

