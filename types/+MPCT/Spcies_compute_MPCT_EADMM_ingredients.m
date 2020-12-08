%% Spcies_compute_MPCT_EADMM_ingredients
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
%   - options: structure containing options of the EADMM solver.
% 
% OUTPUTS:
%   - vars: Structure containing the ingredients required by the solver.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

% Author: Pablo Krupa (pkrupa@us.es)
% 
% Changelog: 
%   v0.1 (2020/09/03): Initial commit version
%   v0.2 (2020/09/17): Added documentation
%   v0.2 (2020/12/08): Added parser and improved overall usability.
%

function vars = Spcies_compute_MPCT_EADMM_ingredients(controller, options)

    %% Extract from controller
    if isa(controller, 'TeackingMPC')
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
        if isfield(controller.sys, 'LBx')
            LBx = controller.sys.LBx;
        else
            LBx = -options.inf_bound;
        end
        if isfield(controller.sys, 'UBx')
            UBx = controller.sys.UBx;
        else
            UBx = options.inf_bound;
        end
        if isfield(controller.sys, 'LBu')
            LBu = controller.sys.LBu;
        else
            LBu = -options.inf_bound;
        end
        if isfield(controller.sys, 'UBu')
            UBu = controller.sys.UBu;
        else
            UBu = options.inf_bound;
        end
    end
    
    %% Extract or compute rho
    if isfield(options, 'rho')
        rho = options.rho;
    else
        rho_base = options.rho_base;
        rho_mult = options.rho_mult;
        rho = rho_base*ones((param.N+1)*(n+m) + n + 1*(n+m), 1);
        % Penalize constraints related to x
        rho(1:n) = 1*rho_mult*rho_base; % Initial constraint: x_0 = x. (6b)
        rho(n+1:2*n) = 1*rho_mult*rho_base; % Initial z1 + z2 + z3 = 0. (6i) for i = 0
        rho(end-2*(n+m)+1:end-n-m-m) = 1*rho_mult*rho_base; % Final  z1 + z2 + z3 = 0. (6i)  for i = N
        rho(end-n-m+1:end-m) = 1*rho_mult*rho_base; % x_N = x_s. (6k)
        % Penalize constraints related to u
        rho(end-2*(n+m)+n+1:end-n-m) = rho_mult*rho_base; % Final  z1 + z2 + z3 = 0. (6j)  for i = N
        rho(end-m+1:end) = rho_mult*rho_base; % u_N = u_s. (6l) 
    end
    
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
    
    % Ingredients with minimal memory consumption
    H3i = 1./diag(H3);
       
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
    vars.H1i = reshape(H1i, m+n, N+1); % Inverse of diagonal of H1 in matrix form
    vars.H3i = reshape(H3i, m+n, N+1); % Inverse of 
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
    vars.LB0 = [-xoptions.inf_bound*ones(n,1); LBu];
    vars.UB0 = [options.inf_bound*ones(n,1); UBu];
    
    % Penalty parameter
    vars.rho = reshape(rho(n+1:end-n-m), m+n, N+1);
    vars.rho_0 = [rho(1:n); zeros(m,1)];
    vars.rho_s = rho(end-n-m+1:end);
    
    % Warmstart
    vars.L_z2 = L_z2;
    vars.L_z3 = L_z3(1:n, :);
    vars.L_l = L_l(1:2*n, :); 
    
    % Scaling vectors
    if isa(controller, 'TrackingMPC')
        vars.scaling = [controller.model.Nx; controller.model.Nu];
        vars.scaling_inv_u = 1./controller.model.Nu;
        vars.OpPoint = [controller.model.x0; controller.model.u0];
    else
        if isfield(controller.sys, 'Nx')
            vars.scaling = controller.sys.Nx;
        else
            vars.scaling = ones(n, 1);
        end
        if isfield(controller.sys, 'Nu')
            vars.scaling = [vars.scaling; controller.sys.Nu];
        else
            vars.scaling = [vars.scaling; ones(m, 1)];
        end
        if isfield(controller.sys, 'Nu')
            vars.scaling_inv_u = 1./controller.sys.Nu;
        else
            vars.scaling_inv_u = ones(m, 1);
        end
        if isfield(controller.sys, 'x0')
            vars.OpPoint = controller.sys.x0;
        else
            vars.OpPoin=  zeros(n, 1);
        end
        if isfield(controller.sys, 'u0')
            vars.OpPoint = [vars.OpPoint; controller.sys.u0];
        else
            vars.OpPoint = [vars.OpPoint; zeros(m, 1)];
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
    
end
