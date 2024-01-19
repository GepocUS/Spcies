%% compute_ellipMPC_ADMM_soc_ingredients
%
% Computes the ingredients for the ellipMPC controller solved using ADMM for the case in which
% the terminal constraint is imposed using a second order cone constraint.
% 
% The ellipMPC formulation can be found at 
% 
% P. Krupa, R. Jaouani, D. Limon, and T. Alamo, “A sparse ADMM-based solver for linear MPC subject
% to terminal quadratic constraint,” arXiv:2105.08419, 2021.
% 
% However, there is currently no specific documentation on this solver.
% 
% INPUTS:
%   - controller: Contains the information of the controller.
%   - options: Structure containing options of the ADMM solver.
%   - spcies_options: Structure containing the options of the toolbox.
% 
% OUTPUTS:
%   - vars: Structure containing the ingredients required by the solver.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function vars = compute_ellipMPC_ADMM_soc_ingredients(controller, options, spcies_options)

    %% Extract from controller
    if isa(controller, 'ellipMPC')
        A = controller.model.A;
        B = controller.model.Bu;
        n = controller.model.n_x;
        m = controller.model.n_u;
        N = controller.N;
        Q = controller.Q;
        R = controller.R;
        T = controller.T;
        P = controller.P;
        c = controller.c;
        r = controller.r;
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
        P = controller.param.P;
        if isfield(controller.param, 'c')
            c = controller.param.c;
        else
            c = zeros(n, 1);
        end
        if isfield(controller.param, 'r')
            r = controller.param.r;
        else
            r = 1;
        end
    end
    
    % Check ingredients
    if ~isdiag(blkdiag(Q, R))
        error('Spcies:ellipMPC:ADMM:non_diagonal', 'ellipMPC using ADMM: matrices Q and R must be diagonal in the curret version of SPCIES');
    end
    
    %% Compute the Hessian H and the vector q
    H = blkdiag(R, kron(eye(N-1), blkdiag(Q, R)), T, 0);
    q = zeros(N*(n+m)+1, 1);
    dim = size(H, 1);
    
    %% Equality constraints: G and b
    
    % Compute the top part of matrix Az, which corresponds to the prediction model constraints
    G = kron(eye(N-1), [A B]); % Diagonal of the matrix
    j = 0;
    for i=1:n:n*N-n % Insert matrices -I in Az
        j = j+1;
        G(i:i+n-1,((j-1)*(n+m)+(m+n+1)):((j-1)*(n+m)+(n+m+1)+n-1)) = -eye(n);
    end
    G = [B -eye(n) zeros(n, size(G, 2) - n); zeros(size(G, 1), m) G]; % Initial condition
    G = blkdiag(G, 1);
    b = [-A*zeros(n, 1); zeros(n*(N-1), 1); r];
    n_eq = size(G, 1);
    
    %% Cone constrait: C and d
    P_half = sqrtm(P);
    qSOC = -(c'*P)';
    dSOC = c'*P*c - r^2;
    bSOC = P_half\-qSOC;
    
    C = [zeros(n+1, dim-n-1), [zeros(1, n), -1; -P_half, zeros(n, 1)]];
    d = [0; -bSOC];
    %d = zeros(n, 1);
    n_s = size(C, 1);
    
    %% Compute the tightened constraints
       
    % Determine value of incBx
    if isfield(controller.param, 'incBx')  
        if min(size(controller.param.incBx)) == 1
            incBx = rechape(controller.param.incBx, n, N+1);
        else
            incBx = controller.param.incBx;
        end
    else
        incBx = zeros(n, N+1);
    end

    % Determine value of incBu
    if isfield(controller.param, 'incBu') 
        if min(size(controller.param.incBu)) == 1
            incBu = rechape(controller.param.incBu, m, N+1);
        else
            incBu = controller.param.incBu;
        end
    else
        incBu = zeros(m, N+1);
    end

    % Generate LBz and UBz vectors
    LB = controller.sys.LBu; UB = controller.sys.UBu;
    for i = 2:N
        LB = [LB; controller.sys.LBx + incBx(:, i); controller.sys.LBu + incBu(:, i)];
        UB = [UB; controller.sys.UBx - incBx(:, i); controller.sys.UBu - incBu(:, i)];
    end
    
    %% Solver ingredients
    Hh = blkdiag(H + options.sigma*eye(dim), options.rho*eye(n_s));
    Gh = [G, zeros(n_eq, n_s); C, eye(n_s)];
    bh = [b; d];

    % Matrices for solving the minimization w.r.t. z
    Hhi = inv(Hh);
    W = Gh*Hhi*Gh';
    Wc = chol(W);
    
    % Compute the LDL decomposition of W
    Wc_diag = diag(Wc); % Diagonal of Wc
    L = Wc'*diag(1./Wc_diag); % Matrix L of the LDL decomposition
    D = diag(Wc_diag.^2); % Matrix D of the LDL decomposition
    Dinv = 1./diag(D); % Vector containing the inverse of the diagonal elements of D
    
    % Compute the CSC representation of (L - I)
    L_CSC = sp_utils.full2CSC(L - eye(size(L, 1)));
    
    % Compute the CSR representation of the matrices -Gh*Hhi and -Hhi*Gh'
    GhHhi_CSR = sp_utils.full2CSR(-Gh*Hhi);
    HhiGh_CSR = sp_utils.full2CSR(-Hhi*Gh');
    Hhi_CSR = sp_utils.full2CSR(-Hhi);
    
    %% Create variables used in the sparse solver
    vars.dim = dim; % Dimension of primal variables z
    vars.n_s = n_s; % Dimension of promal variables s
    vars.n_eq = n_eq; % Numer of equality constraints on z
    vars.n = n;
    vars.m = m;
    vars.N = N;
    vars.A = A;
    vars.Q = -Q;
    vars.R = -R;
    vars.T = -T;
    vars.LB = LB;
    vars.UB = UB;
    vars.rho = options.rho;
    vars.rho_i = 1./options.rho;
    vars.sigma = options.sigma;
    vars.sigma_i = 1./options.sigma;
    vars.L_CSC = L_CSC;
    vars.Dinv = Dinv;
    vars.GhHhi_CSR = GhHhi_CSR;
    vars.HhiGh_CSR = HhiGh_CSR;
    vars.Hhi_CSR = Hhi_CSR;
    vars.PhiP = inv(P_half)*P;
    vars.P_half_i = inv(P_half);
    vars.bh = bh;
    
    % Scaling vectors and operating point
    if isa(controller, 'ellipMPC')
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
    
end
