%% Spcies_compute_RMPC_EADMM_ingredients
% Computes the ingredients for the RMPC controller solved using the ADMM algorithm
% This version uses the projection algorithm onto the ellipsoid

% Author: Pablo Krupa (pkrupa@us.es)
% 
% Changelog: 
%   v0.1 (2020/10/16): Initial commit version
%

function stru = Spcies_compute_RMPC_ADMM_ingredients(sys, param, options)

    %% Default values
    def_rho = 0.01;

    %% Rename variables
    A = sys.A;
    B = sys.Bu;
    n = size(A, 1);
    m = size(B, 2);
    N = param.N;
    Q = param.Q;
    R = param.R;
    P = param.P;
    Pt = param.Pt;
    
    %% Penalty parameter
    if isfield(options, 'rho')
        if isscalar(options.rho)
            rho = options.rho*ones(N*(n+m), 1);
        else
            rho = options.rho;
        end
    else
        rho = def_rho*ones(N*(n+m), 1);
    end
    
    %% Compute the Hessian H and the vector q
    
    % Hessian and q for variable z
    Hz = blkdiag(R, kron(eye(N-1), blkdiag(Q, R)), Pt);
    
    P_half = sqrtm(P);
    H = Hz + rho.*blkdiag(eye(size(Hz, 1)-n), P);
    q = zeros(N*(n+m), 1);
    
    %% Compute the matrix Aeq and vector beq
    
    % Compute the top part of matrix Az, which corresponds to the prediction model constraints
    Aeq = kron(eye(N-1), [A B]); % Diagonal of the matrix
    j = 0;
    for i=1:n:n*N-n % Insert matrices -I in Az
        j = j+1;
        Aeq(i:i+n-1,((j-1)*(n+m)+(m+n+1)):((j-1)*(n+m)+(n+m+1)+n-1)) = -eye(n);
    end
    Aeq = [B -eye(n) zeros(n, size(Aeq, 2) - n); zeros(size(Aeq, 1), m) Aeq]; % Initial condition
    
    beq = zeros(size(Aeq, 1), 1);
    
    %% Compute matrix W
    Hinv = inv(H);
    W = Aeq*Hinv*Aeq';
    Wc = chol(W);
    
    %% Compute the tightened constraints
    
    LBz = sys.LBu; UBz = sys.UBu;
    for i = 2:N
        LBz = [LBz; sys.LBx + param.HLsets.incBx(:, i); sys.LBu + param.HLsets.incBu(:, i)];
        UBz = [UBz; sys.UBx - param.HLsets.incBx(:, i); sys.UBu - param.HLsets.incBu(:, i)];
    end
    
    %% Create variables used in the sparse solver
    stru.n = n;
    stru.m = m;
    stru.N = N;
    stru.Hi_0 = diag(Hinv(1:m, 1:m));
    stru.Hi = reshape(diag(Hinv(m+(1:(N-1)*(n+m)),m+(1:(N-1)*(n+m)))), n+m, N-1);
    stru.Hi_N = Hinv(end-n+1:end, end-n+1:end);
    stru.AB = [A B];
    stru.UBu0 = UBz(1:m);
    stru.LBu0 = LBz(1:m);
    stru.UBz = reshape(UBz(m+1:end), n+m, N-1);
    stru.LBz = reshape(LBz(m+1:end), n+m, N-1);
    stru.rho_0 = rho(1:m);
    stru.rho = reshape(rho(m+1:end-n), n+m, N-1);
    stru.rho_N = rho(end-n+1:end);
    stru.rho_i_0 = 1./rho(1:m);
    stru.rho_i = reshape(1./rho(m+1:end-n), n+m, N-1);
    stru.rho_i_N = 1./rho(end-n+1:end);
    stru.P = P;
    stru.P_half = P_half;
    stru.Pinv_half = inv(P)*P_half;
    stru.Q = -diag(param.Q);
    stru.R = -diag(param.R);
    stru.T = -param.Pt;
    stru.c = param.c;
    stru.r = param.r;
    
    % Scaling vectors and operating point
    stru.scaling_x = sys.Nx;
    stru.scaling_i_u = 1./sys.Nu;
    stru.OpPoint_x = sys.x0;
    stru.OpPoint_u = sys.u0;
    
    % Alpha and Beta
    stru.Beta = zeros(n,n,N);
    stru.Alpha = zeros(n,n,N-1);
    for i = 1:N
        stru.Beta(:,:,i) = Wc((i-1)*n+(1:n),(i-1)*n+(1:n));
        for j = 1:n
            stru.Beta(j,j,i) = 1/stru.Beta(j,j,i);
        end
    end
    for i = 1:N-1
        stru.Alpha(:,:,i) = Wc((i-1)*n+(1:n),i*n+(1:n));
    end
    
end
