%% Spcies_compute_ellipMPC_EADMM_ingredients
% Computes the ingredients for the ADMM-based solver for MPC with ellipsoidal terminal constraint
%
% Information about this formulaiton and the solver  can be found at:
%
% t.b.d.
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

function vars = Spcies_compute_ellipMPC_ADMM_ingredients(controller, options)

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
    
    %% Turn rho into a vector if scalar is provided
    if isscalar(options.rho)
        rho = options.rho*ones(N*(n+m), 1);
    else
        rho = options.rho;
    end
    
    %% Compute the Hessian H and the vector q
    
    % Hessian and q for variable z
    Hz = blkdiag(R, kron(eye(N-1), blkdiag(Q, R)), T);
    
    P_half = sqrtm(P);
    H = Hz + rho.*blkdiag(eye(size(Hz, 1)-n), P);
    q = zeros(N*(n+m), 1);
    
    %% Compute the matrix Aeq
    
    % Compute the top part of matrix Az, which corresponds to the prediction model constraints
    Aeq = kron(eye(N-1), [A B]); % Diagonal of the matrix
    j = 0;
    for i=1:n:n*N-n % Insert matrices -I in Az
        j = j+1;
        Aeq(i:i+n-1,((j-1)*(n+m)+(m+n+1)):((j-1)*(n+m)+(n+m+1)+n-1)) = -eye(n);
    end
    Aeq = [B -eye(n) zeros(n, size(Aeq, 2) - n); zeros(size(Aeq, 1), m) Aeq]; % Initial condition
    
    %% Compute matrix W
    Hinv = inv(H);
    W = Aeq*Hinv*Aeq';
    Wc = chol(W);
    
    %% Compute the tightened constraints
    if isa(controller, 'ellipMPC')
        LBz = controller.LBz;
        UBz = controller.UBz;
    else
       
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
        LBz = controller.sys.LBu; UBz = controller.sys.UBu;
        for i = 2:N
            LBz = [LBz; controller.sys.LBx + incBx(:, i); controller.sys.LBu + incBu(:, i)];
            UBz = [UBz; controller.sys.UBx - incBx(:, i); controller.sys.UBu - incBu(:, i)];
        end
    end
    
    %% Create variables used in the sparse solver
    vars.n = n;
    vars.m = m;
    vars.N = N;
    vars.Hi_0 = diag(Hinv(1:m, 1:m));
    vars.Hi = reshape(diag(Hinv(m+(1:(N-1)*(n+m)),m+(1:(N-1)*(n+m)))), n+m, N-1);
    vars.Hi_N = Hinv(end-n+1:end, end-n+1:end);
    vars.AB = [A B];
    vars.UBu0 = UBz(1:m);
    vars.LBu0 = LBz(1:m);
    vars.UBz = reshape(UBz(m+1:end), n+m, N-1);
    vars.LBz = reshape(LBz(m+1:end), n+m, N-1);
    vars.rho_0 = rho(1:m);
    vars.rho = reshape(rho(m+1:end-n), n+m, N-1);
    vars.rho_N = rho(end-n+1:end);
    vars.rho_i_0 = 1./rho(1:m);
    vars.rho_i = reshape(1./rho(m+1:end-n), n+m, N-1);
    vars.rho_i_N = 1./rho(end-n+1:end);
    vars.P = P;
    vars.P_half = P_half;
    vars.Pinv_half = inv(P)*P_half;
    vars.Q = -diag(Q);
    vars.R = -diag(R);
    vars.T = -T;
    vars.c = c;
    vars.r = r;
    
    % Scaling vectors and operating point
    if isa(controller, 'ellipMPC')
        vars.scaling_x = controller.model.Nx;
        vars.scaling_i_u = 1./controller.model.Nu;
        vars.OpPoint_x = controller.model.x0;
        vars.OpPoint_u = controllr.model.u0;
    else
        if isfield(controller.sys, 'Nx')
            vars.scaling_x = controller.sys.Nx;
        else
            vars.scaling_x = ones(n, 1);
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
        vars.Beta(:,:,i) = Wc((i-1)*n+(1:n),(i-1)*n+(1:n));
        for j = 1:n
            vars.Beta(j,j,i) = 1/vars.Beta(j,j,i);
        end
    end
    for i = 1:N-1
        vars.Alpha(:,:,i) = Wc((i-1)*n+(1:n),i*n+(1:n));
    end
    
end
