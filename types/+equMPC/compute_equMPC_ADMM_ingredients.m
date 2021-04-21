%% compute_equMPC_EADMM_ingredients
%
% Computes the ingredients for the ADMM-based solver for the equality MPC formulation
%
% Information about this formulation and the solver  can be found at:
%
% P. Krupa, D. Limon, T. Alamo, "Implementation of model predictive control in
% programmable logic controllers", Transactions on Control Systems Technology, 2020.
% 
% Specifically, this formulation is given in equation (8) of the above reference.
%
% INPUTS:
%   - controller: Contains the information of the controller.
%   - options: Structure containing options of the EADMM solver.
%   - spcies_options: Structure containing the options of the toolbox.
% 
% OUTPUTS:
%   - vars: Structure containing the ingredients required by the solver.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
%

function vars = compute_equMPC_ADMM_ingredients(controller, options, spcies_options)

    %% Extract from controller
    if isa(controller, 'EqualityMPC')
        A = controller.model.A;
        B = controller.model.Bu;
        n = controller.model.n_x;
        m = controller.model.n_u;
        N = controller.N;
        Q = controller.Q;
        R = controller.R;
        P = controller.P;
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
        P = controller.param.P;
    end
    
    % Check ingredients
    if ~isdiag(blkdiag(Q, R))
        error('Spcies:equMPC:non_diagonal_matrices', 'Matrices Q and R must be diagonal');
    end
    
    %% Turn rho into a vector
    if isscalar(options.rho) && options.force_vector_rho
        rho = options.rho*ones(N*(n+m) - n, 1);
    else
        rho = options.rho;
    end
    if isscalar(rho)
        vars.rho_is_scalar = true;
    else
        vars.rho_is_scalar = false;
    end
    
    %% Compute the Hessian H and the vector q
    
    % Hessian and q for variable z
    H = blkdiag(R, kron(eye(N-1), blkdiag(Q, R)));
    if vars.rho_is_scalar
        Hhat = H + rho*eye(N*(n+m) - n);
    else
        Hhat = H + diag(rho);
    end
    q = zeros(N*(n+m) - n,1);
    
    %% Compute the matrix Aeq
    
    % Compute the top part of matrix Az, which corresponds to the prediction model constraints
    Aeq = kron(eye(N-1), [A B]); % Diagonal of the matrix
    j = 0;
    for i=1:n:n*N-n % Insert matrices -I in Az
        j = j+1;
        Aeq(i:i+n-1,((j-1)*(n+m)+(m+n+1)):((j-1)*(n+m)+(n+m+1)+n-1)) = -eye(n);
    end
    Aeq = [B -eye(n) zeros(n, size(Aeq, 2) - n); zeros(size(Aeq, 1), m) Aeq]; % Initial condition
    Aeq = Aeq(:,1:end-n);
    
    %% Compute matrix W
    Hinv = inv(Hhat);
    W = Aeq*Hinv*Aeq';
    Wc = chol(W);
    
    %% Compute the tightened constraints
    if isa(controller, 'EqualityMPC')
        LB = [controller.model.LBx; controller.model.LBu];
        UB = [controller.model.UBx; controller.model.UBu];
    else
        LB = [controller.sys.LBx; controller.sys.LBu];
        UB = [controller.sys.UBx; controller.sys.UBu];
    end
    
    %% Create variables used in the sparse solver
    vars.n = n;
    vars.m = m;
    vars.N = N;
    vars.Hi_0 = diag(Hinv(1:m, 1:m));
    vars.Hi = reshape(diag(Hinv(m+(1:(N-1)*(n+m)),m+(1:(N-1)*(n+m)))), n+m, N-1)';
    vars.AB = [A B];
    vars.UB = UB;
    vars.LB = LB;
    vars.Q = -diag(Q);
    vars.R = -diag(R);
    vars.P = -P;
    
    % rho
    if (vars.rho_is_scalar)
        vars.rho = rho;
        vars.rho_i = 1/rho;
    else
        vars.rho_0 = rho(1:m);
        vars.rho = reshape(rho(m+1:end-n), n+m, N-1)';
        vars.rho_i_0 = 1./rho(1:m);
        vars.rho_i = reshape(1./rho(m+1:end-n), n+m, N-1)';
    end
    
    % Scaling vectors and operating point
    if isa(controller, 'EqualityMPC')
        vars.scaling_x = controller.model.Nx;
        vars.scaling_u = controller.model.Nu;
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
        vars.Beta(:,:,i) = Wc((i-1)*n+(1:n),(i-1)*n+(1:n));
        for j = 1:n
            vars.Beta(j,j,i) = 1/vars.Beta(j,j,i);
        end
    end
    for i = 1:N-1
        vars.Alpha(:,:,i) = Wc((i-1)*n+(1:n),i*n+(1:n));
    end
    
end

