%% compute_MPCT_ADMM_cs_ingredients
%
% Computes the ingredients for the MPCT controller solved using ADMM on an extended state space
%
% The solver extends the state and control inputs by adding the artificial reference to them.
% Currently, there is no additional documentation available for the solver.
% 
% INPUTS:
%   - controller: Contains the information of the controller.
%   - opt: Structure containing options of the solver.
% 
% OUTPUTS:
%   - vars: Structure containing the ingredients required by the solver.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function vars = compute_MPCT_ADMM_cs_ingredients(controller, opt)

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
            LBx = -opt.inf_value*ones(n, 1);
        end
        try
            UBx = controller.sys.UBx;
        catch
            UBx = opt.inf_value*ones(n, 1);
        end
        try
            LBu = controller.sys.LBu;
        catch
            LBu = -opt.inf_value*ones(m, 1);
        end
        try
            UBu = controller.sys.UBu;
        catch
            UBu = opt.inf_value*ones(m, 1);
        end
    end
    
    %% Turn rho into a vector
    if isscalar(opt.solver.rho) && opt.solver.force_vector_rho
        rho = opt.solver.rho*ones(N*(n+m), 1);
    else
        rho = opt.solver.rho;
    end
    if isscalar(rho)
        vars.rho_is_scalar = true;
    else
        vars.rho_is_scalar = false;
    end
    
    %% Compute the Hessian and q vector
    Qz = [Q, -Q; -Q, (Q + (1/N)*T)];
    Rz = [R, -R; -R, (R + (1/N)*S)];

    H = kron(eye(N), blkdiag(Qz, Rz));
    if vars.rho_is_scalar
        Hhat = H + rho*eye(N*2*(n+m));
    else
        Hhat = H + diag(rho);
    end
    
    q = zeros(2*N*(n+m), 1);
    
    %% Compute equality constraints (Aeq*dec_var = beq)
    AA = [A, zeros(n); zeros(n), eye(n); zeros(m,2*n)];
    BB = [B, zeros(n,m); zeros(n,2*m); zeros(m), eye(m)];
    II = [-eye(n), zeros(n,n+2*m); zeros(n), -eye(n), zeros(n,2*m); zeros(m,2*n+m), -eye(m)];
    
    Aeq = blkdiag(kron(eye(N-1), [AA, BB]), [A, -eye(n), B, zeros(n,m)]);
    % Insert the II matrices
    j = 0;
    for(i = 1:2*n+m:(2*n+m)*(N-1))
        j = j + 1;
        Aeq(i:i+2*n+m-1, j*(2*n+2*m)+1:j*(2*n+2*m)+2*n+2*m) = II;
    end
    % Insert the initial condition and the condition xs = A*xs + B*us
    init_cond = [eye(n), zeros(n,n+2*m); zeros(n), (A-eye(n)), zeros(n,m), B];
    Aeq = [init_cond, zeros(2*n, size(Aeq,2) - (2*n + 2*m)); Aeq];
    
    % Compute bz
    beq = zeros(2*n+(2*n+m)*(N-1)+n, 1);
    
    %% Compute upper and lower bounds
    LBz = [LBx; LBx + opt.solver.epsilon_x*ones(n, 1)]; % z_j = [x_j; x_s];
    UBz = [UBx; UBx - opt.solver.epsilon_x*ones(n, 1)];
    LBv = [LBu; LBu + opt.solver.epsilon_u*ones(m, 1)]; % v_j = [u_j; u_s];
    UBv = [UBu; UBu - opt.solver.epsilon_u*ones(m, 1)];
    
    LB = kron(ones(N, 1), [LBz; LBv]);
    UB = kron(ones(N, 1), [UBz; UBv]);
    
    %% Compute ingredients for solving the W*dec_var = rhs system of equations
    Hinv = inv(Hhat);
    W = Aeq*Hinv*Aeq';
    Wc = chol(W);
    
    % Compute the LDL decomposition of W
    Wc_diag = diag(Wc); % Diagonal of Wc
    L = Wc'*diag(1./Wc_diag); % Matrix L of the LDL decomposition
    D = diag(Wc_diag.^2); % Matrix D of the LDL decomposition
    Dinv = 1./diag(D); % Vector containing the inverse of the diagonal elements of D
    
    % Compute the CSC representation of (L - I)
    L_CSC = sp_utils.full2CSC(L - eye(size(L, 1)));
    
    % Compute the CSR representation of the matrices -Aeq*Hinv and -Hinv*Aeq'
    AHi_CSR = sp_utils.full2CSR(-Aeq*Hinv);
    HiA_CSR = sp_utils.full2CSR(-Hinv*Aeq');
    Hi_CSR = sp_utils.full2CSR(-Hinv);
    
    %% Create variables used in the sparse solver
    vars.n = n;
    vars.m = m;
    vars.N = N;
    vars.Tz = -(1/N)*T;
    vars.Sz = -(1/N)*S;
    vars.LB = LB;
    vars.UB = UB;
    vars.rho = rho;
    vars.rho_i = 1./rho;
    vars.L_CSC = L_CSC;
    vars.Dinv = Dinv;
    vars.AHi_CSR = AHi_CSR;
    vars.HiA_CSR = HiA_CSR;
    vars.Hi_CSR = Hi_CSR;
    
    % Scaling vectors and operating point
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
    
end

