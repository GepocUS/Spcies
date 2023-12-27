%% compute_MPCT_ADMM_semiband_ingredients
%
% Computes the ingredients for the MPCT controller solved with ADMM
% exploiting the semi-banded structure of the problem using Woodbury Matrix Identity
%
% Information about this formulation and the solver can be found at:
%
% TODO: PONER NOMBRE DEL ARTÍCULO DEL ECC24 SI LO ACEPTAN
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

function [vars] = compute_MPCT_ADMM_semiband_ingredients(controller, options, spcies_options)

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

    %% Turn rho into a vector
    if isscalar(options.rho) && options.force_vector_rho
        rho = options.rho*ones((N+1)*(n+m), 1);
    else
        rho = options.rho;
    end
    if isscalar(rho)
        vars.rho_is_scalar = true;
    else
        vars.rho_is_scalar = false;
    end

    %% Compute the Hessian
    H = [];
    
    for i=1:N
        H = blkdiag(H,blkdiag(Q,R));
    end
    
    H = blkdiag(H,blkdiag(N*Q+T,N*R+S));
    
    band = H; % Banded part of the Hessian
    
    % Constructing vertical right-hand side of H
    for i=1:N
        H((i-1)*(n+m)+1:i*(n+m),end-(n+m)+1:end) = [-Q , zeros(n,m) ; zeros(m,n), -R];
    end
    
    % Constructing bottom side of H
    for i=1:N
        H(end-(n+m)+1:end,(i-1)*(n+m)+1:i*(n+m)) = [-Q , zeros(n,m) ; zeros(m,n), -R]';
    end

    %% Compute equality constraints (G*dec_var = beq)
    G = zeros((N+2)*n,(N+1)*(n+m));

    G(1:n,1:n) = [eye(n)]; % Initial condition: x_0 = x(t)
    k=0;
    for i = n+1 : n : (N+1)*n
        for j=1: n+m : size(G,2)-m 
            if (i-n)==(j-k*m)
                G(i:i+n-1,j:j+m+2*n-1) = [A, B, -eye(n)]; % Constraints of system dynamicss: x_{k+1} = A*x_{k} + B*u_{k}
            end
        end
        k=k+1;
    end
    
    G(end-n+1:end,end-(n+m)+1:end) = [A-eye(n), B]; % Condition of (x_s,u_s) as an equilibrium point

    m_z = size(G,1); % Number of equality constraints
    n_z = size(H,1); % Number of decision variables

%     beq = zeros(m_z,1);
%     beq(1:nx,1) = x;

    %% Ingredients necessary for low computational complexity
    
    if vars.rho_is_scalar
        Gamma_hat = band + rho * eye(n_z); % Band of P
    else
        Gamma_hat = band + diag(rho);
    end
%     dGamma_hat = decomposition(Gamma_hat,'diagonal'); % Decomposition object of Gamma_hat

    Gamma_hat_inv = inv(Gamma_hat); % This avoids the inverse computation online

    % Isolated vertical right-hand side of H
    Y = [-Q , zeros(n,m) ; zeros(m,n), -R]; 
    for i=1:N-1
        Y = [Y ; [-Q , zeros(n,m) ; zeros(m,n), -R]];
    end
    
    NN = size(Y,1);
    MM = size(Y,2);

    U_hat = [Y , zeros(NN,MM) ; zeros(MM,MM) , eye(MM)];
    V_hat = [zeros(MM,NN) , eye(MM) ; Y' , zeros(MM,MM)];

    % P = (Gamma_hat + U_hat*V_hat)

    Gamma_tilde = G*Gamma_hat_inv*G';
%     dGamma_tilde = decomposition(Gamma_tilde,'banded'); % Decomposition object of Gamma_tilde

    Gamma_tilde_inv = inv(Gamma_tilde); % This avoids the inverse computation online

    U_tilde = -G*Gamma_hat_inv*U_hat*inv(eye(2*MM)+V_hat*Gamma_hat_inv*U_hat);
    
    V_tilde = V_hat*Gamma_hat_inv*G';

    M_hat = inv((eye(2*(n+m))+V_hat*Gamma_hat_inv*U_hat))*V_hat;

    M_hat_x = zeros(2*n,2*n);

    M_hat_u = zeros(2*m,2*m);

    M_hat_x(1:2*n,1:2*n) = [M_hat(1:n,1:n) , M_hat(1:n,N*(n+m)+1:N*(n+m)+n) ; M_hat(n+m+1:2*n+m,1:n) , M_hat(n+m+1:2*n+m,N*(n+m)+1:N*(n+m)+n)];
    M_hat_u(1:2*m,1:2*m) = [M_hat(n+1:n+m,n+1:n+m) , M_hat(n+1:n+m, N*(n+m)+n+1:(N+1)*(n+m)) ; M_hat(2*n+m+1:2*(n+m),n+1:n+m), M_hat(2*n+m+1:2*(n+m),N*(n+m)+n+1:(N+1)*(n+m))];

    % W = G*inv(H)*G' = Gamma_tilde + U_tilde * V_tilde

    %% Compute upper and lower bounds
    LB = [LBx ; LBu];
    UB = [UBx ; UBu];
    
    %% Create variables used in the ADMM_semiband solver for MPCT
    vars.N = N;  % Prediction horizon
    vars.n = n; % Dimension of state space
    vars.m = m; % Dimension of input space
    vars.A = A;
    vars.B = B;
    vars.Q = Q;
    vars.R = R;
    vars.T = T;
    vars.S = S;
    vars.Q_rho_i = inv(Q + rho*diag(ones(n,1)));
    vars.R_rho_i = inv(R + rho*diag(ones(m,1)));
    vars.S_rho_i = inv(N*R + S + rho*diag(ones(m,1)));
    vars.T_rho_i = inv(N*Q + T + rho*diag(ones(n,1)));
    vars.G = G;
%     vars.Gamma_hat = Gamma_hat;
%     vars.Gamma_hat_inv = Gamma_hat_inv;
    vars.U_hat = U_hat;
%     vars.V_hat = V_hat;
    vars.Gamma_tilde = Gamma_tilde; % Only needed for solver in Matlab. In C Alpha's and Beta's are used
%     vars.Gamma_tilde_inv = Gamma_tilde_inv;
    vars.U_tilde = U_tilde;
%     vars.V_tilde = V_tilde;
    vars.M_hat = M_hat;
    vars.M_hat_x = M_hat_x;
    vars.M_hat_u = M_hat_u;
    vars.M_tilde = inv((eye(2*(n+m))+V_tilde*Gamma_tilde_inv*U_tilde))*V_tilde;
    vars.LB = LB;
    vars.UB = UB;
    vars.rho = rho;
    if (vars.rho_is_scalar)
        vars.rho_i = 1/rho;
    else
        vars.rho_i = 1./rho;
    end

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

    % Alpha and Beta
    n_beta = m_z/n; % Number of Beta's is N+2
    n_alpha = n_beta-1; % Number of Alpha's is N+1
    
    vars.Beta = zeros(n, n, n_beta);
    vars.Alpha = zeros(n, n, n_alpha);
    
    % Extract Alpha's and Beta's from Gamma_tilde_c
    Gamma_tilde_c = chol(Gamma_tilde);
    
    for i = 1 : n : m_z
        vars.Beta(:,:,(i-1)/n+1) = Gamma_tilde_c(i:i+n-1,i:i+n-1);
        if((i-1)/n+1 <= n_alpha) % If we are in the last column, we do not add a new Alpha
            vars.Alpha(:,:,(i-1)/n+1) = Gamma_tilde_c(i:i+n-1,i+n:i+2*n-1);
        end
    end

    % Passing the inverse of the diagonal of Beta's so that we multiply by them instead
    % of divinding. Used in solve_banded_Chol() function in C
    for i = 1 : n 
        vars.Beta(i,i,:) = 1/vars.Beta(i,i,:);
    end
    
end

