%% compute_MPCT_ADMM_band_ingredients
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

function [vars] = compute_MPCT_ADMM_band_ingredients(controller, options, spcies_options)

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
    
    Gamma_hat = band + rho * eye(n_z); % Band of P
    Gamma_hat = diag(Gamma_hat); % We only take a vector with the diagonal elements
%     dGamma_hat = decomposition(Gamma_hat,'diagonal'); % Decomposition object of Gamma_hat

    Gamma_hat_inv = 1./Gamma_hat; % This avoids the inverse computation online

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

    Gamma_tilde = G*diag(Gamma_hat_inv)*G';
%     dGamma_tilde = decomposition(Gamma_tilde,'banded'); % Decomposition object of Gamma_tilde

    Gamma_tilde_inv = inv(Gamma_tilde); % This avoids the inverse computation online

    U_tilde = -G*diag(Gamma_hat_inv)*U_hat*inv(eye(2*MM)+V_hat*diag(Gamma_hat_inv)*U_hat);

    V_tilde = V_hat*diag(Gamma_hat_inv)*G';

    % W = G*inv(H)*G' = Gamma_tilde + U_tilde * V_tilde

    %% Compute upper and lower bounds
    LBx_s = LBx + options.epsilon_x * ones(n,1); % Lower limit for the artificial reference of the state
    UBx_s = UBx - options.epsilon_x * ones(n,1); % Upper limit for the artificial reference of the state
    
    LBu_s = LBu + options.epsilon_u * ones(m,1); % Lower limit for the artificial reference of the input
    UBu_s = UBu - options.epsilon_u * ones(m,1); % Upper limit for the artificial reference of the input
    
    LB = [-options.inf_bound*ones(n,1) ; LBu]; % This makes x_0 not constrained
    UB = [options.inf_bound*ones(n,1) ; UBu];
    
    for i = 2:N
        LB = [LB ; LBx ; LBu];
        UB = [UB ; UBx ; UBu];
    end
    
    LB = [LB ; LBx_s ; LBu_s];
    UB = [UB ; UBx_s ; UBu_s];

    %% Create variables used in the ADMM_band solver for MPCT
    vars.N = N;  % Prediction horizon
    vars.n = n; % Dimension of state space
    vars.m = m; % Dimension of input space
    vars.T = T;
    vars.S = S;
    vars.G = G;
    vars.Gamma_hat = Gamma_hat;
    vars.Gamma_hat_inv = Gamma_hat_inv;
    vars.U_hat = U_hat;
    vars.V_hat = V_hat;
    vars.Gamma_tilde = Gamma_tilde;
    vars.Gamma_tilde_inv = Gamma_tilde_inv;
    vars.U_tilde = U_tilde;
    vars.V_tilde = V_tilde;
    vars.LB = LB;
    vars.UB = UB;
    vars.rho = rho;
    vars.rho_i = 1./rho;

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
    
    % Extract Alpha's and Beta's from Gamma_tilde
    for i = 1 : n : m_z
        vars.Beta(:,:,(i-1)/n+1) = Gamma_tilde(i:i+n-1,i:i+n-1);
        if((i-1)/n+1 <= n_alpha) % If we are in the last column, we do not add a new Alpha
            vars.Alpha(:,:,(i-1)/n+1) = Gamma_tilde(i:i+n-1,i+n:i+2*n-1);
        end
    end
    
end

