%% compute_ellipHMPC_ADMM_ingredients
%
% Computes the ingredients of the ADMM-based solver for HMPC with harmonic reference.
%
% Information about the solver can be found in:
%
% Pablo Krupa, Daniel Limon, Alberto Bemporad, Teodoro Alamo, "Efficiently
% solving the hamonic model predictive control formulation", arXiv: 2202.06629, 2022.
% 
% INPUTS:
%   - controller: Contains the information of the controller.
%   - opt: Structure containing options of the solver.
% 
% OUTPUTS:
%   - vars: Structure containing the ingredients required by the solver
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
%

function var = compute_ellipHMPC_ADMM_ingredients(controller, opt)

    %% Extract from controller
    
    % Model
    A = controller.sys.A;
    if isa(controller.sys, 'ssModel')
        B = controller.sys.Bu;
    else
        B = controller.sys.B;
    end
    n = size(A, 1);
    m = size(B, 2);
    % Constraints
    if isa(controller.sys, 'ssModel')
        E = controller.sys.Cc;
        F = controller.sys.Dc;
        LBy = controller.sys.LBy;
        UBy = controller.sys.UBy;   
    else
        E = controller.sys.E;
        F = controller.sys.F;
        LBy = controller.sys.LBy;
        UBy = controller.sys.UBy;
    end        
    n_y = length(LBy);
    
    % HMPC parameters
    N = controller.param.N;
    w = controller.param.w;
    Q = controller.param.Q;
    R = controller.param.R;
    Te = controller.param.Te;
    Th = controller.param.Th;
    Se = controller.param.Se;
    Sh = controller.param.Sh;
    
    %% Compute auxiliary terms used to compute the Hessian
    
    % Sin and cos at each prediction step j: s_j = sin(w*(j)); c_j = cos(w*(j));
    s_j = zeros(1, N);
    c_j = zeros(1, N);
    for j = 0:N-1
        s_j(j+1) = sin(w*(j));
        c_j(j+1) = cos(w*(j));
    end
    
    % Terms obtained from the summation of sine and cosine
    s_sum = 0;
    c_sum = 0;
    s2_sum = 0;
    c2_sum = 0;
    sc_sum = 0;
    for j = 0:N-1 % CHANGED from j = 1:N-1
        s_sum = s_sum + sin(w*j);
        c_sum = c_sum + cos(w*j);
        s2_sum = s2_sum + sin(w*j)^2;
        c2_sum = c2_sum + cos(w*j)^2;
        sc_sum = sc_sum + sin(w*j)*cos(w*j);
    end
    
    %% Compute Hessian matrix and q vector
    % We construct each block of the Hessian matrix and then join them together
    % We have the symmetric block is, H = [H11, H12, H13, H21, H22, H23, H31, H32, H33]
    % The vector of decision variables is given by:
    %   z = (u_0, x_1, u_1, x_2, u_2, ...., x_{N-1}, u_{N-1}, xe, xs, xc, ue, us, uc)
    % H11 corresponds to x_i, u_i with themselves, H12 to crossterms between x_i with (xs, xs, xc), 
    % H13 to crossterms between u_i with (ue, us, uc), H22 of (xe, xs, xc) with themselves and H33 of (ue, us, uc) with themselves.
    % Sin H is symmetric, we only need to compute the diagonal blocks and the upper triangular blocks.

    % Matrix H11
    H11 = blkdiag(R, kron(eye(N-1), blkdiag(Q, R)));

    % Matrix H12
    H12 = zeros((N-1)*(n+m)+m, 3*n);
    for j = 0:N-2
        H12(j*(n+m)+m+1:(j+1)*(n+m), :) = kron([1 s_j(j+2) c_j(j+2)], -Q);
    end

    % Matrix H13
    H13 = zeros((N-1)*(n+m)+m, 3*m);
    for j = 0:N-1
        H13(j*(n+m)+1:j*(n+m)+m, :) = kron([1 s_j(j+1) c_j(j+1)], -R);
    end

    % Matrix H22
    H22 = [Te + N*Q, s_sum*Q, c_sum*Q;
           s_sum*Q, Th + s2_sum*Q, sc_sum*Q;
           c_sum*Q, sc_sum*Q, Th + c2_sum*Q];
    
    % Add the j=0 tern to the summations of sines and cosines terms
%     s_sum = s_sum + sin(0);
%     c_sum = c_sum + cos(0);
%     s2_sum = s2_sum + sin(0)^2;
%     c2_sum = c2_sum + cos(0)^2;
%     sc_sum = sc_sum + sin(0)*cos(0);
    
    % Matrix H33
    H33 = [Se + N*R, s_sum*R, c_sum*R;
           s_sum*R, Sh + s2_sum*R, sc_sum*R;
           c_sum*R, sc_sum*R, Sh + c2_sum*R];
      
    % Matrix H23
    H23 = zeros(3*n, 3*m);
    
    % Construct Hessian and q
    H = [H11, H12, H13; H12', H22, H23; H13', H23', H33];
    dim = size(H, 1);
    q = -[zeros((N-1)*(n+m)+m, 1); Te*zeros(n,1); Th*zeros(n,1); Th*zeros(n,1); Se*zeros(m,1); Sh*zeros(m,1); Sh*zeros(m,1)];
    
    %% Equality constraints G and b
    G = kron(eye(N-1), [A B]); % Diagonal of the matrix
    j = 0;
    for i=1:n:n*N-2*n % Insert matrices -I in G
        j = j+1;
        G(i:i+n-1,((j-1)*(n+m)+(n+m+1)):((j-1)*(n+m)+(n+m+1)+n-1)) = -eye(n);
    end
    G = [B -eye(n) zeros(n, size(G, 2) - n); zeros(size(G, 1), m) G];
    %G = [G, [zeros(size(G, 1)-n, 3*(n+m)); -eye(n), zeros(n), -eye(n), zeros(n, 3*m)]];
    G = [G, [zeros(size(G, 1)-n, 3*(n+m)); -eye(n), -eye(n)*sin(w*(N)), -eye(n)*cos(w*(N)), zeros(n, 3*m)]];
    G = [G; zeros(3*n, size(G, 2)-3*(n+m)), [(A - eye(n)) zeros(n, 2*n), B, zeros(n, 2*m);
                                            zeros(n), (A - cos(w)*eye(n)), sin(w)*eye(n), zeros(n, m), B, zeros(n, m);
                                            zeros(n), -sin(w)*eye(n), (A - cos(w)*eye(n)), zeros(n, 2*m), B]];
    n_eq = size(G, 1);
    b = [-A*zeros(n, 1); zeros(n*(N+2), 1)];
    
    %% Cone constraints
        
    if opt.solver.use_soc

        C_aux = [];
        dsoc = [];
        for j = 1:n_y
            Eub = blkdiag(E(j,:), -E(j,:), -E(j,:));
            Elb = blkdiag(-E(j,:), -E(j,:), -E(j,:));
            Fub = blkdiag(F(j,:), -F(j,:), -F(j,:));
            Flb = blkdiag(-F(j,:), -F(j,:), -F(j,:));
            C_aux = [C_aux; [Eub, Fub; Elb, Flb]];
            dsoc = [dsoc; UBy(j); 0; 0; -LBy(j); 0; 0];
        end
        n_soc = 2*n_y;

    else

        C_aux = [];
        for j = 1:n_y
            C_aux = [C_aux; kron(eye(3), -E(j, :)), kron(eye(3), -F(j, :))];
        end
        dsoc = zeros(3*n_y, 1);
        n_soc = n_y;

    end

    C = blkdiag(-F, kron(eye(N-1), [-E, -F]), C_aux);
    d = [zeros(N*n_y, 1); dsoc];
    LB = kron(ones(N, 1), LBy);
    UB = kron(ones(N, 1), UBy);
    n_box = N*n_y;

    n_s = size(C, 1); % Dimension of s
    
    %% Solver ingredients
    Hh = H + opt.solver.rho*(C'*C);
    
    % Matrices for solving the system of equations sparsely
    M = [Hh, G'; G, zeros(n_eq)];
    % Compute LDL factorization of M
    [L, D, Pldl] = ldl(M);
    L_CSC = full2CSC(L - eye(size(M, 1)));
    Dinv = inv(D);
    idx_LDL = zeros(dim+n_eq, 1);
    for i = 1:dim+n_eq
        idx_LDL(i) = find(Pldl(i,:));
    end
    
    % Matrices for solving the system of equations non-sparsely
    Hhi = inv(Hh);
    W = G*Hhi*G';
    M1 = Hhi*G'*inv(W)*G*Hhi - Hhi;
    M2 = Hhi*G'*inv(W);
    M2 = M2(:, 1:n);
    
    if strcmp(opt.method, 'ADMM')
        opt.set('alpha', 1);
    end
    
    %% Return variables for the solver
    var.dim = dim;
    var.n_s = n_s;
    var.n_eq = n_eq;
    var.N = N;
    var.n = n;
    var.m = m;
    var.n_y = n_y;
    var.n_soc = n_soc;
    var.n_box = n_box;
    
    var.Q = Q; % ADDED
    var.R = R;
    var.Te = Te;
    var.Th = Th;
    var.Se = Se;
    var.Sh = Sh;
    var.A = A;
    var.LB = LB;
    var.UB = UB;
    var.LBy = LBy + opt.solver.sigma*ones(length(LBy), 1);
    var.UBy = UBy - opt.solver.sigma*ones(length(LBy), 1);
    var.E = E;
    var.F = F;
    
    var.H = H; % Cost function: Hessian and vector q
    var.q = q;
    var.Hh = Hh; % Hessian of the z update subproblem
    var.G = G; % Equality constraints
    var.b = b;
    var.C = C; % Matrices for the cone constraints: relate x to s
    var.C_CSR = sp_utils.full2CSR(C);
    var.Ct_CSR = sp_utils.full2CSR(C');
    var.d = d;
    
    var.Hwx = [eye(n)*cos(w), -eye(n)*sin(w);
               eye(n)*sin(w), eye(n)*cos(w)];
    var.Hwu = [eye(m)*cos(w), -eye(m)*sin(w);
               eye(m)*sin(w), eye(m)*cos(w)];
    var.Hx = blkdiag(eye(n), var.Hwx);
    var.Hu = blkdiag(eye(m), var.Hwu);
    
    var.L_CSC = L_CSC;
    var.Dinv = diag(Dinv);
    var.Pldl = Pldl;
    var.idx_LDL = idx_LDL;
    var.M1 = M1;
    var.M2 = M2(:, 1:n);
    
    var.rho = opt.solver.rho; % Penalty parameter rho
    var.rho_i = 1/opt.solver.rho;
    var.k_max = opt.solver.k_max; % Maximum number of iterations
    var.tol_p = opt.solver.tol_p; % Absolute tolerance
    var.tol_d = opt.solver.tol_d; % Relative tolerance
    
    % Scaling vectors and operating point
    if isfield(controller.sys, 'Nx')
        var.scaling_x = controller.sys.Nx;
    else
        var.scaling_x = ones(n, 1);
    end
    if isfield(controller.sys, 'Nu')
        var.scaling_u = controller.sys.Nu;
    else
        var.scaling_u = ones(m, 1);
    end
    if isfield(controller.sys, 'Nu')
        var.scaling_i_u = 1./controller.sys.Nu;
    else
        var.scaling_i_u = ones(m, 1);
    end
    if isfield(controller.sys, 'x0')
        var.OpPoint_x = controller.sys.x0;
    else
        var.OpPoint_x = zeros(n, 1);
    end
    if isfield(controller.sys, 'u0')
        var.OpPoint_u = controller.sys.u0;
    else
        var.OpPoint_u = zeros(m, 1);
    end
    
end
