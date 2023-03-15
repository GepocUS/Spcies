%% compute_laxMPC_FISTA_ingredients
%
% Computes the ingredients for the FISTA-based solver for the lax MPC formulation
%
% Information about this formulation and the solver  can be found at:
%
% P. Krupa, D. Limon, T. Alamo, "Implementation of model predictive control in
% programmable logic controllers", Transactions on Control Systems Technology, 2020.
% 
% Specifically, this formulation is given in equation (9) of the above reference.
%
% INPUTS:
%   - controller: Contains the information of the controller.
%   - options: Structure containing options of the FISTA solver.
%   - spcies_options: Structure containing the options of the toolbox.
% 
% OUTPUTS:
%   - vars: Structure containing the ingredients required by the solver.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
%

function vars = compute_laxMPC_FISTA_ingredients(controller, options, spcies_options)

    %% Extract from controller
    if isa(controller, 'LaxMPC')
        A = controller.model.A;
        B = controller.model.Bu;
        n = controller.model.n_x;
        m = controller.model.n_u;
        N = controller.N;
        Q = controller.Q;
        R = controller.R;
        T = controller.P;
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
    end
    
    % Check ingredients
    if ~isdiag(blkdiag(Q, R, T))
        error('Spcies:laxMPC:FISTA:non_diagonal', 'laxMPC using ADMM: matrices Q, R and T must be diagonal in the curret version of SPCIES');
    end
     
    %% Compute the Hessian H and the vector q
    
    % Hessian and q for variable z
    H = blkdiag(R, kron(eye(N-1), blkdiag(Q, R)), T);
    
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
    if ~options.time_varying
        Hinv = inv(H);
        W = Aeq*Hinv*Aeq';
        Wc = chol(W);
    end    
    %% Compute the tightened constraints
    if isa(controller, 'LaxMPC')
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
    vars.T = -diag(T); 
    vars.Ti = -1./diag(T);

    if ~options.time_varying
        vars.AB = [A B];
        vars.Q = -diag(Q);
        vars.R = -diag(R);
        vars.QRi = -[1./diag(Q); 1./diag(R)];
    end
    
%     vars.UB = UB;
%     vars.LB = LB;
    
    % Scaling vectors and operating point
    if isa(controller, 'LaxMPC')
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
    if ~options.time_varying
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
    
end

