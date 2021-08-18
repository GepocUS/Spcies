%% full2LDL - compute LDL decomposition of positive definite matrix
% 
% [L, D] = full2LDL(M) computes the matrices L and D of the 
% LDL decomposition of matrix M, i.e., the lower triangular
% matrix L and the diagonal matrix D that satisfy M = L*D*L'.
% 
% [val, row, col_ptr, Dinv] = full2LDL(M, 1) computes the
% ingredients of the LDL decomposition of M required by function
% LDLsolve to sparsely solve the system of equations M*x = b.
% 
% See also: LDLsolve, full2CSC
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function varargout = full2LDL(M, for_LDLsolve)
    if nargin == 1
        for_LDLsolve = 0;
    end

    %% Compute the LDL decomposition
    
    % Compute the Cholesky decomposition of M
    try
        Mc = chol(M); % Cholesky decomposition of M
    catch
        error('Could not compute the Cholesky decomposition of the matrix');
    end
    Mc_diag = diag(Mc); % Diagonal of Mc
    
    % Compute L and D
    L = Mc'*diag(1./Mc_diag); % Matrix L of the LDL decomposition
    D = diag(Mc_diag.^2); % Matrix D of the LDL decomposition
    
    %% Return variables
    if ~for_LDLsolve
        
        varargout(1) = {L};
        varargout(2) = {D};
        
    else
        
        % Compute CSC decomposition of (L - I)
        L_CSC = utils.full2CSC(L - eye(size(L, 1)));
        
        % Compute inverse of D (in vector form)
        Dinv = 1./diag(D); % Vector containing the inverse of the diagonal elements of D
        
        % Return variables
        varargout(1) = {L_CSC.val};
        varargout(2) = {L_CSC.row};
        varargout(3) = {L_CSC.col};
        varargout(4) = {Dinv};
        
    end
    
end
