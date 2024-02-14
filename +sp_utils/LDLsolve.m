%% LDLsolve - Sparse solver for systems of equations L*D*L'*x = b
% 
% This function sparsely solves the system of equations L*D*L'*x = b where 
% L is lower triangular and given in the Compressed Sparse Column (CSC) format
% and D is diagonal and positive definite.
% 
% INPUTS:
%   - val: vector containing the values of the non-zero elements of (L-I).
%   - row: vector containing the column-index of each of the non-zero elements of (L-I).
%   - col_ptr: vector containing the row-index range of the non-zero elements of (L-I).
%   - Dinv: vector containing the inverse of the diagonal elements of D.
%   - b: right-hand-side vector of the system of equations.
%
% OUTPUTS:
%   - x: solution to the system of equations.
%
% See also: full2CSC, full2LDL
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function x = LDLsolve(val, row, col_ptr, Dinv, b)
    
    n = length(b); % Dimension of the vectors
    x = b; % Initialize x to b
    
    %% Forward substitution
    for i=1:n
        value = x(i);
        for j = col_ptr(i):col_ptr(i+1)-1
            x(row(j)) = x(row(j)) - val(j)*value;
        end
    end
    
    %% Multiply by Dinv
    for i = 1:n
        x(i) = x(i)*Dinv(i);
    end
    
    %% Backward substitution
    for i = n:-1:1
        value = x(i);
        for j = col_ptr(i):col_ptr(i+1)-1
            value = value - val(j)*x(row(j));
        end
        x(i) = value;
    end
    
end
