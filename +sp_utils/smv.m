%% smv - Sparse Matrix-Vector multiplication
% 
% Performs a sparse matrix-vector multiplication where the matrix is given
% in the Compressed Sparse Row (CSR) format.
% 
% y = SpMV(val, col, row_ptr, x) computes the multiplication between the
% vector x and the matrix in the CSR format given by vectors val, col and row_ptr.
% 
% INPUTS:
%   - var: vector containing the values of the non-zero elements.
%   - col: vector containing the column-index of each of the non-zero elements in var.
%   - row_ptr: vector containing the row-index range of the non-zero elements in var.
%   - x: Vector to which to multiply the matrix by.
%
% OUTPUS:
%   - y: result of the matrix-vector multiplication.
% 
% See also: full2CSR
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function y = smv(val, col, row_ptr, x)

    % Initialize
    n = length(row_ptr)-1;
    y = zeros(n, 1);
    
    % Perform multiplication
    for i = 1:n
        for j = row_ptr(i):row_ptr(i+1)-1
                y(i) = y(i) + val(j)*x(col(j));
        end
    end

end
