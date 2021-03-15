%% full2CSR - Compute Compressed Sparse Row form of a sparse matrix
% 
% M_spare = full2CSR(M) computes the Compressed Sparse Row (CSR)
% representation of the (sparse) matrix M.
% 
% INPUTS:
%   - M: Matrix to be converted
%
% OUTPUS:
%   - M_sparse: Structure containing the sparse representation of M
%       - val: Vector containing the values of the non-zero elements
%       - col: Vector containing the column-indexes of each of the
%              non-zero elements
%       - row_ptr: Vector containing the row-index range of the
%                  non-zero elements
%       - nnz: Number of non-zero elements
%       - nrow: number of rows of M
%       - ncol: number of columns of M
%
% For CSC format, pass the transposed of the matrix, then
% [var, row, col_ptr] = full2CSC(M)
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function M_sparse = full2CSR(M, threshhold)
    if nargin == 1
        threshhold = 0; % Any number smaller than threshhold is considered a 0
    end
    
    %% Initialize
    [n, m] = size(M); % Dimensions of M
    nnz = 0; % Number of non-zero elements
    val = [];
    col = [];
    row = zeros(1, n+1);
    
    %% Compute the CSR form
    for i = 1:n
        row(i) = nnz + 1;
        for j = 1:m
            if abs(M(i, j)) > threshhold
                val = [val M(i, j)];
                col = [col j];
                nnz = nnz + 1;
            end
        end
    end
    
    row(n + 1) = nnz + 1; % Add the last element to row_ptr
    
    %% Construct the output structure
    M_sparse.val = val;
    M_sparse.col = col;
    M_sparse.row = row;
    M_sparse.nnz = nnz;
    M_sparse.nrow = n;
    M_sparse.ncol = m;

end
