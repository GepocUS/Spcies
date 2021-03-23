%% full2CSC - Compute Compressed Sparse Column form of a sparse matrix
% 
% M_spare = full2CSC(M) computes the Compressed Sparse Column (CSC)
% representation of the (sparse) matrix M.
% 
% INPUTS:
%   - M: Matrix to be converted
%   - threshold: Any number smaller than this value is considered 0.
%                This in an optional argument that defaults to 0.
%
% OUTPUS:
%   - M_sparse: Structure containing the sparse representation of M
%       - val: Vector containing the values of the non-zero elements
%       - row: Vector containing the row-indexes of each of the
%              non-zero elements
%       - col: Vector containing the column-index range of the
%              non-zero elements
%       - nnz: Number of non-zero elements
%       - nrow: number of rows of M
%       - ncol: number of columns of M
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function M_sparse = full2CSC(M, threshold)
    if nargin == 1
        threshold = 0; % Any number smaller than threshold is considered a 0
    end
    
	%% Compute dimensions of M
    [n, m] = size(M);
    
    %% Compute the CSR representation of the transposed
    M_CSR = utils.full2CSR(M', threshold);
    
    %% Fix the values of nrow and ncol
    M_sparse.val = M_CSR.val;
    M_sparse.row = M_CSR.col;
    M_sparse.col = M_CSR.row;
    M_sparse.nnz= M_CSR.nnz;
    M_sparse.nrow = n;
    M_sparse.ncol = m;
    
end

