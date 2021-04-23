%% solve_boxQP - Solves a QP problem with box constraints and diagonal Hessian
%
% INPUTS:
%   - q: Vector of the cost function of the QP problem.
%   - Hinv: Vector containing the inverse of the diagonal elements of the Hessian.
%           A scalar can be provided if its elements are all the same.
%   - z_lb: Lower bound of the box constraints.
%   - z_ub: Upper bound of the box constraints.
% 
% OUTPUTS:
%   - z_opt: Optimal solution of the QP problem
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
%

function z_opt = solve_boxQP(q, Hinv, z_lb, z_ub)

    % Get number of decision variables
    n_z = length(q);
    
    % Initialize optimal solution
    z_opt = zeros(n_z, 1);
    
    % Compute each component
    if length(Hinv) == 1
        for j = 1:n_z
            z_opt(j) = max( min( -Hinv*q(j), z_ub(j) ), z_lb(j) );
        end
    else
        for j = 1:n_z
            z_opt(j) = max( min( -Hinv(j)*q(j), z_ub(j) ), z_lb(j) );
        end
    end

end

