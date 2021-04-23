%% solve_eqQP - Solves a QP problem subjecto to equality constraints
%
% INPUTS:
%   - q: Vector of the cost function of the QP problem.
%   - b: Vector of the equality constraints.
%   - Hinv: Inverse of the Hessian.
%   - G: Matrix defining ther equality constraints.
%   - W: Matrix given by W = G'*Hinv*G'
% 
% OUTPUTS:
%   - z_opt: Optimal solution of the QP problem
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
%

function z_opt = solve_eqQP(q, b, Hinv, G, W)

    % Compute mu
    mu = -G*Hinv*q - b;
    
    % Solve the system of equations
    mu = W\mu;
    
    % Compute the optimal solution
    z_opt = -Hinv*G'*mu - Hinv*q;

end

