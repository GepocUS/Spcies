%

function z_opt = solve_eqQP(q, b, Hinv, G, W)

    % Compute mu
    mu = -G*Hinv*q - b;
    
    % Solve the system of equations
    mu = W\mu;
    
    % Compute the optimal solution
    z_opt = -Hinv*G'*mu - Hinv*q;

end

