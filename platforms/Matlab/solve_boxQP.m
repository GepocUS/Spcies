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

