%% scale_ss - Scale and normalize a discrete state-space model
%
% This function receives a discrete ss object "sys" and normalizes
% it and scales it with the given operating point and scaling vectors.
% It does the same for the given constraints of the "real" system, i.e.
% the constraints given in "engineering" units.
% 
% sysN = scale_ss(sys, UBx, LBx, UBu, LBu, x0, u0, Nx, Nu)
%
% INPUTS:
% - sys: Discrete state-space model of the "real" system (an ss instance)
% - UBx, LBx, UBu, LBu: System constraints in "engineering" units
% - x0, u0: Operating point of the system (in "engineering" units)
% - Nx, Nu: Vectors containing the diagonal elements of the scaling matrices
%           for the state and control input.
% 
% The function transforms the state and control input (x_o, u_o) of the given
% model sys to a new system whose state "x" and control input "u" are given by:
%   x = Nx.*(x_o - x0),   u = Nu.*(u_o - u0).
% The given system constraints are also scaled and shifted in the same way.
% 
% OUTPUTS:
% - sysN: A structure containing the fields required by Spcies for the scaled and
%         normalized system. It can be directly passed to spcies_gen_controller().
% 

function sysN = scale_ss(sys, UBx, LBx, UBu, LBu, x0, u0, Nx, Nu)

    % Scale the variables
    A = diag(Nx)*sys.A*diag(1./Nx);
    B = diag(Nx)*sys.B*diag(1./Nu);
    UBx = Nx.*(UBx - x0);
    LBx = Nx.*(LBx - x0);
    UBu = Nu.*(UBu - u0);
    LBu = Nu.*(LBu - u0);

    % Construct the output structure
    sysN = struct('A', A, 'B', B, 'UBx', UBx, 'LBx', LBx, 'UBu', UBu, 'LBu', LBu,...
                  'x0', x0, 'u0', u0, 'Nx', Nx, 'Nu', Nu);

end

