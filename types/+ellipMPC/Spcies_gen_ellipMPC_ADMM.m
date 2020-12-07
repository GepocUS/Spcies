%% Spcies_gen_ellipMPC_ADMM - Generate the RMPC solver based on the ADMM alggorithm
% This version uses the projection algorithm onto the ellipsoid

% Author: Pablo Krupa (pkrupa@us.es)
% 
% Changelog: 
%   v0.1 (2020/10/16): Initial commit version
%

function str = Spcies_gen_ellipMPC_ADMM(target, sys, param, options, save_name)

    %% Compute the ingredients of the controller
    str = ellipMPC.Spcies_compute_ellipMPC_ADMM_ingredients(sys, param, options);
    
    %% Call the funciton that constructs the controller
    if strcmp(target, 'C')
        ellipMPC.gen_ellipMPC_ADMM_C(str, options, save_name);   
    else
        if ~strcmp(target, 'Matlab')
            error('Target not recognized or supported');
        end
    end
    
end
