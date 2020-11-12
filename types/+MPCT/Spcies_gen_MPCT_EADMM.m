%% Spcies_gen_MPCT_EADMM - Generate the MPCT solver based on the EADMM alggorithm
% 
% Information about this formulaiton and the solver  can be found at:
% 
% "Implementation of model predictive control for tracking in embedded systems
% using a sparse extended ADMM algorithm", by P. Krupa, I. Alvarado, D. Limon
% and T. Alamo, arXiv preprint: 2008:09071, 2020.
% 
% INPUTS:
%   - target: target embedded system that the controller is generated for. See documentation for a list of supported ones.
%   - sys: model of the system.
%   - param: structure containing  parameters of the MPCT controller.
%            See documentation for the parameters needed.
%   - options: structure containing options of the EADMM solver.
%              See documentation for the options available.
%   - save_name: string that determines the name of any files saved to the current directory.
% 
% OUTPUTS:
%   - str: Structure containing a variety of information
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

% Author: Pablo Krupa (pkrupa@us.es)
% 
% Changelog: 
%   v0.1 (2020/09/03): Initial commit version
%   v0.2 (2020/09/17): Added documentation
%

function str = Spcies_gen_MPCT_EADMM(target, sys, param, options, save_name)

    %% Compute the ingredients of the controller
    str = MPCT.Spcies_compute_MPCT_EADMM_ingredients(sys, param, options);
    
    %% Call the funciton that constructs the controller
    if strcmp(target, 'Arduino')
        gen_MPCT_EADMM_Arduino(str, options, save_name);
    elseif strcmp(target, 'Unity')
        gen_MPCT_EADMM_Unity(str, options, save_name);      
    else
        if ~strcmp(target, 'Matlab')
            error('Target not recognized or supported');
        end
    end
    
end
