%% def_options_HMPC_SADMM
% 
% Returns the default options for the SADMM-based solver for HMPC.
%
% OUTPUT:
%   - def_options: Structure containing the default options of the solver.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function def_options = def_options_HMPC_SADMM()

    def_options = personal.def_options_HMPC_ADMM;
    def_options.alpha = 0.95;
    
end
