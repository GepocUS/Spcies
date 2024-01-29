%% def_options_HMPC_SADMM
% 
% Returns the default options for the SADMM-based solver for HMPC.
%
% Information about this formulation can be found at:
%
% P. Krupa, D. Limon, and T. Alamo, “Harmonic based model predictive
% control for set-point tracking", IEEE Transactions on Automatic Control.
%
% Information about the solver itself will be available shortly.
%
% INPUT:
%   - submethod: string containing the submethod name (can be used for returning different default values)
% OUTPUT:
%   - def_options: Structure containing the default options of the solver.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function def_options = def_options_HMPC_SADMM(submethod)

    def_options = HMPC.def_options_HMPC_ADMM(submethod);
    
end
