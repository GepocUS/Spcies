%% def_options_HMPC_ADMM
% 
% Returns the default options for the ADMM-based solver for HMPC.
%
% Information about this formulation can be found at:
%
% P. Krupa, D. Limon, and T. Alamo, “Harmonic based model predictive
% control for set-point tracking", IEEE Transactions on Automatic Control.
%
% Information about the solver itself will be available shortly.
%
% OUTPUT:
%   - def_options: Structure containing the default options of the solver.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function def_options = def_options_HMPC_ADMM()

    def_options.rho = 1e-2;
    def_options.sigma = 1e-2;
    def_options.tol_p = 1e-4;
    def_options.tol_d = 1e-4;
    def_options.k_max = 1000;
    def_options.in_engineering = false;
    def_options.debug = true;
    def_options.box_constraints = [];
    def_options.sparse = true;
    def_options.use_soc = false;
    
end
