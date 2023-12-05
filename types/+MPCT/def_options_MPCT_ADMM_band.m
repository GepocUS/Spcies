%% def_options_MPCT_ADMM_band
%
% Returns the default options for the ADMM_band-based solver for the MPCT formulation
% 
% Information about this formulation and the solver can be found at:
% 
% TODO: Poner nombre del artículo si se publica
% 
% OUTPUT:
%   - def_options: Structure containing the default options of the solver
% 

function def_options = def_options_MPCT_ADMM_band()

    def_options.rho = 1e-2;
    def_options.epsilon_x = 1e-6;
    def_options.epsilon_u = 1e-6;
    def_options.inf_bound = 1e6;
    def_options.tol_p = 1e-4;
    def_options.tol_d = 1e-4;
    def_options.k_max = 1000;
    def_options.in_engineering = false;
    def_options.debug = false;
    def_options.force_vector_rho = false; % If true, forces the penalty parameter rho to be defined as a vector

end

