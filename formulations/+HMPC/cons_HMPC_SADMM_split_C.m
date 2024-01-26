%% cons_HMPC_SADMM_split_C
%
% Generates the constructor for C of the HMPC formulation based on SADMM which splits
% the decision variables into (z, s) and (z_hat, s_hat).
% 
% Information about this formulation can be found at:
%
% P. Krupa, D. Limon, and T. Alamo, “Harmonic based model predictive
% control for set-point tracking", IEEE Transactions on Automatic Control.
% 
% INPUTS:
%   - recipe: An instance of the Spcies_problem class. Its properties must contain:
%       - controller: Structure containing the information of the controller.
%                 It must contain the fields .sys and .param.
%                 - .sys: Structure containing the state space model (see Spcies_gen_controller).
%                 - .param: Structure containing the ingredients of the controller:
%                           - .Q: Cost function matrix Q.
%                           - .R: Cost function matrix R.
%                           - .Te: Cost function matrix Te.
%                           - .Th: Cost function matrix Th.
%                           - .Se: Cost function matrix Se.
%                           - .Sh: Cost function matrix Sh.
%                           - .N: Prediction horizon.
%                           - .w: Base frequency.
%       - options: Instance of Spcies_options. Solver specific options are:
%              - .sigma: Penalty parameter sigma. Scalar.
%              - .rho: Penalty parameter rho. Scalar.
%              - .alpha: Relaxation parameter of the SADMM algorithm.
%              - .tol_p: Primal exit tolerance of the solver.
%              - .tol_d: Dual exit tolerance of the solver.
%              - .k_max: Maximum number of iterations of the solver.
%              - sparse: Boolean that determines if the system os equations is solved using the sparse
%                        LDL approach (if true) or the non-sparse approach (if false).
%              - use_soc: Boolean that determines if the SOC constraints are grouped into the "diamond"
%                         sets (if false) or if they are all considered (if true).
% 
% OUTPUTS:
%   - constructor: An instance of the Spcies_constructor class ready for file generation.
%                  
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function constructor = cons_HMPC_SADMM_split_C(recipe)

    constructor = HMPC.cons_HMPC_ADMM_split_C(recipe);
    
end
