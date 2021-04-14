%% Spcies_gen_equMPC_ADMM - Generate the ADMM-based solver for the equality MPC formulation
% 
% This function is called by Spcies_gen_controller if the 'type' argument is set to 'equMPC'.
% 
% Information about this formulation and the solver can be found at:
% 
% P. Krupa, D. Limon, T. Alamo, "Implementation of model predictive control in
% programmable logic controllers", Transactions on Control Systems Technology, 2020.
% 
% Specifically, this formulation is given in equation (8) of the above reference.
%
% INPUTS (all inputs are name-value pairs, except 'controller'):
%   - controller: Structure containing the information of the controller.
%                 It must contain the fields .sys and .param.
%                 - .sys: Structure containing the state space model (see Spcies_gen_controller).
%                 - .param: Structure containing the ingredients of the controller:
%                           - .Q: Cost function matrix Q.
%                           - .R: Cost function matrix R.
%                           - .T: Cost function matrix T.
%                           - .N: Prediction horizon.
%   - options: Structure containing options of the ADMM solver.
%              - .rho: Penalty parameter. Scalar of vector. Defaults to the scalar 1e-2.
%                      If a vector is provided, it must have the same dimensions as the decision variables.
%              - .tol: Exit tolerance of the solver. Defaults to 1e-4.
%              - .k_max: Maximum number of iterations of the solver. Defaults to 1000.
%              - .in_engineering: Boolean that determines if the arguments of the solver are given in
%                                 engineering units (true) or incremental ones (false - default).
%              - .debug: Boolean that determines if debugging options are enables in the solver.
%                        Defaults to false.
%              - .const_are_static: Boolean that determines if constants are defined as static variables.
%                                   Defaults to true.
%   - spcies_options: Structure containing the options of the toolbox. See Spcies_default_options.
% 
% OUTPUTS:
%   - vars: Structure containing the necessary variables to run the solver.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function vars = Spcies_gen_equMPC_ADMM(varargin)
    
    %% Default values
    def_spcies_opt = Spcies_default_options();
    % Default values of the options argumenr
    def_rho = 1e-2;
    def_tol = 1e-4;
    def_k_max = 1000;
    def_in_engineering = false;
    def_debug = false;
    def_const_are_static = true;
    def_options = struct('rho', def_rho, 'tol', def_tol, 'k_max', def_k_max, 'in_engineering',...
                         def_in_engineering, 'debug', def_debug, 'const_are_static', def_const_are_static);
    
    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'Spcies_gen_equMPC_ADMM';
    
    % Required
    addRequired(par, 'controller', @(x) isa(x, 'ssMPC') || isstruct(x));
    
    % Name-value parameters
    addOptional(par, 'options', def_options, @(x) isstruct(x) || isempty(x));
    addOptional(par, 'spcies_options', def_spcies_opt, @(x) isstruct(x) || isempty(x));
    
    % Parse
    parse(par, varargin{:})
    
    % Set default options if options is empty
    if isempty(par.Results.options)
        options = def_options;
    else
        options = par.Results.options;
    end
    if ~isfield(options, 'rho'); options.rho = def_rho; end
    if ~isfield(options, 'tol'); options.tol = def_tol; end
    if ~isfield(options, 'k_max'); options.k_max = def_k_max; end
    if ~isfield(options, 'in_engineering'); options.in_engineering = def_in_engineering; end
    if ~isfield(options, 'debug'); options.debug = def_debug; end
    if ~isfield(options, 'const_are_static'); options.const_are_static = def_const_are_static; end
    
    %% Compute the ingredients of the controller
    vars = equMPC.Spcies_compute_equMPC_ADMM_ingredients(par.Results.controller, options, par.Results.spcies_options);
    
    %% Call the function that constructs the controller
    if strcmp(par.Results.spcies_options.target, 'C') 
        equMPC.gen_equMPC_ADMM_C(vars, options, par.Results.spcies_options);

    elseif strcmp(par.Results.spcies_options.target, 'Matlab')
        equMPC.gen_equMPC_ADMM_Matlab(vars, options, par.Results.spcies_options);

    else
        error('Target not recognized or supported');
    end
    
end

