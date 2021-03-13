%% Spcies_gen_MPCT_extended_ss_ADMM - Generate the MPCT solver using ADMM on an extended state space
% 
% The solver extends the state and constol inputs by adding the artificial reference to them.
% Currently, there is no additional documentation available for the solver.
% 
% INPUTS (all inputs are name-value pairs, except 'controller'):
%   - controller: Contains the information of the controller.
%   - target: target embedded system that the controller is generated for.
%   - options: structure containing options of the EADMM solver.
%   - save_name: string that determines the name of any files saved to the current directory.
%   - override: Boolean that determines is the controller is overriden if the file already exists.
% 
% OUTPUTS:
%   - vars: Structure containing a variety of information
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function vars = Spcies_gen_MPCT_extended_ss_ADMM(varargin)

    %% Default values
    def_spcies_opt = Spcies_default_options();
    % Default values of the options argumenr
    def_rho = 1e-2;
    def_epsilon_x = 1e-6;
    def_epsilon_u = 1e-6;
    def_inf_bound = 1e6;
    def_tol = 1e-4;
    def_k_max = 1000;
    def_in_engineering = false;
    def_debug = false;
    def_const_are_static = true;
    def_options = struct('rho', def_rho, 'tol', def_tol, 'k_max', def_k_max, 'in_engineering', def_in_engineering,...
                         'epsilon_x', def_epsilon_x, 'epsilon_u', def_epsilon_u, 'inf_bound', def_inf_bound,...
                         'debug', def_debug, 'const_are_static', def_const_are_static);
                     
      %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'Spcies_gen_MPCT_EADMM';
    
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
    if ~isfield(options, 'epsilon_x'); options.epsilon_x = def_epsilon_x; end
    if ~isfield(options, 'epsilon_u'); options.epsilon_u = def_epsilon_u; end
    if ~isfield(options, 'inf_bound'); options.inf_bound = def_inf_bound; end
    if ~isfield(options, 'tol'); options.tol = def_tol; end
    if ~isfield(options, 'k_max'); options.k_max = def_k_max; end
    if ~isfield(options, 'in_engineering'); options.in_engineering = def_in_engineering; end
    if ~isfield(options, 'debug'); options.debug = def_debug; end
    if ~isfield(options, 'const_are_static'); options.const_are_static = def_const_are_static; end
                     
    %% Compute the ingredients of the controller
    vars = MPCT.Spcies_compute_MPCT_extended_ss_ADMM_ingredients(par.Results.controller, options, par.Results.spcies_options);
    
    %% Call the funciton that constructs the controller
    if strcmp(par.Results.spcies_options.target, 'C') 
        MPCT.gen_MPCT_extended_ss_ADMM_C(vars, options, par.Results.spcies_options);
    elseif strcmp(par.Results.spcies_options.target, 'Matlab')
        MPCT.gen_MPCT_extended_ss_ADMM_Matlab(vars, options, par.Results.spcies_options);
    else
        error('Target not recognized or supported');
    end
    
end
