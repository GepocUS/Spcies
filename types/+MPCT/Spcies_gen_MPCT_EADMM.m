%% Spcies_gen_MPCT_EADMM - Generate the MPCT solver based on the EADMM alggorithm
% 
% Information about this formulaiton and the solver  can be found at:
% 
% "Implementation of model predictive control for tracking in embedded systems
% using a sparse extended ADMM algorithm", by P. Krupa, I. Alvarado, D. Limon
% and T. Alamo, arXiv preprint: 2008:09071v2, 2020.
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

function vars = Spcies_gen_MPCT_EADMM(varargin)

    %% Default values
    def_target = 'Matlab'; % Default target
    def_save_name = ''; % Default value of the save_name argument
    % Default values of the options argumenr
    def_rho_base = 3;
    def_rho_mult = 20;
    def_epsilon_x = 1e-6;
    def_epsilon_u = 1e-6;
    def_inf_bound = 1e6;
    def_tol = 1e-4;
    def_k_max = 1000;
    def_k_inc_max = [];
    def_in_engineering = false;
    def_debug = false;
    def_options = struct('rho_base', def_rho_base, 'rho_mult', def_rho_mult, 'epsilon_x', def_epsilon_x,...
                         'epsilon_u', def_epsilon_u, 'inf_bound', def_inf_bound, 'tol', def_tol, 'k_max', def_k_max,...
                         'in_engineering', def_in_engineering, 'debug', def_debug, 'k_inc_max', def_k_inc_max);
    
    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'Spcies_gen_MPCT_EADMM';
    
    % Required
    addRequired(par, 'controller', @(x) isa(x, 'ssMPC') || isstruct(x));
    
    % Name-value parameters
    addParameter(par, 'target', def_target, @(x) ischar(x));
    addParameter(par, 'save_name', def_save_name, @(x) ischar(x));
    addParameter(par, 'options', def_options, @(x) isstruct(x) || isempty(x));
    addParameter(par, 'override', [], @(x) islogical(x) || x==1 || x==0);
    
    % Parse
    parse(par, varargin{:})
    
    % Set default options if options is empty
    if isempty(par.Results.options)
        options = def_options;
    else
        options = par.Results.options;
    end
    if ~isfield(options, 'rho_base'); options.rho_base = def_rho_base; end
    if ~isfield(options, 'rho_mult'); options.rho_mult = def_rho_mult; end
    if ~isfield(options, 'epsilon_x'); options.epsilon_x = def_epsilon_x; end
    if ~isfield(options, 'epsilon_u'); options.epsilon_u = def_epsilon_u; end
    if ~isfield(options, 'inf_bound'); options.inf_bound = def_inf_bound; end
    if ~isfield(options, 'tol'); options.tol = def_tol; end
    if ~isfield(options, 'k_max'); options.k_max = def_k_max; end
    if ~isfield(options, 'in_engineering'); options.in_engineering = def_in_engineering; end
    if ~isfield(options, 'debug'); options.debug = def_debug; end
    if ~isfield(options, 'k_inc_max'); options.k_inc_max = def_k_inc_max; end
    if isempty(options.k_inc_max); options.k_inc_max = options.k_inc; end
    
    %% Compute the ingredients of the controller
    vars = MPCT.Spcies_compute_MPCT_EADMM_ingredients(sys, param, options);
    
    %% Call the funciton that constructs the controller
    if strcmp(target, 'Arduino')
        MPCT.gen_MPCT_EADMM_Arduino(vars, options, par.Results.save_name, par.Results.override); 
    elseif strcmp(target, 'Unity')
        MPCT.gen_MPCT_EADMM_Unity(vars, options, par.Results.save_name, par.Results.override);      
    else
        if ~strcmp(target, 'Matlab')
            error('Target not recognized or supported');
        end
    end
    
end
