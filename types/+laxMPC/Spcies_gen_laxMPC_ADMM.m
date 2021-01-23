%% Spcies_gen_laxMPC_ADMM - Generate the ADMM-based solver for the lax MPC formulation
% 
% Information about this formulation and the solver  can be found at:
% 
% P. Krupa, D. Limon, T. Alamo, "Implementation of model predictive control in
% programmable logic controllers", Transactions on Control Systems Technology, 2020.
% 
% INPUTS (all inputs are name-value pairs, except 'controller'):
%   - controller: Contains the information of the controller.
%   - target: target embedded system that the controller is generated for.
%   - options: structure containing options of the EADMM solver.
%   - save_name: string that determines the name of any files saved to the current directory.
%   - override: Boolean that determines is the controller is overriden if the file already exists.
% 
% OUTPUTS:
%   - vars: Structure containing the necessary variables to run the solver
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function vars = Spcies_gen_laxMPC_ADMM(varargin)
    
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
    par.FunctionName = 'Spcies_gen_laxMPC_ADMM';
    
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
    vars = laxMPC.Spcies_compute_laxMPC_ADMM_ingredients(par.Results.controller, options, par.Results.spcies_options);
    
    %% Call the funciton that constructs the controller
    if strcmp(par.Results.spcies_options.target, 'C') 
        laxMPC.gen_laxMPC_ADMM_C(vars, options, par.Results.spcies_options);
    elseif strcmp(par.Results.spcies_options.target, 'Matlab')
        laxMPC.gen_laxMPC_ADMM_Matlab(vars, options, par.Results.spcies_options);
    else
        error('Target not recognized or supported');
    end
    
end
