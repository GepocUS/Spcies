%% Spcies_gen_ellipMPC_ADMM - Generate the RMPC solver based on the ADMM alggorithm
% This version uses the projection algorithm onto the ellipsoid

% Author: Pablo Krupa (pkrupa@us.es)
% 
% Changelog: 
%   v0.1 (2020/10/16): Initial commit version
%   v0.2 (2020/12/07): Added parser and improved overall usability
%

function vars = Spcies_gen_ellipMPC_ADMM(varargin)

    %% Default values
    def_target = 'Matlab'; % Default target
    def_save_name = ''; % Default value of the save_name argument
    % Default values of the options argumenr
    def_rho = 1e-2;
    def_tol = 1e-4;
    def_k_max = 1000;
    def_in_engineering = false;
    def_debug = false;
    def_options = struct('rho', def_rho, 'tol', def_tol, 'k_max', def_k_max, 'in_engineering', def_in_engineering, 'debug', def_debug);
    
    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'Spcies_gen_ellipMPC_ADMM';
    
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
    
    %% Compute the ingredients of the controller
    vars = ellipMPC.Spcies_compute_ellipMPC_ADMM_ingredients(par.Results.controller, options);
    
    %% Call the funciton that constructs the controller
    if strcmp(par.Results.target, 'C')
        ellipMPC.gen_ellipMPC_ADMM_C(vars, options, par.Results.save_name, par.Results.override);   
    else
        if ~strcmp(par.Results.target, 'Matlab')
            error('Target not recognized or supported');
        end
    end
    
end
