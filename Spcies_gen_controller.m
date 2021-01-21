%% Spcies_gen_controller - Generation of embedded MPC controllers
% 
% This function is the main Spcies function. It generates the controller designated by
% input 'type' for the target embedded system designated by input 'target'.
% This function tipically saves files to the current working directory.
% 
% INPUTS (all inputs are name-value pairs):
%   - type: type of controller generated. See documentation for a list and description of the controllers available.
%   - target: target embedded system that the controller is generated for. See documentation for a list of supported ones.
%   - sys: model of the system.
%   - param: structure containing  parameters of the controller.
%            Each controller will require different parameters. See documentation for which ones.
%   - options: structure containing options of the solver.
%              Each controller/solver will have different ones. See documantation for which ones.
%   - save_name: string that determines the name of any files saved to the current directory.
%   - override: Boolean that determines is the controller is overriden if the file already exists.
% 
% OUTPUTS:
%   - vars: Structure containing a variety of information
%   - The function may also create files in the working directory
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function vars = Spcies_gen_controller(varargin)
    import utils.determine_type;

    %% Default values
    def_sys = []; % Default value for the sys argument
    def_param = []; % Default value for the param argument
    def_controller = []; % Default value for the controller argument
    def_type = ''; % Default type
    def_options = []; % Default value of the options argument
    def_spcies_options = Spcies_default_options();
    
    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'Spcies_gen_controller';
    
    % Name-value parameters
    addParameter(par, 'sys', def_sys, @(x) isa(x, 'ss') || isa(x, 'ssModel') || isstruct(x));
    addParameter(par, 'param', def_param, @(x) isstruct(x));
    addParameter(par, 'controller', def_controller, @(x) isa(x, 'ssMPC'));
    addParameter(par, 'type', def_type, @(x) ischar(x));
    addParameter(par, 'options', def_options, @(x) isstruct(x)); 
    % Options of the toolbox (can be given in the structure spcies_options or individually)
    addParameter(par, 'spcies_options', [], @(x) isstruct(x));
        addParameter(par, 'target', def_spcies_options.target, @(x) ischar(x));
        addParameter(par, 'save_name', def_spcies_options.save_name, @(x) ischar(x));
        addParameter(par, 'directory', def_spcies_options.directory, @(x) ischar(x));
        addParameter(par, 'override', def_spcies_options.override, @(x) islogical(x) || x==1 || x==0);
    
    % Parse
    parse(par, varargin{:});
    if isempty(par.Results.spcies_options)
        spcies_options = struct('target', par.Results.target, 'save_name', par.Results.save_name,...
                            'directory', par.Results.directory, 'override', par.Results.override);
    else
        spcies_options = par.Results.spcies_options;
    end
    
    % Check arguments
    if isempty(spcies_options.directory)
        spcies_options.directory = def_spcies_options.directory;
    end
    
    %% Create the controller structure, which either contains the param and sys structures or the controller object
    if isempty(par.Results.controller)
        controller.sys = par.Results.sys;
        controller.param = par.Results.param;
    else
        controller = par.Results.controller;
    end
    
    %% Determine the type of the controller
    if isempty(par.Results.type)
        type = determine_type(controller);
    else
        type = par.Results.type;
    end

    %% Generate the controller
    if strcmp(type, 'MPCT')
        vars = MPCT.Spcies_gen_MPCT_EADMM(controller, 'options', par.Results.options, 'spcies_option', spcies_options);
    elseif strcmp(type, 'ellipMPC')
        vars = ellipMPC.Spcies_gen_ellipMPC_ADMM(controller, 'options', par.Results.options, 'spcies_option', spcies_options);
    elseif strcmp(type, 'laxMPC')
        vars = laxMPC.Spcies_gen_laxMPC_ADMM(controller, 'options', par.Results.options, 'spcies_option', spcies_options);
    else
        error('Spcies:gen_controller:input_error', 'Type not recognized or supported');
    end
    
end
