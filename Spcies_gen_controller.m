%% Spcies_gen_controller - Generation of solvers for embedded MPC
%
% This function generated solvers for the specified MPC controller.
% For a list of the available MPC formulations, please refer to the
% documentation of the toolbox, which can be found at its GitHub
% repository https://github.com/GepocUS/Spcies.
% 
% INPUTS:
%
% All the inputs of this function are passed as name-value pairs.
% This toolbox can be used in conjunction with the GepocToolbox, 
% which is available at https://github.com/GepocUS/GepocToolbox.
% Some of the inputs may be given as classes defined in GepocToolbox.
%
%   - sys: State space model of the system. It should either be an
%          instance of the ssModel class of the GepocToolbox or a
%          structure containing:
%          - .A: matrix A of the state space model.
%          - .B: matrix B of the state space model.
%          It can optionally also contain the following fields:
%          - .xOptPoint: operating point for the system state.
%          - .uOptPoint: operating point for the system input.
%          - .LBx: Lower bound for the system state.
%          - .UBx: Upper bound for the system state.
%          - .LBu: Upper bound for the system input.
%          - .UBu: Upper bound for the system input.
%          - .Nx: Vector defining the scaling of the system state.
%          - .nU: Vector defining the scaling of the system input.
%   - param: Structure containing the ingredients of the MPC controller.
%            Its fields will depend on the chosen MPC formulation.
%            Please, refer to the documentation for additional help.
%   - controller: The sys and param arguments can be omitted and instead
%            replaced by this argument, which must be an instance of
%            a subclass of the ssMPC class of the GepocToolbox that is 
%            supported by Spcies.
%   - type: String that determines the type of MPC controller.
%           Currently, it can take the following values:
%           - 'laxMPC': Standard MPC without terminal constraint.
%           - 'MPCT': Model predictive control for tracking (using EADMM).
%           - 'MPCT_ess': MPCT using an extended state-space along ADMM.
%           - 'ellipMPC': Standard MPC with a terminal quadratic constraint.
%           If an empty string is provided, the function tries to determine
%           the type automatically based on the other arguments.
%   - options: Structure containing the options of the solver.
%              Its fields will depend on the chosen MPC formulation.
%              Please, refer to the documentation for additional help.
%              Default values are provided, so this argument is optional.
%   - spcies_options: Options of the Spcies toolbox. These control aspects
%                     such as directories where files are saved, or the
%                     name of the saved files. See Spcies_default_options
%                     for a list of the available options. Not all fields
%                     need to be provided, since default options are given.
%                     The most important of its fields is .target, which
%                     is a string containing the target embedded system
%                     of the code generation routine. Currently, the
%                     supported targets are:
%                     - 'C': For plain C. A .c and a .h file are generated.
%                     - 'Matlab' (default): A mex file is generated.
%   - The fields of the spcies_options argument can also be provided
%     individually as name-value arguments. In such a case, the name-value
%     parameter will take prevalence against the same field of the
%     spcies_options argument if both are provided.
%           
% OUTPUTS:
%   - vars: A structure containing a variety of information.
%           The exact contents will depend on the MPC formulation and target
%           system selected.
%   - This function will also save files into the directory provided in the 
%     spcies_options argument.
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
    def_spcies_options = Spcies_default_options(); % Get the default options of the toolbox
    
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
        addParameter(par, 'force_vector_rho', def_spcies_options.force_vector_rho, @(x) islogical(x) || x==1 || x==0);
    
    % Parse
    parse(par, varargin{:});
    if isempty(par.Results.spcies_options)
        spcies_options = struct('target', par.Results.target, 'save_name', par.Results.save_name,...
                            'directory', par.Results.directory, 'override', par.Results.override,...
                            'force_vector_rho', par.Results.force_vector_rho);
    else
        spcies_options = par.Results.spcies_options;
        if ~isfield(spcies_options, 'target'); spcies_options.target = par.Results.target; end
        if ~isfield(spcies_options, 'save_name'); spcies_options.save_name = par.Results.save_name; end
        if ~isfield(spcies_options, 'directory'); spcies_options.directory = par.Results.directory; end
        if ~isfield(spcies_options, 'override'); spcies_options.override = par.Results.override; end
        if ~isfield(spcies_options, 'force_vector_rho'); spcies_options.force_vector_rho = par.Results.force_vector_rho; end
    end
    
    % Use the default directory if non is provided
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
    
    %% Determine the type of the controller if it is not provided
    if isempty(par.Results.type)
        type = determine_type(controller);
    else
        type = par.Results.type;
    end

    %% Generate the controller
    if strcmp(type, 'MPCT')
        vars = MPCT.Spcies_gen_MPCT_EADMM(controller, 'options', par.Results.options, 'spcies_option', spcies_options);

    elseif strcmp(type, 'MPCT_ess')
        vars = MPCT.Spcies_gen_MPCT_extended_ss_ADMM(controller, 'options', par.Results.options, 'spcies_option', spcies_options);

    elseif strcmp(type, 'ellipMPC')
        vars = ellipMPC.Spcies_gen_ellipMPC_ADMM(controller, 'options', par.Results.options, 'spcies_option', spcies_options);

    elseif strcmp(type, 'laxMPC')
        vars = laxMPC.Spcies_gen_laxMPC_ADMM(controller, 'options', par.Results.options, 'spcies_option', spcies_options);
        
    elseif strcmp(type, 'equMPC')
        vars = equMPC.Spcies_gen_equMPC_ADMM(controller, 'options', par.Results.options, 'spcies_option', spcies_options);

    else
        error('Spcies:gen_controller:input_error', 'Type not recognized or supported');
    end
    
end

