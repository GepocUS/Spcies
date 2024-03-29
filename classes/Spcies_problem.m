%% Spcies_problem - Spcies class for defining the code generation problem
% 
% This class contains the information required to define the desired controller
% to be generated. It is generated when calling the spcies_gen_controller
% function and is passed to most of the functions in Spcies during code generation.
% 
% Its constructor receives the exact same arguments as spcies_gen_controller, so
% we refer the user to the documentation of that function.
% 
% This class is part of Spcies: https://github.com/GepocUS/Spcies
% 

classdef Spcies_problem
    
    %% PROPERTIES
    properties
        controller % Information of the controller to be generated
        % solver_options % Options of the solver
        options % Options of the toolbox. Instance of Spcies_options
    end
    
    methods

        function self = Spcies_problem(sys, param, options)
            self.controller.sys = sys;
            self.controller.param = param;
            self.options = options;
        end

        function new = copy(self)
            new = Spcies_problem(self.controller.sys, self.controller.param, copy(self.options));
        end
        
        %% CONSTRUCTOR
    %     function self = Spcies_problem(varargin)
    %         %% Default values
    %         def_sys = []; % Default value for the sys argument
    %         def_param = []; % Default value for the param argument
    %         def_controller = []; % Default value for the controller argument
    %         del_solver_options = [];% Default options of the solver
    %         def_options = sp_utils.default_options();  % Get the default options of the toolbox
    %         
    %         %% Parser
    %         par = inputParser;
    %         par.CaseSensitive = false;
    %         par.FunctionName = 'Spcies_constructor';
    %         
    %         % Name-value parameters
    %         addParameter(par, 'sys', def_sys, @(x) isa(x, 'ss') || isa(x, 'ssModel') || isstruct(x));
    %         addParameter(par, 'param', def_param, @(x) isstruct(x));
    %         addParameter(par, 'controller', def_controller, @(x) isa(x, 'ssMPC'));
    %         addParameter(par, 'solver_options', del_solver_options, @(x) isstruct(x)); 
    %         addParameter(par, 'options', [], @(x) isstruct(x));
    %             addParameter(par, 'formulation', def_options.formulation, @(x) ischar(x));
    %             addParameter(par, 'method', def_options.method, @(x) ischar(x));
    %             addParameter(par, 'submethod', def_options.submethod, @(x) ischar(x));
    %             addParameter(par, 'platform', def_options.platform, @(x) ischar(x));
    %             addParameter(par, 'save_name', def_options.save_name, @(x) ischar(x));
    %             addParameter(par, 'directory', def_options.directory, @(x) ischar(x));
    %             addParameter(par, 'override', def_options.override, @(x) islogical(x) || x==1 || x==0);
    %             addParameter(par, 'const_are_static', def_options.const_are_static, @(x) islogical(x) || x==1 || x==0);
    %             addParameter(par, 'precision', def_options.precision, @(x) ischar(x));
    %             addParameter(par, 'save', def_options.save);
    %             addParameter(par, 'time', def_options.time, @(x) islogical(x) || x==1 || x==0);
    %
    %             % Deprecated:
    %             addParameter(par, 'type', NaN, @(x) ischar(x)); % Now 'formulation'
    %             addParameter(par, 'subclass', NaN, @(x) ischar(x)); % Now 'submethod'
    %             
    %         % Parse
    %         parse(par, varargin{:});
    %         
    %         %% Process the arguments
    %         
    %         % Get controller
    %         if isempty(par.Results.controller)
    %             self.controller.sys = par.Results.sys;
    %             self.controller.param = par.Results.param;
    %         else
    %             self.controller = par.Results.controller;
    %         end
    %         
    %         % Get solver_options
    %         self.solver_options = par.Results.solver_options;
    %         
    %         % Toolbox options
    %         fn_options = fieldnames(def_options); % Get the names of the fields in par.Results
    %         
    %         if isempty(par.Results.options)
    %             % It no options argument was provided, get the individual ones (some may be defaults)
    %             for i = 1:numel(fn_options)    
    %                 self.options.(fn_options{i}) = par.Results.(fn_options{i});
    %             end
    %             
    %         else
    %             % It the options argument was provided use this instead
    %             self.options = par.Results.options;
    %             
    %             % Then add any missing fields
    %             for i = 1:numel(fn_options)
    %                 if ~isfield(self.options, fn_options{i})
    %                     self.options.(fn_options{i}) = par.Results.(fn_options{i});
    %                 end
    %             end
    %
    %         end
    %
    %         % Check deprecated options
    %         if isfield(self.options, 'type')
    %             warning("Spcies: 'type' is now deprecated. Please use 'formulation' instead. 'type' will be removed in some future release.")
    %             self.options.formulation = self.options.type;
    %         end
    %         if isfield(self.options, 'subclass')
    %             warning("Spcies: 'subclass' is now deprecated. Please use 'submethod' instead. 'subclass' will be removed in some future release.")
    %             self.options.submethod = self.options.subclass;
    %         end
    %         if ~isnan(par.Results.type)
    %             warning("Spcies: 'type' is now deprecated. Please use 'formulation' instead. 'type' will be removed in some future release.")
    %             self.options.formulation = par.Results.type;
    %         end
    %         if ~isnan(par.Results.subclass)
    %             warning("Spcies: 'subclass' is now deprecated. Please use 'submethod' instead. 'subclass' will be removed in some future release.")
    %             self.options.submethod = par.Results.subclass;
    %         end
    %         
    %         if isempty(self.options.formulation)
    %             self.options.formulation = sp_utils.determine_formulation(self.controller);
    %         end
    %
    %         
    %     end         
        
    end
    
end
