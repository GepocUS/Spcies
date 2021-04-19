%% Spcies class for defining an instance of a problem, i.e., the information required to define the desired solver.

classdef Spcies_problem
    
    %% PROPERTIES
    properties
        controller % Information of the controller to be generated
        solver_options % Options of the solver
        options % Options of the toolbox
    end
    
    methods
        
        function self = Spcies_problem(varargin)
            %% Default values
            def_sys = []; % Default value for the sys argument
            def_param = []; % Default value for the param argument
            def_controller = []; % Default value for the controller argument
            del_solver_options = [];% Default options of the solver
            def_options = Spcies_default_options();  % Get the default options of the toolbox

            %% Parser
            par = inputParser;
            par.CaseSensitive = false;
            par.FunctionName = 'Spcies_constructor';
            
            % Name-value parameters
            addParameter(par, 'sys', def_sys, @(x) isa(x, 'ss') || isa(x, 'ssModel') || isstruct(x));
            addParameter(par, 'param', def_param, @(x) isstruct(x));
            addParameter(par, 'controller', def_controller, @(x) isa(x, 'ssMPC'));
            addParameter(par, 'solver_options', del_solver_options, @(x) isstruct(x)); 
            addParameter(par, 'options', [], @(x) isstruct(x));
                addParameter(par, 'method', def_options.method, @(x) ischar(x));
                addParameter(par, 'subclass', def_options.subclass, @(x) ischar(x));
                addParameter(par, 'type', def_options.type, @(x) ischar(x));
                addParameter(par, 'platform', def_options.type, @(x) ischar(x));
                addParameter(par, 'save_name', def_options.save_name, @(x) ischar(x));
                addParameter(par, 'directory', def_options.directory, @(x) ischar(x));
                addParameter(par, 'override', def_options.override, @(x) islogical(x) || x==1 || x==0);
                addParameter(par, 'force_vector_rho', def_options.force_vector_rho, @(x) islogical(x) || x==1 || x==0);
                addParameter(par, 'save', def_options.save);
                
            % Parse
            parse(par, varargin{:});
            
            %% Process the arguments
            
            % Get controller
            if isempty(par.Results.controller)
                self.controller.sys = par.Results.sys;
                self.controller.param = par.Results.param;
            else
                self.controller = par.Results.controller;
            end
            
            % Get solver_options
            self.solver_options = par.Results.solver_options;
            
            % Toolbox options
            fn_options = fieldnames(def_options); % Get the names of the fields in par.Results
            
            if isempty(par.Results.options)
                % It no options argument was provided, get the individual ones (some may be defaults)
                for i = 1:numel(def_options)    
                    self.options.(fn_options{i}) = par.Results.(fn_options{i});
                end
                
            else
                % It the options argument was provided use this instead
                self.options = par.Results.options;
                
                % Then add any missing fields
                for i = 1:numel(fn_options)
                    if ~isfield(self.options, fn_options{i})
                        self.options.(fn_options{i}) = par.Results.(fn_options{i});
                    end
                end

            end
            
            if isempty(self.options.type)
                self.options.type = determine_type(controller);
            end
            
        end         
        
    end
    
end
