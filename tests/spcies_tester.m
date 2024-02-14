%% spcies_tester - Tests the solvers in Spcies to see that everything works correctly
% 
% This function calls functions that test each of the solvers of the toolbox.
% Two tests are performed: a comparison between the sparse and non-sparse versions,
% and a comparison between the solution of the sparse version with the optimal
% solution (which has been obtained through other means, typically quadprog).
% 
% Information is displayed showing if the solvers work correctly by means of a 
% table that is printed on the Command Window. The columns of the table indicate:
%   - Type: MPC formulation that is being tested.
%   - Method: Method used to solve the MPC's optimization problem.
%   - SPCIES: Indicates if the sparse and non-sparse solvers give the same solution.
%       - 'Yes': They provide the same solution.
%       - 'No': They do not provide the same solution.
%   - Opt: Indicates if the solution obtained from the sparse solver is sufficiently
%          close to the optimal one (obtained through other means).
%       - 'Yes': Good suboptimal solution attained.
%       - 'No': We did not obtain a solution close to the optimal one.
%   - Flag: Flag returned by the solvers. If both solvers returned positive flags we
%           display the largest flag. If one or more returned a negative or zero flag
%           we return the smallest flag. A good result here is to see a positive integer.
%   - Error: Some exception occurred during the execution of the sparse or non-sparse solvers.
% 
% The ideal thing to see in each row is: SPCIES = 'Yes', Opt = 'Yes' and Flag >= 1.
% 
% INPUTS:
%   - verbose: (optional) Integer that determines if information is printed on screen.
%              >=1: Table is printed. 0: Table is not printed. Default: 2.
%   - debug: (optional) Boolean that determines if we catch and ignore exceptions during
%            the execution of the solvers. True: Do not ignore. False (default): Ignore.
%   - type: String or cell of strings that indicate which formulations are tested. Call also
%           be set to 'all' (default) to test all the MPC formulations.
%   - method: String or cell of strings that indicate which methods are tested. Call also
%             be set to 'all' (default) to test all the methods.
% 
% OUTPUTS:
%   - correct: Boolean indicating if all the tests were successful.
%              True: successful. False: at least one was not successful.
%   - test: Cell of structures containing all the information of each test.
%       - type: String indicating the type of MPC formulation.
%       - method: String indicating the method.
%       - fnc: String indicating the function called.
%       - gap: Structure containing the difference between solutions.
%           - spcies: Array containing the infinity norm between the solutions of the sparse
%                     and non-sparse solvers. It contains as many elements as variables are 
%                     returned by the solver.
%           - opt: Infinity norm of the difference between the solution of the sparse solver
%                  and the optimal solution.
%       - exit: Array containing the exit flags of each solver -> [sparse, non-sparse].
%       - error, spcies, opt, flag: Strings that are displayed in the table to the user.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function [correct, test] = spcies_tester(varargin)

    %% Default values
    def_verbose = 2;
    def_debug = 0;
    def_type = 'all';
    def_method = 'all';

    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'spcies_tester';
    
    % Optional
    addOptional(par, 'verbose', def_verbose, @(x) isnumeric(x) && (x>=0));
    addOptional(par, 'debug', def_debug, @(x) islogical(x) || x==1 || x==0);
    
    % Name-value parameters
    addParameter(par, 'type', def_type, @(x) ischar(x) || iscell(x));
    addParameter(par, 'method', def_method, @(x) ischar(x) || iscell(x));
    
    % Parse
    parse(par, varargin{:})
    
    % Rename
    verbose = par.Results.verbose;
    debug = par.Results.debug;
    type = par.Results.type;
    method = par.Results.method;
    
    if isempty(type); type = def_type; end
    if isempty(method); method = def_method; end
    
    %% Define the problem used for the tests. We will use the oscillating masses system
    
    p = 3; % Number of objects
    M = [1; 0.5; 1]; % Mass of each object
    K = 2*ones(p+1, 1); % Spring constant of each spring
    F = [1; zeros(p-2, 1); 1]; % Objects in which an external force is applied

    sysC = sp_utils.gen_oscillating_masses(M, K, F); % Generate the continuous-time model
    
    n = size(sysC.A, 1); % For convenience, we save the size of the state dimension
    m = size(sysC.B, 2); % and of the input dimension
    
    % Obtain the discrete-time state space model
    Ts = 0.2; % Sample time
    sysD = c2d(sysC, Ts); % Discrete-time state space model
    
    % Boounds
    LBx = -[ones(p, 1); 1000*ones(p, 1)]; % Lower bound for the state
    UBx = [0.3; 0.3; 0.3; 1000*ones(p, 1)]; % Upper bound for the state
    LBu = -[0.8; 0.8]; % Lower bound for the input
    UBu = [0.8; 0.8]; % Upper bound for the input

    % Structure containing the discrete-time system
    sys = struct('A', sysD.A, 'B', sysD.B, 'LBx', LBx, 'UBx', UBx, 'LBu', LBu, 'UBu', UBu, 'p', p, 'n', n, 'm', m);
    
    % Current state and reference
    status.x = 0.02*ones(n, 1); % Current state
    status.ur = 0.5*ones(m, 1); % We select a reference ur
    status.xr = (sys.A - eye(n))\(-sys.B*status.ur); % We compute a state reference xr that is a steady state

    %% Test each one of the solvers
    t = 1;
    test = [];
    
    % laxMPC
    if any(strcmp('laxMPC', type)) || any(strcmp('all', type))
        
        if any(strcmp('ADMM', method)) || any(strcmp('all', method))
            test{t}.type = 'lax';
            test{t}.method = 'ADMM';
            test{t}.fnc = 'test_laxMPC_ADMM';
            t = t+1;
        end
        
        if any(strcmp('FISTA', method)) || any(strcmp('all', method))
            test{t}.type = 'lax';
            test{t}.method = 'FISTA';
            test{t}.fnc = 'test_laxMPC_FISTA';
            t = t+1;
        end
        
    end
    
    % equMPC
    if any(strcmp('equMPC', type)) || any(strcmp('all', type))
        
        if any(strcmp('ADMM', method)) || any(strcmp('all', method))
            test{t}.type = 'equ';
            test{t}.method = 'ADMM';
            test{t}.fnc = 'test_equMPC_ADMM';
            t = t+1;
        end
        
        if any(strcmp('FISTA', method)) || any(strcmp('all', method))
            test{t}.type = 'equ';
            test{t}.method = 'FISTA';
            test{t}.fnc = 'test_equMPC_FISTA';
            t = t+1;
        end
        
    end
    
    % MPCT
    if any(strcmp('MPCT', type)) || any(strcmp('all', type))
    
        if any(strcmp('EADMM', method)) || any(strcmp('all', method))
            test{t}.type = 'MPCT';
            test{t}.method = 'EADMM';
            test{t}.fnc = 'test_MPCT_EADMM';
            t = t+1;
        end
        
        if any(strcmp('ADMM', method)) || any(strcmp('all', method))
            test{t}.type = 'MPCT';
            test{t}.method = 'ADMM';
            test{t}.fnc = 'test_MPCT_ADMM';
            t = t+1;
        end
        
    end
    
    % ellipMPC
    if any(strcmp('ellipMPC', type)) || any(strcmp('all', type))
        
        if any(strcmp('ADMM', method)) || any(strcmp('all', method))
            test{t}.type = 'ellip';
            test{t}.method = 'ADMM';
            test{t}.fnc = 'test_ellipMPC_ADMM';
            t = t+1;
        end
        
        if any(strcmp('SOC', method)) || any(strcmp('all', method))
            test{t}.type = 'ellip';
            test{t}.method = 'SOC';
            test{t}.fnc = 'test_ellipMPC_ADMM_soc';
            t = t+1;
        end
        
    end
    
    % HMPC
    if any(strcmp('HMPC', type)) || any(strcmp('all', type))
        
        if any(strcmp('ADMM', method)) || any(strcmp('all', method))
            test{t}.type = 'HMPC';
            test{t}.method = 'ADMM';
            test{t}.fnc = 'test_HMPC_ADMM';
            t = t+1;
        end
        
        if any(strcmp('ADMM_split', method)) || any(strcmp('all', method))
            test{t}.type = 'HMPC';
            test{t}.method = 'ADMM_s';
            test{t}.fnc = 'test_HMPC_ADMM_s';
            t = t+1;
        end
        
        if any(strcmp('SADMM_split', method)) || any(strcmp('all', method))
            test{t}.type = 'HMPC';
            test{t}.method = 'SADMM_s';
            test{t}.fnc = 'test_HMPC_SADMM_s';
            t = t+1;
        end
        
    end
    
    spcies_clear(); % Clear solvers
    
    if isempty(test)
        
        if verbose > 0
            warning('SPCIES tester: No matching solvers.')
        end
        
    else
    
        if verbose > 0
            fprintf('Starting Spcies testing routines.\n')
        end
        for i = 1:length(test)

            if verbose > 0
                fprintf('Starting test %d out of %d: %s using %s.\n', i, t-1, test{i}.type, test{i}.method);
            end

            if ~debug
                try
                    [test{i}.gap, test{i}.exit] = eval([test{i}.fnc '(sys, status)']);
                    test{i}.error = 'No';
                catch
                    test{i}.error = 'Yes';
                end
            else
                [test{i}.gap, test{i}.exit] = eval([test{i}.fnc '(sys, status)']);
                test{i}.error = 'No';
            end

        end
        
    end
    
    %% Check results
    tol_spcies = 1e-10; % Maximum allowed difference between the sparse and non-sparse SPCIES solvers
    tol_opt = 1e-4; % Maximum allowed difference between the SPCIES solver and optimal solution
    
    correct = true;
    for i = 1:length(test)
        
        if strcmp(test{i}.error, 'Yes')
            correct = false;
            test{i}.spcies = '??';
            test{i}.opt = '??';
            test{i}.flag = '??';
        else
            
            % Check sparse and non-sparse solvers
            if max(test{i}.gap.spcies) > tol_spcies
                correct = false;
                test{i}.spcies = 'No';
            else
                test{i}.spcies = 'Yes';
            end

            % Check against the optimal solution
            if max(test{i}.gap.opt) > tol_opt
                correct = false;
                test{i}.opt = 'No';
            else
                test{i}.opt = 'Yes';
            end
            
            if any(test{i}.exit <= 0)
                correct = false;
                test{i}.flag = num2str(min(test{i}.exit));
            else
                test{i}.flag = num2str(max(test{i}.exit));
            end
            
        end
        
    end
    
    %% Print information
    if verbose > 0
        
        fprintf('Result:\n');
        fprintf('----------------------------------------------\n');
        fprintf('Type\tMethod\tSPCIES\tOpt\tFlag\tError\n');
        fprintf('----------------------------------------------\n');
        for i = 1:length(test)
            fprintf('%s\t%s\t%s\t%s\t%s\t%s\n', test{i}.type, test{i}.method, test{i}.spcies, test{i}.opt, test{i}.flag, test{i}.error);
        end
        fprintf('----------------------------------------------\n\n');
        
        if correct
            fprintf('SPCIES test result: Positive.\n');
        else
            fprintf('SPCIES test result: Negative.\n');
            fprintf('Something was not OK. Please check the entries of the table.\n');
        end
        
    end

end
