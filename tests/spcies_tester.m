%% spcies_tester - Tests the solvers in Spcies to see that everything works correctly

function correct = spcies_tester(verbose, debug)
    if nargin == 0
        verbose = true;
        debug = 0;
    elseif nargin == 1
        debug = 0;
    end
    
    %% Define the problem used for the tests. We will use the oscillating masses system
    root_path = spcies_get_root_directory;
    addpath([root_path '/examples']);
    
    p = 3; % Number of objects
    M = [1; 0.5; 1]; % Mass of each object
    K = 2*ones(p+1, 1); % Spring constant of each spring
    F = [1; zeros(p-2, 1); 1]; % Objects in which an external force is applied

    sysC = utils.gen_oscillating_masses(M, K, F); % Generate the continuous-time model
    
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
    
    test{t}.type = 'laxMPC';
    test{t}.method = 'ADMM ';
    test{t}.fnc = 'test_laxMPC_ADMM';
    t = t+1;
    
    test{t}.type = 'laxMPC';
    test{t}.method = 'FISTA';
    test{t}.fnc = 'test_laxMPC_FISTA';
    t = t+1;
    
%     test{t}.type = 'equMPC';
%     test{t}.method = 'ADMM';
%     test{t}.fnc = 'test_equMPC_ADMM';
%     t = t+1;
%     
%     test{t}.type = 'equMPC';
%     test{t}.method = 'FISTA';
%     test{t}.fnc = 'test_equMPC_FISTA';
%     t = t+1;
    
%     test{t}.name = 'ellipMPC_ADMM';
%     test{t}.fnc = 'test_ellipMPC_ADMM';
%     t = t+1;
%     
%     test{t}.name = 'MPCT_ADMM';
%     test{t}.fnc = 'test_MPCT_EADMM';
%     t = t+1;
%     
%     test{t}.name = 'MPCT_EADMM_cs';
%     test{t}.fnc = 'test_MPCT_ADMM_cs';
%     t = t+1;

    spcies_clear(); % Clear solvers
    
    if verbose
        fprintf('Starting Spcies testing routines.\n')
    end
    for i = 1:length(test)
        
        fprintf('Starting test %d out of %d: %s using %s.\n', i, t-1, test{i}.type, test{i}.method);
        
        if ~debug
            try
                test{i}.gap = eval([test{i}.fnc '(sys, status)']);
                test{i}.error = 'No';
            catch
                test{i}.error = 'Yes';
            end
        else
            test{i}.gap = eval([test{i}.fnc '(sys, status)']);
            test{i}.error = 'No';
        end
        
    end
    
    %% Check results
    tol_spcies = 1e-10; % Maximum allowed difference between the sparse and non-sparse SPCIES solvers
    tol_opt = 1e-4; % Maximum allowed difference between the SPCIES solver and optimal solution
    
    correct = true;
    for i = 1:length(test)
        
        if strcmp(test{i}.error, 'Yes')
            correct = false;
            test{i}.spcies = '?? ';
            test{i}.opt = '?? ';
        else
            
            % Check sparse and non-sparse solvers
            if max(test{i}.gap.spcies) > tol_spcies
                correct = false;
                test{i}.spcies = 'No ';
            else
                test{i}.spcies = 'Yes';
            end

            % Check against the optimal solution
            if max(test{i}.gap.opt) > tol_opt
                correct = false;
                test{i}.opt = 'No ';
            else
                test{i}.opt = 'Yes';
            end
            
        end
        
    end
    
    %% Print information
    if verbose
        
        fprintf('---------------------------------------\n');
        fprintf('Type      Method  SPCIES  Opt   Error\n');
        fprintf('---------------------------------------\n');
        for i = 1:length(test)
            fprintf('%s    %s   %s     %s   %s\n', test{i}.type, test{i}.method, test{i}.spcies, test{i}.opt, test{i}.error);
        end
        fprintf('---------------------------------------\n\n');
        
        if correct
            fprintf('SPCIES test result: Positive.\n');
        else
            fprintf('SPCIES test result: Negative.\n');
            fprintf('Something was not ok. Please check the entries of the table.\n');
        end
        
    end

end
