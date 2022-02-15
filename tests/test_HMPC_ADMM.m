%% Test for the HMPC formulation using ADMM

function [gap, exit] = test_HMPC_ADMM(sys, status)
    
    % Solver options
    solver_options.rho = 2;
    solver_options.sigma = 20;
    solver_options.k_max = 5000;
    solver_options.tol_p = 1e-7;
    solver_options.tol_d = 1e-7;
    solver_options.debug = true;
    
    % Parameters of the MPC formulation
    param.N = 10;
    param.w = 3*1.627*0.2;
    param.Q = blkdiag(15*eye(sys.p), 1*eye(sys.p));
    param.R = 0.1*eye(sys.m);
    param.Te = 10*param.N*param.Q;
    param.Th = param.Te;
    param.Se = param.R;
    param.Sh = 0.5*param.Se;
    
    % Optimal solution
    z_opt = [0.799999796294972;0.799999796294973;0.0389465656616011;0.0243980535014476;0.0389465656616010;0.167221419199546;0.0278930741901471;0.167221419199547;0.590884289279489;0.590884289279466;0.0811512334111853;0.0339468417337970;0.0811512334111851;0.249809400932770;0.0763768569031614;0.249809400932767;-0.364638607948685;-0.364638607948696;0.117787237936796;0.0580528262570660;0.117787237936794;0.113259596324250;0.168060002649530;0.113259596324245;-0.163794230578206;-0.163794230578221;0.130105037887550;0.100120779654616;0.130105037887547;0.0110943828776571;0.244637170514373;0.0110943828776510;0.309247039126921;0.309247039126910;0.132718277846113;0.151271458430021;0.132718277846109;0.0181253138134725;0.253840089517441;0.0181253138134667;0.600656724254849;0.600656724254844;0.144237848759220;0.196879236143137;0.144237848759216;0.0985912743611562;0.193090435547613;0.0985912743611516;0.573460133931556;0.573460133931560;0.171588407732169;0.226508980975169;0.171588407732164;0.173239219075780;0.102604974303114;0.173239219075781;0.301084753714618;0.301084753714612;0.206873519864162;0.239240673962006;0.206873519864157;0.175737527284923;0.0307793867335241;0.175737527284924;0.00369756671971782;0.00369756671969504;0.234329569971691;0.241681022460652;0.234329569971687;0.0953064592444379;0.000349743371903263;0.0953064592444423;0.0121264312855273;0.0121264312854932;0.242305186467455;0.242305186468073;0.242305186467451;4.46948639223161e-12;3.24018113581033e-12;4.46921083414241e-12;0.000324894511828989;-0.000164772796472833;0.000324894511829046;-0.00970232319492711;0.00460511786197254;-0.00970232319493123;0.00185860096612097;-0.000942604031734900;0.00185860096612176;0.00169602383028817;-0.000805002012675058;0.00169602383028852;0.484610372935355;0.484610372935337;0.0114852213758538;0.0114852213758603;-0.0352419181958594;-0.0352419181958748];
    
    % Construct solver use_soc = false
    solver_options.use_soc = false;
    spcies_gen_controller('sys', sys, 'param', param, 'solver_options', solver_options,...
    'platform', 'Matlab', 'type', 'HMPC', 'method', 'ADMM_split');
    
    % Solve using the sparse solver
    [~, ~, e_s, sol_s] = HMPC(status.x, status.xr, status.ur);
    
    % Solse using the non-sparse solver
    [~, ~, e_ns, sol_ns] = spcies_HMPC_ADMM_split_solver(status.x, status.xr, status.ur, 'sys', sys,...
                                                 'param', param, 'options', solver_options, 'genHist', 1);
                                             
    % Construct solver for use_soc = true
    solver_options.use_soc = true;
    spcies_gen_controller('sys', sys, 'param', param, 'solver_options', solver_options,...
    'platform', 'Matlab', 'type', 'HMPC', 'method', 'ADMM_split');

    % Solve using the sparse solver
    [~, ~, e_s_soc, sol_s_soc] = HMPC(status.x, status.xr, status.ur);
    
    % Solse using the non-sparse solver
    [~, ~, e_ns_soc, sol_ns_soc] = spcies_HMPC_ADMM_split_solver(status.x, status.xr, status.ur, 'sys', sys,...
                                                 'param', param, 'options', solver_options, 'genHist', 1);
    
      
    % Compare results
    gap.spcies = [norm(sol_s.z - sol_ns.sol.z, Inf);
                  norm(sol_s.s - sol_ns.sol.s, Inf);
                  norm(sol_s.lambda - sol_ns.sol.lambda, Inf);
                  norm(sol_s.mu - sol_ns.sol.mu, Inf);
                  norm(sol_s_soc.z - sol_ns_soc.sol.z, Inf);
                  norm(sol_s_soc.s - sol_ns_soc.sol.s, Inf);
                  norm(sol_s_soc.lambda - sol_ns_soc.sol.lambda, Inf);
                  norm(sol_s_soc.mu - sol_ns_soc.sol.mu, Inf)];
                         
    gap.opt = max([norm(sol_s.z - z_opt, Inf);
                   norm(sol_s_soc.z - z_opt, Inf);]);
    
     % Exit flags
    exit = [e_s, e_ns];
    
end

