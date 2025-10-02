%% Test for the HMPC formulation using ADMM with the (z_hat, s_hat) = (z, s) splitting

function [gap, exit] = test_HMPC_ADMM_s(sys, status)
    
    % Solver options
    solver_options.rho = 2;
    solver_options.sigma = 20;
    solver_options.k_max = 5000;
    solver_options.tol_p = 1e-7;
    solver_options.tol_d = 1e-7;
    solver_options.debug = true;
    solver_options.sparse = true;
    
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
    z_opt = [0.799999998591329;0.799999998591329;0.0389465698140178;0.0243980535433597;0.0389465698140201;0.167221459328144;0.0278930770602070;0.167221459328143;0.582514850939310;0.582514850939329;0.0809860647404112;0.0339424505397498;0.0809860647404112;0.248179477230433;0.0762897197072570;0.248179477230440;-0.364005483011976;-0.364005483011975;0.117329605451217;0.0579905729946198;0.117329605451218;0.112000196934985;0.167505431775567;0.112000196934989;-0.166239305568833;-0.166239305568858;0.129385691039406;0.0998783123184663;0.129385691039408;0.00975961058112611;0.243367625032559;0.00975961058111972;0.305841721158637;0.305841721158662;0.131715186787747;0.150699631914623;0.131715186787747;0.0166385459467073;0.251828185149335;0.0166385459467083;0.600070079274639;0.600070079274598;0.142984556783909;0.195844593215654;0.142984556783909;0.0975784690394498;0.190531163940160;0.0975784690394414;0.574596298171685;0.574596298171711;0.170211503074848;0.224946773551085;0.170211503074848;0.172997012511079;0.0999970024114949;0.172997012511080;0.299978793931548;0.299978793931549;0.205468058246307;0.237211156039507;0.205468058246307;0.175666586226905;0.0288320331060385;0.175666586226906;-0.000692780951117285;-0.000692780951109453;0.232850073233862;0.239377392975579;0.232850073233861;0.0946285522526808;-0.000390331216338919;0.0946285522526840;0.00869720570111586;0.00869720570110384;0.240206717177514;0.240206717177141;0.240206717177514;-8.20517974564524e-19;2.23371258238097e-17;2.23371258238097e-17;-0.000878852841546889;0.000445717099959532;-0.000878852841547110;0.0111293855921368;-0.00528245983749755;0.0111293855921369;-0.00213197256842042;0.00108124658093168;-0.00213197256842049;-0.00458781332408732;0.00217756312077097;-0.00458781332408783;0.480413434355034;0.480413434355034;-0.00439188207258474;-0.00439188207258392;0.0450886042978604;0.0450886042978611];

    % Construct solver use_soc = false
    solver_options.use_soc = false;
    spcies_gen_controller('sys', sys, 'param', param, 'options', solver_options,...
    'platform', 'Matlab', 'formulation', 'HMPC', 'method', 'ADMM', 'submethod', 'split');
    
    % Solve using the sparse solver
    [~, ~, e_s, sol_s] = HMPC(status.x, status.xr, status.ur);
    
    % Solse using the non-sparse solver
    [~, ~, e_ns, sol_ns] = spcies_HMPC_ADMM_split_solver(status.x, status.xr, status.ur, 'sys', sys,...
                                                 'param', param, 'options', solver_options, 'genHist', 1);
                                             
    % Construct solver for use_soc = true
    solver_options.use_soc = true;
    spcies_gen_controller('sys', sys, 'param', param, 'options', solver_options,...
    'platform', 'Matlab', 'formulation', 'HMPC', 'method', 'ADMM', 'submethod', 'split');

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

