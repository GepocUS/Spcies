%% Test for the MPCT formulation using EADMM

function [gap, exit] = test_MPCT_EADMM(sys, status)
    
    % Solver options
    solver_options.rho_base = 2;
    solver_options.rho_mult = 20;
    solver_options.k_max = 5000;
    solver_options.tol = 1e-7;
    solver_options.debug = true;
    
    % Parameters of the MPC formulation
    param.Q = blkdiag(15*eye(sys.p), 1*eye(sys.p));
    param.R = 0.1*eye(sys.m);
    param.T = 10*param.Q;
    param.S = param.R;
    param.N = 10;
    
    % Construct solver
    spcies_gen_controller('sys', sys, 'param', param, 'solver_options', solver_options,...
    'platform', 'Matlab', 'type', 'MPCT', 'method', 'EADMM');
    
    % Solve using the sparse solver
    [u_s, k_s, e_s, sol_s] = MPCT(status.x, status.xr, status.ur);
    
    % Solse using the non-sparse solver
    [u_ns, k_ns, e_ns, sol_ns] = spcies_MPCT_EADMM_solver(status.x, status.xr, status.ur, 'sys', sys,...
                                                 'param', param, 'options', solver_options, 'genHist', 1);
                                            
    % Compare solutions
    z_opt = zeros(length(sol_s.z1), 1);
    z_opt = [0.0200000000000000;0.0200000000000000;0.0200000000000000;0.0200000000000000;0.0200000000000000;0.0200000000000000;0.799999993102319;0.799999993102319;0.0389465697053870;0.0243980535400605;0.0389465697053870;0.167221458258950;0.0278930770031138;0.167221458258950;0.121011781250797;0.121011781250922;0.0718777671371167;0.0337002238227884;0.0718777671370676;0.158301203400015;0.0714839714881919;0.158301203400129;-0.260687655582512;-0.260687655582931;0.0934441759009577;0.0545933703671533;0.0934441759009157;0.0558760132113321;0.137635996073868;0.0558760132115594;-0.00254981933894682;-0.00254981933949838;0.0994528756645121;0.0871757235500106;0.0994528756644928;0.00559522301887228;0.181055674362824;0.00559522301909965;0.320913585701752;0.320913585701219;0.102950142529125;0.123515912227536;0.102950142529138;0.0313508921584571;0.173530385920799;0.0313508921586845;0.453839374558552;0.453839374558111;0.115191075046879;0.153681904433316;0.115191075046872;0.0914448400966421;0.123321737085917;0.0914448400968695;0.363252139593876;0.363252139593506;0.137422371980256;0.172122772787257;0.137422371980247;0.129127322572344;0.0621125291690987;0.129127322572572;0.149310099615544;0.149310099615159;0.161587477929828;0.179805069309396;0.161587477929811;0.109800665733701;0.0191441392684055;0.109800665733815;-0.00698718405291632;-0.00698718405329080;0.177204961603197;0.181584837037649;0.177204961603193;0.0444002373325247;0.00237405924895029;0.0444002373325247;0.135424577133934;0.135424577133468;0.181704498014360;0.181704498014367;0.181704498014348;0;-1.13686837721616e-13;0;0.363408996028736;0.363408996028701];
    
    gap.spcies = [norm(sol_s.z1 - sol_ns.sol.z1, Inf);
                  norm(sol_s.z2 - sol_ns.sol.z2, Inf);
                  norm(sol_s.z3 - sol_ns.sol.z3, Inf);
                  norm(sol_s.lambda - sol_ns.sol.lambda, Inf)];
                         
    gap.opt = norm(sol_s.z1 - z_opt, Inf);
    
    % Exit flags
    exit = [e_s, e_ns];
    
end
