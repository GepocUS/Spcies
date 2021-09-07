%% Test for the MPCT formulation using ADMM (extended state space)

function [gap, exit] = test_MPCT_ADMM(sys, status)
    
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
    'platform', 'Matlab', 'type', 'MPCT', 'method', 'ADMM');
    
    % Solve using the sparse solver
    [u_s, k_s, e_s, sol_s] = MPCT(status.x, status.xr, status.ur);
    
    % Solse using the non-sparse solver
    [u_ns, k_ns, e_ns, sol_ns] = spcies_MPCT_ADMM_cs_solver(status.x, status.xr, status.ur, 'sys', sys,...
                                                 'param', param, 'options', solver_options, 'genHist', 1);
                                            
    % Compare solutions
    z_opt = [0.0200000000000000;0.0200000000000000;0.0200000000000000;0.0200000000000000;0.0200000000000000;0.0200000000000000;0.181704497766961;0.181704497766939;0.181704497766954;-1.13686837721616e-13;0;-1.13686837721616e-13;0.799999968544582;0.799999968544582;0.363408995533953;0.363408995533887;0.0389465692207114;0.0243980535271706;0.0389465692207114;0.167221453476259;0.0278930767474321;0.167221453476259;0.181704497766961;0.181704497766939;0.181704497766954;-1.13686837721616e-13;0;-1.13686837721616e-13;0.121011810627501;0.121011810627372;0.363408995533953;0.363408995533887;0.0718777663373522;0.0337002236545256;0.0718777663373522;0.158301205063822;0.0714839701473693;0.158301205063935;0.181704497766961;0.181704497766939;0.181704497766954;-1.13686837721616e-13;0;-1.13686837721616e-13;-0.260687653930779;-0.260687653931245;0.363408995533953;0.363408995533887;0.0934441755102000;0.0545933698660637;0.0934441755101905;0.0558760155601021;0.137635994286029;0.0558760155603295;0.181704497766961;0.181704497766939;0.181704497766954;-1.13686837721616e-13;0;-1.13686837721616e-13;-0.00254982740535548;-0.00254982740591792;0.363408995533953;0.363408995533887;0.0994528755785116;0.0871757227477954;0.0994528755785280;0.00559522365654175;0.181055673301671;0.00559522365688281;0.181704497766961;0.181704497766939;0.181704497766954;-1.13686837721616e-13;0;-1.13686837721616e-13;0.320913579192175;0.320913579191715;0.363408995533953;0.363408995533887;0.102950142412019;0.123515911338335;0.102950142412025;0.0313508912073530;0.173530386127027;0.0313508912075804;0.181704497766961;0.181704497766939;0.181704497766954;-1.13686837721616e-13;0;-1.13686837721616e-13;0.453839369459857;0.453839369459449;0.363408995533953;0.363408995533887;0.115191074619423;0.153681903690214;0.115191074619422;0.0914448379953683;0.123321738218124;0.0914448379958230;0.181704497766961;0.181704497766939;0.181704497766954;-1.13686837721616e-13;0;-1.13686837721616e-13;0.363252144759414;0.363252144759263;0.363408995533953;0.363408995533887;0.137422371253226;0.172122772288953;0.137422371253249;0.129127321734472;0.0621125303379131;0.129127321734813;0.181704497766961;0.181704497766939;0.181704497766954;-1.13686837721616e-13;0;-1.13686837721616e-13;0.149310106335564;0.149310106335444;0.363408995533953;0.363408995533887;0.161587477212773;0.179805068992782;0.161587477212810;0.109800666682190;0.0191441398707184;0.109800666682418;0.181704497766961;0.181704497766939;0.181704497766954;-1.13686837721616e-13;0;-1.13686837721616e-13;-0.00698718138196452;-0.00698718138245413;0.363408995533953;0.363408995533887;0.177204961168828;0.181584836785242;0.177204961168879;0.0444002391772074;0.00237405934774415;0.0444002391773211;0.181704497766961;0.181704497766939;0.181704497766954;-1.13686837721616e-13;0;-1.13686837721616e-13;0.135424567166759;0.135424567165551;0.363408995533953;0.363408995533887];
    
    gap.spcies = [norm(sol_s.z - sol_ns.sol.z, Inf);
                  norm(sol_s.v - sol_ns.sol.v, Inf);
                  norm(sol_s.lambda - sol_ns.sol.lambda, Inf)];
                         
    gap.opt = norm(sol_s.z - z_opt, Inf);                   
    
     % Exit flags
    exit = [e_s, e_ns];
    
end
