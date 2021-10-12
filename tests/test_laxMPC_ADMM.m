%% Test for the laxMPC formulation using ADMM

function [gap, exit] = test_laxMPC_ADMM(sys, status)

    % Solver options
    solver_options.rho = 15;
    solver_options.k_max = 5000;
    solver_options.tol = 1e-7;
    solver_options.debug = true;
    
    % Parameters of the MPC formulation
    param.Q = blkdiag(15*eye(sys.p), 1*eye(sys.p));
    param.R = 0.1*eye(sys.m);
    [~, param.T] = dlqr(sys.A, sys.B, param.Q, param.R);
    param.T = diag(sum(param.T, 2));
    param.N = 10;
    
%     sys.LBx = kron(ones(1, param.N+1), sys.LBx);
%     sys.LBu = kron(ones(1, param.N+1), sys.LBu);
%     sys.UBx = kron(ones(1, param.N+1), sys.UBx);
%     sys.UBu = kron(ones(1, param.N+1), sys.UBu);
    
    % Construct solver
    spcies_gen_controller('sys', sys, 'param', param, 'solver_options', solver_options,...
    'platform', 'Matlab', 'type', 'laxMPC', 'method', 'ADMM');

    % Solve using the sparse solver
    [u_s, k_s, e_s, sol_s] = laxMPC(status.x, status.xr, status.ur);
    
    % Solse using the non-sparse solver
    [u_ns, k_ns, e_ns, sol_ns] = spcies_laxMPC_ADMM_solver(status.x, status.xr, status.ur, 'sys', sys,...
                                                 'param', param, 'options', solver_options, 'genHist', 1);
                                            
    % Compare solutions
    z_opt = [0.799999998898375;0.799999998898375;0.0389465698197782;0.0243980535431019;0.0389465698197782;0.167221459387633;0.0278930770634815;0.167221459387633;0.669949837264263;0.669949837264415;0.0827116954234906;0.0339883420816092;0.0827116954235005;0.265207545088970;0.0772002022902143;0.265207545089197;-0.240914727794479;-0.240914727794505;0.124670526897251;0.0587090766316964;0.124670526897270;0.150417502961432;0.174650012341431;0.150417502961545;-0.0600965690063428;-0.0600965690066245;0.145777664398236;0.103423839478784;0.145777664398218;0.0608295099757470;0.266167451622323;0.0608295099758607;0.338958107896186;0.338958107895706;0.157625406422991;0.161086580547147;0.157625406423003;0.0599349571234598;0.298164623026423;0.0599349571236871;0.544664829980786;0.544664829980502;0.174734765063995;0.217811011393060;0.174734765064011;0.112677290068177;0.258450688047446;0.112677290068405;0.481003008446973;0.481003008446651;0.201602695879163;0.261375687715573;0.201602695879200;0.155328122135188;0.172723292889827;0.155328122135302;0.259946613749497;0.259946613749023;0.231771940701393;0.286407451645552;0.231771940701360;0.144002371553484;0.0789844167899219;0.144002371553597;0.0696017597279808;0.0696017597273373;0.254346529192001;0.294216982424153;0.254346529192026;0.0792422039161238;0.00308287069060498;0.0792422039162375;0.109884742343462;0.109884742342662;0.263448050683504;0.289247615641941;0.263448050683496;0.0102189716612884;-0.0489957628619777;0.0102189716614021];
    
    gap.spcies = [norm(sol_s.z - sol_ns.sol.z, Inf);
                  norm(sol_s.v - sol_ns.sol.v, Inf);
                  norm(sol_s.lambda - sol_ns.sol.lambda, Inf)];
                         
    gap.opt = norm(sol_s.z - z_opt, Inf);                   
    
     % Exit flags
    exit = [e_s, e_ns];
    
end
