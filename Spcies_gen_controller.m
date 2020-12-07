%% Spcies_gen_controller - Generation of embedded MPC controllers
% 
% This function is the main Spcies function. It generates the controller designated by
% input 'type' for the target embedded system designated by input 'target'.
% This function tipically saves files to the current working directory.
% 
% INPUTS:
%   - type: type of controller generated. See documentation for a list and description of the controllers available.
%   - target: target embedded system that the controller is generated for. See documentation for a list of supported ones.
%   - sys: model of the system.
%   - param: structure containing  parameters of the controller.
%            Each controller will require different parameters. See documentation for which ones.
%   - options: structure containing options of the solver.
%              Each controller/solver will have different ones. See documantation for which ones.
%   - save_name: string that determines the name of any files saved to the current directory.
% 
% OUTPUTS:
%   - str: Structure containing a variety of information
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

% Author: Pablo Krupa (pkrupa@us.es)
% 
% Changelog: 
%   v0.1 (2020/09/03): Initial commit version
%   v0.2 (2020/09/17): Added documentation
%   v0.3 (2020/10/16): Added RMPC_ADMM
%

function str = Spcies_gen_controller(type, target, sys, param, options, save_name)

if strcmp(type, 'MPCT')
    str = MPCT.Spcies_gen_MPCT_EADMM(target, sys, param, options, save_name);
elseif strcmp(type, 'ellipMPC')
    str = ellipMPC.Spcies_gen_ellipMPC_ADMM(target, sys, param, options, save_name);
else
    error('Type not recognized or supported');
end

end
