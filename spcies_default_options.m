%% spcies_default_options - Returns the default options of the Spcies toolbox
% 
% This function containts the default options of the Spcies toolbox.
% It returns them in a structure that can be used as the argument
% spcies_options of other functions of the toolbox.
% 
% The options of the toolbox are:
%   - type: String that determines the type of controller to be generated.
%   - platform: String that determines the target embedded system or platform.
%   - method: String that determines the optimization method used.
%   - subclass: String that determines the exact solver to be used.
%               (only used if various are available).
%   - save_name: String that indicates the name with which files are saved.
%                If empty, then each controller will use its own
%                default naming convention.
%   - directory: String that indicates the directory where files are saved.
%   - override: Boolean that determines if files can be overwritten.  
%               If they cannot, then they are saved with a number appended
%               to the end of the file.
%   - const_are_static: Boolean that determines is constant variables are
%                       declared as static (is the language allowes it)
%   - precision: String that determines the precision with which variables
%                are strored. Either 'float' or 'double'.
%   - force_vector_rho: Some solvers use a penalty parameter named rho.
%               This penalty is usually a scalar, but this boolean
%               option forces the use of an array if set to true.
%   - time: If set to true the solvers return timing information.
%
% OUTPUTS:
%   - def_opt: Structure containing the default options of the toolbox.
%   - def_metod: Structure containing the default method of eath type.
%   - def_subclass: Structure containing the default subclass of each type.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function [def_opt, def_method, def_subclass] = spcies_default_options()
    
    % Default options of the toolbox
    def_opt.method = ''; % Default method
    def_opt.subclass = ''; % Defult subclass
    def_opt.type = ''; % Default type
    def_opt.platform = 'Matlab'; % Default platform
    def_opt.save_name = ''; % Default value of the save_name argument
    def_opt.directory = '$SPCIES$'; % Default directory where to save files
    def_opt.override = true; % Determines if files are overwritten if one with the same name already exists
    def_opt.const_are_static = true; % Determines if constant variables are defined as static
    def_opt.precision = 'double'; % Determines the precision of real numbers ('float' or 'double')
    def_opt.save = 1; % Determines if the file is saved (currently unused)
    def_opt.time = 0; % Determines if timing results are measured and returned by the solvers
    
    % Default metods for each type of controller
    def_method.laxMPC = 'ADMM';
    def_method.equMPC = 'ADMM';
    def_method.ellipMPC = 'ADMM';
    def_method.MPCT = 'EADMM';
    def_method.HMPC = 'ADMM';
    def_method.Other = '';
    
    % Default subclasses for each type of controller
    def_subclass.laxMPC.ADMM = '';
    def_subclass.laxMPC.FISTA = '';
    def_subclass.equMPC.ADMM = '';
    def_subclass.equMPC.FISTA = '';
    def_subclass.ellipMPC.ADMM = '';
    def_subclass.MPCT.EADMM = '';
    def_subclass.MPCT.ADMM = 'cs';
    def_subclass.HMPC.ADMM = '';
    def_subclass.HMPC.SADMM = '';
    def_subclass.Other = '';
     
end
