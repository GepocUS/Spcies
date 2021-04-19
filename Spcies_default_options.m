%% Spcies_default_options - Returns the default options of the Spcies toolbox
% 
% This function containts the default options of the Spcies toolbox.
% It returns them in a structure that can be used as the argument
% spcies_options of other functions of the toolbox.
% 
% The options of the toolbox are:
%   - target: String that determines the target embedded system.
%   - save_name: String that indicates the name with which files are saved.
%                If empty, then each controller will use its own
%                default naming convention.
%   - directory: String that indicates the directory where files are saved.
%   - override: Boolean that determines if files can be overwritten.  
%               If they cannot, then they are saved with a number appended
%               to the end of the file.
%   - force_vector_rho: Some solvers use a penalty parameter named rho.
%               This penalty is usually a scalar, but this boolean
%               option forces the use of an array if set to true.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function [def_opt, def_method, def_subclass] = Spcies_default_options()
    
    % Default options of the toolbox
    def_opt.method = ''; % Default method
    def_opt.subclass = ''; % Defult subclass
    def_opt.type = ''; % Default type
    def_opt.platform = 'Matlab'; % Default platform
    def_opt.save_name = ''; % Default value of the save_name argument
    def_opt.directory = '$SPCIES$'; % Default directory where to save files
    def_opt.override = true; % Determined if files are overwritten if one with the same name already exists
    def_opt.force_vector_rho = false; % If true, forces the penalty parameter rho to be defined as a vector
    def_opt.save = 1; % Determines if the file is saved
    
    % Default metods for each type
    def_method.laxMPC = 'ADMM';
    def_method.equMPC = 'ADMM';
    def_method.ellipMPC = 'ADMM';
    def_method.MPCT = 'EADMM';
    def_method.Other = '';
    
    % Default subclasses for each type
    def_subclass.laxMPC = '';
    def_subclass.equMPC = '';
    def_subclass.ellipMPC = '';
    def_subclass.MPCT = 'mf';
    def_subclass.Other = '';
        
end
