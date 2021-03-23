%% gen_mex - Generates the MEX file from the given .c file
%
% INPUTS:
%   - c_file: String containing the name of the C file (without the extension)
%   - directory: String containing the working directory
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function gen_mex(c_file, directory)

    % String for main call to the mex command
    mex_cmd = 'mex -silent';
    
    % Add the flags to the mex command
    mex_flags = '-DCONF_MATLAB'; % Add the flag that tells the source code that this is for Matlab
    
    % Add optimization flags
    if(~ispc)
        mex_flags = sprintf('%s %s', mex_flags, '-lm COPTIMFLAGS="-O3"');
    end
    
    % Generate string to call mex
    cmd = sprintf('%s %s%s.c %s%s_C.c %s', mex_cmd, directory, c_file, directory, c_file, mex_flags);
    
    % Call the mex command
    eval(cmd);
    
end

