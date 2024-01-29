%% get_generic_mex_exec
%
% Returns the string required by Spcies_constructor for
% compiling the mex file of a controller.
% It includes the flags that must be filled in by the
% Spcies_constructor.construct method.
%
% INPUTS:
%   - options: Instance of Spcies_problem.options
% 
% OUTPUTS:
%   - exec_me: String used by Spcies_constructor
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
%

function exec_me = get_generic_mex_exec(options)

    % String for main call to the mex command
    mex_cmd = 'mex -silent';
    
    % Add the flags to the mex command
    mex_flags = '-DCONF_MATLAB'; % Add the flag that tells the source code that this is for Matlab
    
    % Add optimization flags
    if(~ispc)
        mex_flags = sprintf('%s %s', mex_flags, '-lm COPTIMFLAGS="-O3"');
    end
    
    % Generate string to call mex
    exec_me = sprintf('%s -I$INSERT_PATH$ -I$C_CODE_PATH$ $INSERT_PATH$$INSERT_NAME$.c $C_CODE_PATH$$C_CODE_NAME$.c -outdir $INSERT_PATH$ %s', mex_cmd, mex_flags);
    
end

