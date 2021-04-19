

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
