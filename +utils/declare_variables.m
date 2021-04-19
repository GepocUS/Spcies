

function text = declare_variables(data, options)

    if strcmp(options.platform, 'C')
        text = C_code.declareVariables(data);
    elseif strcmp(options.platform, 'Arduino')
        text = Arduino.declareVariables(data);
    elseif strcmp(options.platform, 'Unity')
        text = Unity.declareVariables(data);
    end
    
end
