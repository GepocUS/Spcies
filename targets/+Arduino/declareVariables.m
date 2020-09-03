%% Function that declares the variables for Arduino

function s = declareVariables(vars)
import Arduino.decVar;
%% Preliminaries
s = ['']; % String array containing the text for variable declaration
numVars = size(vars, 1);

for i = 1:numVars
    s = [s decVar(vars(i,:))];
end

end
