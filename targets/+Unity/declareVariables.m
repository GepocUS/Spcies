%% Function that declares the variables for Unity Pro

function s = declareVariables(vars)
import Unity.decVar;
%% Preliminaries
s = ['']; % String array containing the text for variable declaration
numVars = size(vars, 1);

%% Declare input variables
s = [s sprintf('<inputParameters>\n')];

position = 1;
for i = 1:numVars
    if strcmp(vars{i, 5}, 'input')
        s = [s decVar(vars(i,:), position)];
        position = position + 1;
    end
end

s = [s sprintf('</inputParameters>\n')];

%% Output variables
s = [s '<outputParameters>\n'];

position = 1;
for i = 1:numVars
    if strcmp(vars{i, 5}, 'output')
        s = [s decVar(vars(i,:), position)];
        position = position + 1;
    end
end

s = [s '</outputParameters>\n'];

%% Public variables
s = [s '<publicLocalVariables>\n'];

for i = 1:numVars
    if strcmp(vars{i, 5}, 'public') 
        s = [s decVar(vars(i,:))];
    end
end

s = [s '</publicLocalVariables>\n'];

%% Private variables
s = [s '<privateLocalVariables>\n'];

for i = 1:numVars
    if strcmp(vars{i, 5}, 'private') 
        s = [s decVar(vars(i,:))];
    end
end

s = [s '</privateLocalVariables>\n'];

end