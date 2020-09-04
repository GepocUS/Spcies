%% merger_Unity - Merges the text for variables and the text for the code
% Adds the headers where necessary
%
% INPUTS:
%   - var_text: String containing the text for the variable declaration
%   - code_text: String containing the text of the code
%   - name: Name of the FBD in the Unity environment
%
% OUTPUTS:
%   - text: Text of the merged inputs strings
%

% Function belonging to the Spcies toolbox
% Author: Pablo Krupa, pkrupa@us.es, (c) 2020
%

% Devlog and notes:
%   - 20/09/04: Cleaned code and added documentation

function text = merger_Unity(var_text, code_text, name)

    % Header
    text = [''];
    text = [text '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n<FBExchangeFile>\n'];
    text = [text '<fileHeader company="Schneider Automation" product="Unity Pro XL V5.0 - 100504A" '];
    text = [text 'dateTime="date_and_time#2016-5-13-11:5:3" content="Function Block source file" DTDVersion="41"></fileHeader>\n'];
    text = [text '<contentHeader name="Project" version="0.0.17"></contentHeader>\n'];
    text = [text '<FBSource nameOfFBType="' name '" version="0.01">\n'];

    % Variables
    text = [text var_text];

    % Code
    text = [text '<FBProgram name="' name '_ST">\n<STSource>'];
    text = [text code_text];
    text = [text '</STSource>\n</FBProgram>\n</FBSource>\n</FBExchangeFile>\n'];
    
end

