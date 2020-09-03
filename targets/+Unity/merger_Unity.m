

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
