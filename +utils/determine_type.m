%% determine_type - Function that automatically determines the type of the controller
%
% INPUTS:
%   - controller: A structure or object containing the information of the controller
%
% OUTPUTS:
%   - type: A string that determines the type of controller
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function type = determine_type(controller)

    detected = true; % Flag used to determine if the type of controller has been detected

    %% Determine type by seeing if the controller is an instance of one of the GepocToolbox classes
    if isa(controller, 'ssMPC')
        if isa(controller, 'TrackingMPC')
            type = 'MPCT'; % MPC for traking formulation
        else
            detected = false;
        end
    else
        detected = false;
    end
    
    %% Try to determine the type of controller if the type has not already been detected
    if ~detected
        % Test for MPC for tracking
        if isfield(controller.param, 'S')
            type = 'MPCT';
            detected = true;
        % Test for MPC with ellipsoidal terminal set
        elseif isfield(controller.param, 'c')
            type = 'ellipMPC';
            detected = true;
        end
    end
    
    %% Throw error if type has not been detected
    if ~detected
        error('Spcies:gen_controller:unrecognized_type', 'Type not recognized from the diven data');
    end

end
