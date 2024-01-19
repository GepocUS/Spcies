%% determine_formulation - Function that automatically determines the MPC formulation
%
% INPUTS:
%   - controller: A structure or object containing the information of the controller
%
% OUTPUTS:
%   - formulation: A string that determines the formulation of controller
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function formulation = determine_formulation(controller)

    detected = true; % Flag used to determine if the MPC formulation has been detected

    %% Determine formulation by seeing if the controller is an instance of one of the GepocToolbox classes
    if isa(controller, 'ssMPC')
        if isa(controller, 'TrackingMPC')
            formulation = 'MPCT'; % MPC for traking formulation
        elseif isa(controller, 'LaxMPC')
            formulation = 'laxMPC';
        else
            detected = false;
        end
    else
        detected = false;
    end
    
    %% Try to determine the MPC formulaiton if it has not already been detected
    if ~detected
        % Test for MPC for tracking
        if isfield(controller.param, 'S')
            formulation = 'MPCT';
            detected = true;
        % Test for MPC with ellipsoidal terminal set
        elseif isfield(controller.param, 'c')
            formulation = 'ellipMPC';
            detected = true;
        elseif isfield(controller.param, 'P')
            formulation = 'laxMPC';
            detected = true;
        end
    end
    
    %% Throw error if formulation has not been detected
    if ~detected
        error('Spcies:gen_controller:unrecognized_formulation', 'MPC formulation not recognized from the given data. Please specify MPC formulation.');
    end

end

