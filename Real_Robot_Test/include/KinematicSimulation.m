function [q_output]= KinematicSimulation(q,q_dot,ts,q_min,q_max)
%% Kinematic Simulation function
%
% Inputs
% - q current robot configuration
% - q_dot joints velocity
% - ts sample time
% - q_min lower joints bound
% - q_max upper joints bound
%
% Outputs
% - q new joint configuration
q_output = q+ (q_dot*ts);
% disp("q_out");
% disp(q_output);

%compares values in vectors and set it to the bounds
q_output=min(q_output,q_max);
q_output=max(q_output,q_min);
    

end