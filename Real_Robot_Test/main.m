%% Modelling and Control of Manipulator
clc; close all; clear;
addpath('include');

%% Simulation Parameters - DO NOT EDIT
ts = 0.005;
t_start = 0.0;
t_end = 300.0;
time = t_start:ts:t_end;
%%

% Execution flags
enable_real_robot = false;
enable_gripper = true;

%% Establish Robot Communication - DO NOT EDIT
rc = robotCommunication(enable_real_robot);
%%

%% Initial q - DO NOT EDIT
% Initial Joints configuration
q_init = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';
% Joint limits
qmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
qmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
%%

iTj_0 = BuildTree();

if enable_gripper
    % Fixed transform from <ee> to <tool>
    theta = -44.9949;% FIXED ANGLE BETWEEN EE AND TOOL 
    tool_length = 0.2124;% FIXED DISTANCE BETWEEN EE AND TOOL
    eRt = RPYToRot(0,0,deg2rad(theta));
    eOt = [0,0,tool_length]';
    % Tool Definition
    eTt = [eRt eOt; 0 0 0 1];
else
    eTt = eye(4);
end

jointType = [0 0 0 0 0 0 0];
gm = geometricModel(iTj_0,jointType,eTt);
gm.updateDirectGeometry(q_init);
km = kinematicModel(gm);

%Second goal Definition % TODO Change goal
bOg = [0.2280, 0.2200, 0.2238]';
bRg = RPYToRot(-pi/2,pi/2,0);
bTg = [bRg, bOg; 0, 0, 0, 1];

% Cartesian control
angular_gain=[0.1 0 0;0 0.1 0;0 0 0.1];
linear_gain=[0.1 0 0;0 0.1 0;0 0 0.1];

cc = cartesianControl(gm,angular_gain,linear_gain);

%Preallocate TODO verify they are useful
x_dot = zeros(6,1);
error_linear = zeros(3,1);
error_angular = zeros(3,1); 

q = q_init;
for t = time
    %% Read true q from robot if real robot is used- DO NOT EDIT
    is_first_iteration = (t == 0);
    q = rc.get_q(q, is_first_iteration);
    %%

    % Updating transformation matrices for the new configuration 
    gm.updateDirectGeometry(qf);
    b_T_t=gm.getToolTransformWrtBase();
    b_T_e= gm.getTransformWrtBase(gm.jointNumber);

    % Get the cartesian error given an input goal frame
    %if no error this should be an identity, this is e_T_g
    T_error=(inv(b_T_t))*bTg;
    %disp(T_error);

    % Update the jacobian matrix of the given model
    km = kinematicModel(gm);
    km.updateJacobian;

    %% INVERSE KINEMATIC
    % Compute desired joint velocities
    %q_dot = ...
% Compute desired joint velocities 
    cc = cartesianControl(gm,k_a,k_l);
    x_dot= cc.getCartesianReference(bTg);

    J_b_ee= km.J;

    %adding the tool
    %where e is the last joint and t is the tool
    R_b_e = b_T_e(1:3,1:3);
    b_r_et= R_b_e*e_r_te';

    skew_matrix= skew(b_r_et);
    J_et = [eye(3,3) zeros(3,3); skew_matrix' eye(3,3) ];
    
    b_J_t =J_et*J_b_ee;
    %disp(b_J_t)

    %damped SVD of Jacobian 
    J_inverse= Pseudo_Inverse(b_J_t);
    q_dot = J_inverse*x_dot;

    % simulating the robot
    q = KinematicSimulation(q, q_dot, ts, qmin, qmax);
    
    %% Move the robot - DO NOT EDIT
    rc.send_qdot(q_dot, q, t, true);
    %%

    bTt = gm.getToolTransformWrtBase();
    error_linear = bTg(1:3,4)-bTt(1:3,4);
    error_angular = x_dot(1:3) / angular_gain;
    
    if(norm(x_dot(1:3)) < 0.01 && norm(x_dot(4:6)) < 0.001)
        ang = error_angular;
        lin = error_linear;
        disp('REACHED THE REQUESTED GOAL POSITION')
        q_dot=[0,0,0,0,0,0,0]';
        break
    else
        disp('LINEAR ERROR');
        disp(norm(error_linear));
        disp('ANGULAR ERROR');
        disp(norm(error_angular));
    end
    
    %% Timing - DO NOT EDIT
    SlowdownToRealtime(ts);
    %%
    
end
