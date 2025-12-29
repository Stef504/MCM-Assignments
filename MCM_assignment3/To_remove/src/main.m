%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear;
addpath('include'); % put relevant functions inside the /include folder 

%% Compute the geometric model for the given manipulator
iTj_0 = BuildTree();

disp('iTj_0')
disp(iTj_0);
jointType = [0 0 0 0 0 1 0]; % specify two possible link type: Rotational, Prismatic.
q = [pi/2, -pi/4, 0, -pi/4, 0, 0.15, pi/4]';


%% Define the tool frame rigidly attached to the end-effector
% Tool frame definition
ident=[0 0 0 1];

eRt = YPRToRot(pi/10,0,pi/6);
e_r_te = [0.3,0.1,0];
eTt = [eRt e_r_te'; ident];

%% Initialize Geometric Model (GM) and Kinematic Model (KM)
q = [0, 0, 0, 0, 0, 0, 0];
% Initialize geometric model with q0
gm = geometricModel(iTj_0,jointType,eTt);

% Update direct geoemtry given q0
gm.updateDirectGeometry(q);

% Initialize the kinematic model given the goemetric model
km = kinematicModel(gm);

bTt = gm.getToolTransformWrtBase(eTt);

disp("eTt");
disp(eTt);
disp('bTt q = 0');
disp(bTt);

%% Define the goal frame and initialize cartesian control

q = [pi/2, -pi/4, 0, -pi/4, 0, 0.15, pi/4]';

% Update direct geoemtry given q
gm.updateDirectGeometry(q);

% Initialize the kinematic model given the goemetric model
km = kinematicModel(gm);

% Goal definition 
bOg = [0.2; -0.7; 0.3];
theta = pi/2;
bRg = YPRToRot(0,theta,0);
bTg = [bRg bOg;0 0 0 1]; 
disp('bTg')
disp(bTg)

%Q2.1
b_T_t= gm.getToolTransformWrtBase(eTt);

disp("Transformation matrix from base to tool:");
disp(b_T_t);

b_r_t= b_T_t(1:3,4);
b_r_g= bTg(1:3,4);

%error:

%linear error:
e_r_g= b_r_g-b_r_t;

%rotation error:
b_R_t= b_T_t(1:3,1:3);
b_R_g= bTg(1:3,1:3);

e_R_g= b_R_t'*b_R_g;
angular= vex(e_R_g);

%angular and linear velocoties of the tool wrt base
k_a=[0.8 0 0;0 0.8 0;0 0 0.8];
k_l=[0.8 0 0;0 0.8 0;0 0 0.8];

b_v_t= [k_a zeros(3) ;zeros(3) k_l]*[e_r_g;angular'];
disp("Velocity of tool wrt base:");
disp(b_v_t);

% control proportional gain 
J=km.updateJacobian;

skew_matrix= skew(eTt);
J_en = [eye(3,3) zeros(3,3); skew_matrix' eye(3,3) ];

b_J_t =J * J_en;


% Cartesian control initialization
%cc = cartesianControl(....);

%% Initialize control loop 

% Simulation variables
samples = 100;
t_start = 0.0;
t_end = 10.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

% preallocation variables
bTi = zeros(4, 4, gm.jointNumber);
bri = zeros(3, gm.jointNumber+1);

% joints upper and lower bounds 
qmin = -3.14 * ones(7,1);
qmin(6) = 0;
qmax = +3.14 * ones(7,1);
qmax(6) = 1;

show_simulation = true;
pm = plotManipulators(show_simulation);
pm.initMotionPlot(t, bTg(1:3,4));

%%%%%%% Kinematic Simulation %%%%%%%
for i = t
    % Updating transformation matrices for the new configuration 

    % Get the cartesian error given an input goal frame

    % Update the jacobian matrix of the given model

    %% INVERSE KINEMATICS
    % Compute desired joint velocities 
    q_dot = ...

    % simulating the robot
    q = KinematicSimulation(....);
    
    pm.plotIter(gm, km, i, q_dot);

    if(norm(x_dot(1:3)) < 0.01 && norm(x_dot(4:6)) < 0.01)
        disp('Reached Requested Pose')
        break
    end

end

pm.plotFinalConfig(gm);