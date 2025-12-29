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

%where e is the last joint and t is the tool frame
eTt = [eRt e_r_te'; ident];

%% Initialize Geometric Model (GM) and Kinematic Model (KM)
q = [0, 0, 0, 0, 0, 0, 0];
% Initialize geometric model with q0
gm = geometricModel(iTj_0,jointType,eTt);

% Update direct geoemtry given q0
gm.updateDirectGeometry(q);

% Initialize the kinematic model given the goemetric model
km = kinematicModel(gm);

bTt = gm.getToolTransformWrtBase();


disp("nTt");
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
b_T_t= gm.getToolTransformWrtBase();
b_T_e= gm.getTransformWrtBase(gm.jointNumber);
disp("Transformation matrix from base to tool:");
disp(b_T_t);
%disp(b_T_e);

%gain
k_a=[0.8 0 0;0 0.8 0;0 0 0.8];
k_l=[0.8 0 0;0 0.8 0;0 0 0.8];

% Cartesian control initialization
cc = cartesianControl(gm,k_a,k_l,eTt);
velocity=cc.getCartesianReference(bTg);

disp("Desired Velocity of tool wrt base:");
disp(velocity);

% control proportional gain 
km.updateJacobian;
J_b_e= km.J;

%adding the tool
R_b_e = b_T_e(1:3,1:3);
b_r_et= R_b_e*e_r_te';

skew_matrix= skew(b_r_et);
J_et = [eye(3,3) zeros(3,3); skew_matrix' eye(3,3) ];

b_J_t =J_et*J_b_e;
disp("Q2.3:")
disp("Jcobian of the base to the tool:")
disp(b_J_t);

%SVD of Jacobian
[U,S,V]= svd(b_J_t);

S_inv= zeros(size(S'));
sing_vals= diag(S);
threshold= 0.001;

for k= 1:length(sing_vals)
    sigma= sing_vals(k);

    if (sigma> threshold)
        S_inv(k,k)= 1/sigma;
    end

    if (sigma < threshold || sigma == threshold)
        S_inv(k,k)=0;
    end

end

J_inverse= V*S_inv*U';


%Actual Velocity of tool
q_velocity= J_inverse*velocity;
disp("Actually velocity of tool:");
disp(q_velocity);


%% Initialize control loop 
qf = [pi/2, -pi/4, 0, -pi/4, 0, 0.15, pi/4]';
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
    %% INVERSE KINEMATICS
    % Compute desired joint velocities 
    cc = cartesianControl(gm,k_a,k_l,eTt);
    x_dot= cc.getCartesianReference(bTg);
    % disp("x_dot");
    disp(x_dot);
    J_b_ee= km.J;

    %adding the tool
    %where e is the last joint and t is the tool
    R_b_e = b_T_e(1:3,1:3);
    b_r_et= R_b_e*e_r_te';

    skew_matrix= skew(b_r_et);
    J_et = [eye(3,3) zeros(3,3); skew_matrix' eye(3,3) ];
    
    b_J_t =J_et*J_b_ee;
    %disp(b_J_t)

    %SVD of Jacobian
    [U,S,V]= svd(b_J_t);
    
    S_inv= zeros(size(S'));
    sing_vals= diag(S);
    sigma_min=min(sing_vals);
    threshold= 0.01;
    lamda=0.01;
    
    if sigma_min >= threshold
        lamda_sq=0;
    end

    if sigma_min <= threshold
        lamda_sq= (1-(sigma_min/threshold)^2)*lamda;
    end

    for k= 1:length(sing_vals)
        sigma = sing_vals(k);
        S_inv(k,k)=sigma/(sigma^2+lamda_sq);
           
    end
    
    J_inverse= V*S_inv*U';
    q_dot = J_inverse*x_dot;
    %disp(J_inverse);
    
    % disp("q_dot:")
    %disp(q_dot);
    % simulating the robot

    q = KinematicSimulation(qf,q_dot,dt,qmin,qmax);
    %disp(q);
    
    qf=q;

    for k= 1: gm.jointNumber
        bTi(:,:,k)=gm.getTransformWrtBase(k);
    end
    bTi(:,:, gm.jointNumber + 1) = b_T_t;
    
    pm.plotIter(gm, km, i, q_dot);

    if(norm(x_dot(1:3)) < 0.01 && norm(x_dot(4:6)) < 0.01)
        disp('Reached Requested Pose')
        break
    end

end

pm.plotFinalConfig(gm);

%%
% Q2.5

velocity_ee= J_b_ee * q_dot;
velocty_t = b_J_t* q_dot;

disp("Final joint velocities:");
disp(q_dot);
disp("Velocity of end effector, projected in base frame:");
disp(velocity_ee);

disp("Velocity of the tool, projected in the base frame:");
disp(velocty_t);

