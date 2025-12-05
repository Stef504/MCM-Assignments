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
geometricModel = geometricModel(iTj_0,jointType);

%% Q1.3
 %task dependant
T7ee=[1 0 0 0;
      0 1 0 0;
      0 0 1 0.060;
      0 0 0 1];

Tb_n=geometricModel.getTransformWrtBase(7);
Tb_e=Tb_n*T7ee;
disp("Transformation matrix from the base to the EE:");
disp(Tb_e);

T2_6= geometricModel.getTransform6wrt2(6);
disp("Transformation of frame 6 wrt to frame 2");
disp(T2_6);
%Inverse
ident=[0 0 0 1];
Rotation_matrix= T2_6(1:3, 1:3);
Pose_26= T2_6(1:3,4);

Transformation_62=[Rotation_matrix' ((-Rotation_matrix')*Pose_26); ident];

disp("Transformation of frame 2 wrt to frame 6:");
disp(Transformation_62);

%% Q1.4 Simulation
% Given the following configurations compute the Direct Geometry for the manipulator

% Compute iTj : transformation between the base of the joint <i>
% and its end-effector taking into account the actual rotation/traslation of the joint
qi = [pi/4, -pi/4, 0, -pi/4, 0, 0.15, pi/4];
geometricModel.updateDirectGeometry(qi)
disp('iTj')
disp(geometricModel.iTj);

% Compute the transformation of the ee w.r.t. the robot base
bTe = geometricModel.getTransformWrtBase(length(jointType));  
disp('bTe')
disp(bTe)

% Show simulation ?
show_simulation = true;

% Set initial and final joint positions
qf = [5*pi/12, -pi/3, 0, -pi/4, 0, 0.18, pi/5];

%%%%%%%%%%%%% SIMULATION LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation variables
% simulation time definition 
samples = 100;
t_start = 0.0;
t_end = 10.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

pm = plotManipulators(show_simulation);
pm.initMotionPlot(t);

qSteps =[linspace(qi(1),qf(1),samples)', ...
    linspace(qi(2),qf(2),samples)', ...
    linspace(qi(3),qf(3),samples)', ...
    linspace(qi(4),qf(4),samples)', ...
    linspace(qi(5),qf(5),samples)', ...
    linspace(qi(6),qf(6),samples)', ...
    linspace(qi(7),qf(7),samples)'];

% LOOP 
for i = 1:samples

    brij= zeros(3,geometricModel.jointNumber);
    q = qSteps(i,1:geometricModel.jointNumber)';
    % Updating transformation matrices for the new configuration 
    geometricModel.updateDirectGeometry(q)
    % Get the transformation matrix from base to the tool
    bTe = geometricModel.getTransformWrtBase(length(jointType)); 

    %% ... Plot the motion of the robot 
    if (rem(i,0.1) == 0) % only every 0.1 sec
        for j=1:geometricModel.jointNumber
            bTi(:,:,j) = geometricModel.getTransformWrtBase(j); 
        end
        pm.plotIter(bTi)
    end

end

pm.plotFinalConfig(bTi)

disp(geometricModel.q -qf') %should be zero
%% Q1.5

qf = [5*pi/12, -pi/3, 0, -pi/4, 0, 0.18, pi/5];
geometricModel.updateDirectGeometry(qf);

km = kinematicModel(geometricModel);

J= km.getJacobianOfLinkWrtBase(6);
disp("Jacobian of Link 6 with respect to base");
disp(J);

%% Q1.6
qf = [5*pi/12, -pi/3, 0, -pi/4, 0, 0.18, pi/5];
geometricModel.updateDirectGeometry(qf)

km = kinematicModel(geometricModel);
km.updateJacobian;
disp("Jacobian of the end effector with respect to base");
disp(km.J);

%% Q1.7
qf = [0.7 -0.1 1 -1 0 0.03 1.3];
q_velocities = [0.9 0.1 -0.2 0.3 -0.8 0.5 0];

geometricModel.updateDirectGeometry(qf)

km = kinematicModel(geometricModel);
km.updateJacobian;

disp("Jacobian of the end effector with respect to base");
disp(km.J);

%this gives up to n 
angular_velocity_base = km.J(1:3,:) * q_velocities';
linear_velocity_base = km.J(4:6,:) * q_velocities';

%adding EE
angular_velocity_ee= angular_velocity_base; 
%because frame of EE does not rotate about <n>

%linear part of EE
function S = skew(a)
    % input: skew matrix S_a (3x3)
    % output: the original a vector (3x1)
    x=a(1);
    y=a(2);
    z=a(3);
    
    S=[0 -z y;z 0 -x; -y x 0];
end

r_en=[0 0 0.060];
skew_r_en= skew(r_en);
linear_velocity_ee = skew_r_en'*angular_velocity_base + linear_velocity_base;

%wrt to the ee frame
rotation_matrix_base_ee = geometricModel.iTj(1:3,1:3) ;
transpose_rotation_matrix = rotation_matrix_base_ee';

angular_ee_frame = transpose_rotation_matrix * angular_velocity_ee;

linear_ee_frame = transpose_rotation_matrix * linear_velocity_ee;

disp("Angular velocity of the end effector wrt to the base, projected in the EE frame");
disp(angular_ee_frame);

disp("Linear Velocity of the end effector wrt to the base, projected in the EE frame");
disp(linear_ee_frame);



