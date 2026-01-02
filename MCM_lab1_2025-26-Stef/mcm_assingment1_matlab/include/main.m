addpath('include');

% TO DO: Test assignment 1 MCM 2024-2025
% 1.1 Angle-axis to rot
disp('Q1.2');
AngleAxisToRot([1;0;0],deg2rad(90));

disp('Q1.3');
AngleAxisToRot([0;0;1],pi/3);

disp('Q1.4');
RotVectorToRot([-pi/3 -pi/6 pi/3]);

% 1.2 Rot to angle-axis
disp('Q2.2');
RotToAngleAxis([1 0 0;0 0 -1;0 1 0]);

disp('Q2.3');
RotToAngleAxis([0.5 -sqrt(3)/2 0;sqrt(3)/2 0.5 0;0 0 1]);

disp('Q2.4');
RotToAngleAxis([1 0 0;0 1 0;0 0 1]);

disp('Q2.5');
RotToAngleAxis([-1 0 0;0 -1 0; 0 0 1]);

disp('Q2.6');
RotToAngleAxis([-1 0 0;0 1 0;0 0 1]);

% 1.3 Euler to rot
disp('Q3.2');
YPRToRot(0,0,pi/2);

disp('Q3.3');
YPRToRot(deg2rad(60),0,0);

disp('Q3.4');
YPRToRot(pi/3,pi/2,pi/4);

disp('Q3.5');
YPRToRot(0,pi/2,-pi/12);

% 1.4 Rot to Euler
disp('Q4.2');
RotToYPR([1 0 0;0 0 -1;0 1 0]);

disp('Q4.3');
RotToYPR([1/2 -sqrt(3)/2 0;sqrt(3)/2 1/2 0;0 0 1]);

disp('Q4.4');
RotToYPR([0 -sqrt(2)/2 sqrt(2)/2;0.5 (sqrt(2)*sqrt(3))/4 (sqrt(2)*sqrt(3))/4; -sqrt(3)/2 sqrt(2)/4 sqrt(2)/4]);

% 1.5 Rot to angle-axis with eigenvectors
disp('Q5.1');
RotToAngleAxis([1 0 0;0 0 -1;0 1 0]);
Q5([1 0 0;0 0 -1;0 1 0]);

disp('Q5.2');
RotToAngleAxis(1/9*[4 -4 -7;8 1 4;-1 -8 4]);
Q5(1/9*[4 -4 -7;8 1 4;-1 -8 4]);

%Frame Tree
Frame_tree;