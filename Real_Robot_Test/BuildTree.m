function [iTj] = BuildTree()
% Builds the frame tree for Franka Panda (links 1–7 only)
% Based directly on URDF joint origins (zero joint angles)

%% Joint 1: link0 -> link1
iTj(1,1,1) = 1; iTj(1,2,1) = 0; iTj(1,3,1) = 0; iTj(1,4,1) = 0;
iTj(2,1,1) = 0; iTj(2,2,1) = 1; iTj(2,3,1) = 0; iTj(2,4,1) = 0;
iTj(3,1,1) = 0; iTj(3,2,1) = 0; iTj(3,3,1) = 1; iTj(3,4,1) = 0.333;
iTj(4,1,1) = 0; iTj(4,2,1) = 0; iTj(4,3,1) = 0; iTj(4,4,1) = 1;

%% Joint 2: link1 -> link2  (Rx(-pi/2))
iTj(1,1,2) = 1;  iTj(1,2,2) = 0;  iTj(1,3,2) = 0;  iTj(1,4,2) = 0;
iTj(2,1,2) = 0;  iTj(2,2,2) = 0;  iTj(2,3,2) = 1;  iTj(2,4,2) = 0;
iTj(3,1,2) = 0;  iTj(3,2,2) = -1; iTj(3,3,2) = 0;  iTj(3,4,2) = 0;
iTj(4,1,2) = 0;  iTj(4,2,2) = 0;  iTj(4,3,2) = 0;  iTj(4,4,2) = 1;

%% Joint 3: link2 -> link3  (Rx(pi/2), y = -0.316)
iTj(1,1,3) = 1;  iTj(1,2,3) = 0;  iTj(1,3,3) = 0;  iTj(1,4,3) = 0;
iTj(2,1,3) = 0;  iTj(2,2,3) = 0;  iTj(2,3,3) = -1; iTj(2,4,3) = -0.316;
iTj(3,1,3) = 0;  iTj(3,2,3) = 1;  iTj(3,3,3) = 0;  iTj(3,4,3) = 0;
iTj(4,1,3) = 0;  iTj(4,2,3) = 0;  iTj(4,3,3) = 0;  iTj(4,4,3) = 1;

%% Joint 4: link3 -> link4  (Rx(pi/2), x = 0.0825)
iTj(1,1,4) = 1;  iTj(1,2,4) = 0;  iTj(1,3,4) = 0;  iTj(1,4,4) = 0.0825;
iTj(2,1,4) = 0;  iTj(2,2,4) = 0;  iTj(2,3,4) = -1; iTj(2,4,4) = 0;
iTj(3,1,4) = 0;  iTj(3,2,4) = 1;  iTj(3,3,4) = 0;  iTj(3,4,4) = 0;
iTj(4,1,4) = 0;  iTj(4,2,4) = 0;  iTj(4,3,4) = 0;  iTj(4,4,4) = 1;

%% Joint 5: link4 -> link5  (Rx(-pi/2), x = -0.0825, y = 0.384)
iTj(1,1,5) = 1;  iTj(1,2,5) = 0;  iTj(1,3,5) = 0;  iTj(1,4,5) = -0.0825;
iTj(2,1,5) = 0;  iTj(2,2,5) = 0;  iTj(2,3,5) = 1;  iTj(2,4,5) = 0.384;
iTj(3,1,5) = 0;  iTj(3,2,5) = -1; iTj(3,3,5) = 0;  iTj(3,4,5) = 0;
iTj(4,1,5) = 0;  iTj(4,2,5) = 0;  iTj(4,3,5) = 0;  iTj(4,4,5) = 1;

%% Joint 6: link5 -> link6  (Rx(pi/2))
iTj(1,1,6) = 1;  iTj(1,2,6) = 0;  iTj(1,3,6) = 0;  iTj(1,4,6) = 0;
iTj(2,1,6) = 0;  iTj(2,2,6) = 0;  iTj(2,3,6) = -1; iTj(2,4,6) = 0;
iTj(3,1,6) = 0;  iTj(3,2,6) = 1;  iTj(3,3,6) = 0;  iTj(3,4,6) = 0;
iTj(4,1,6) = 0;  iTj(4,2,6) = 0;  iTj(4,3,6) = 0;  iTj(4,4,6) = 1;

%% Joint 7: link6 -> link7  (Rx(pi/2), x = 0.088)
iTj(1,1,7) = 1;  iTj(1,2,7) = 0;  iTj(1,3,7) = 0;  iTj(1,4,7) = 0.088;
iTj(2,1,7) = 0;  iTj(2,2,7) = 0;  iTj(2,3,7) = -1; iTj(2,4,7) = 0;
iTj(3,1,7) = 0;  iTj(3,2,7) = 1;  iTj(3,3,7) = 0;  iTj(3,4,7) = 0.107;
iTj(4,1,7) = 0;  iTj(4,2,7) = 0;  iTj(4,3,7) = 0;  iTj(4,4,7) = 1;

end
