function [iTj_0] = BuildTree()
% This function should build the tree of frames for the chosen manipulator.
% Inputs: 'None'
% Outputs: The tree of frames.

% iTj_0 corresponds to the trasformation from the frame <i> to <i'> which
% for q = 0 is equal to the trasformation from <i> to <i+1>
% (see notes)

% iTj_0 is a 3-dimensional matlab matrix, suitable for defining tree of
% frames. iTj_0 should represent the transformation matrix between the i-th and j-th
% frames. iTj_0(row,col,joint_idx)

%% 0T1
iTj_0(1,1,1) = 1; iTj_0(1,2,1) = 0; iTj_0(1,3,1) = 0; iTj_0(1,4,1) = 0;
iTj_0(2,1,1) = 0; iTj_0(2,2,1) = 1; iTj_0(2,3,1) = 0; iTj_0(2,4,1) = 0;
iTj_0(3,1,1) = 0; iTj_0(3,2,1) = 0; iTj_0(3,3,1) = 1; iTj_0(3,4,1) = 0.105;
iTj_0(4,1,1) = 0; iTj_0(4,2,1) = 0; iTj_0(4,3,1) = 0; iTj_0(4,4,1) = 1;

T01=iTj_0(:,:,1)
%% 1T2
iTj_0(1,1,2) = 0; iTj_0(1,2,2) = 1;  iTj_0(1,3,2) = 0; iTj_0(1,4,2) = 0;
iTj_0(2,1,2) = 0; iTj_0(2,2,2) = 0;  iTj_0(2,3,2) = 1; iTj_0(2,4,2) = 0;
iTj_0(3,1,2) = 1; iTj_0(3,2,2) = 0;  iTj_0(3,3,2) = 0; iTj_0(3,4,2) = 0.110;
iTj_0(4,1,2) = 0; iTj_0(4,2,2) = 0;  iTj_0(4,3,2) = 0; iTj_0(4,4,2) = 1;

T12=iTj_0(:,:,2)
%% 2T3
iTj_0(1,1,3) = 0; iTj_0(1,2,3) = 0; iTj_0(1,3,3) = 1;  iTj_0(1,4,3) = 0.1;
iTj_0(2,1,3) = 0; iTj_0(2,2,3) = 1; iTj_0(2,3,3) = 0;  iTj_0(2,4,3) = 0;
iTj_0(3,1,3) = -1; iTj_0(3,2,3) = 0; iTj_0(3,3,3) = 0;  iTj_0(3,4,3) = 0;
iTj_0(4,1,3) = 0; iTj_0(4,2,3) = 0; iTj_0(4,3,3) = 0;  iTj_0(4,4,3) = 1;

T23=iTj_0(:,:,3)

%% 3T4
iTj_0(1,1,4) = 0; iTj_0(1,2,4) = 0;  iTj_0(1,3,4) = -1; iTj_0(1,4,4) = 0;
iTj_0(2,1,4) = 0; iTj_0(2,2,4) = 1;  iTj_0(2,3,4) = 0; iTj_0(2,4,4) = 0;
iTj_0(3,1,4) = 1; iTj_0(3,2,4) = 0;  iTj_0(3,3,4) = 0; iTj_0(3,4,4) = 0.325;
iTj_0(4,1,4) = 0; iTj_0(4,2,4) = 0;  iTj_0(4,3,4) = 0; iTj_0(4,4,4) = 1;

T34=iTj_0(:,:,4)
%% 4T5
iTj_0(1,1,5) = 0; iTj_0(1,2,5) = 0; iTj_0(1,3,5) = 1;  iTj_0(1,4,5) = 0.095;
iTj_0(2,1,5) = 0; iTj_0(2,2,5) = 1; iTj_0(2,3,5) = 0;  iTj_0(2,4,5) = 0;
iTj_0(3,1,5) = -1; iTj_0(3,2,5) = 0; iTj_0(3,3,5) = 0;  iTj_0(3,4,5) = 0;
iTj_0(4,1,5) = 0; iTj_0(4,2,5) = 0; iTj_0(4,3,5) = 0;  iTj_0(4,4,5) = 1;

T45=iTj_0(:,:,5)
%% 5T6
iTj_0(1,1,6) = 0; iTj_0(1,2,6) = 0;  iTj_0(1,3,6) = 1; iTj_0(1,4,6) = 0;
iTj_0(2,1,6) = 0; iTj_0(2,2,6) = 1;  iTj_0(2,3,6) = 0; iTj_0(2,4,6) = 0;
iTj_0(3,1,6) = -1; iTj_0(3,2,6) = 0;  iTj_0(3,3,6) = 0; iTj_0(3,4,6) = 0.095;
iTj_0(4,1,6) = 0; iTj_0(4,2,6) = 0;  iTj_0(4,3,6) = 0; iTj_0(4,4,6) = 1;

T56=iTj_0(:,:,6)
%% 6T7
iTj_0(1,1,7) = 0; iTj_0(1,2,7) = 0;  iTj_0(1,3,7) = 1; iTj_0(1,4,7) = 0;
iTj_0(2,1,7) = 0; iTj_0(2,2,7) = 1;  iTj_0(2,3,7) = 0; iTj_0(2,4,7) = 0;
iTj_0(3,1,7) = -1; iTj_0(3,2,7) = 0;  iTj_0(3,3,7) = 0; iTj_0(3,4,7) = 0.355;
iTj_0(4,1,7) = 0; iTj_0(4,2,7) = 0;  iTj_0(4,3,7) = 0; iTj_0(4,4,7) = 1;

T67=iTj_0(:,:,7)
end

