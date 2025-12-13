function [iTj_0] = BuildTree_test()

function rotation_z=Z(psi)

psi=deg2rad(psi);
rotation_z=[cos(psi) -sin(psi) 0;
            sin(psi) cos(psi) 0;
            0 0 1];
end

function rotation_y=Y(theta)

theta=deg2rad(theta);
rotation_y=[cos(theta) 0 sin(theta);
            0           1           0;
            -sin(theta) 0 cos(theta)];
end

function rotation_x=X(phi)

phi=deg2rad(phi);
rotation_x=[1      0       0;
            0 cos(phi) -sin(phi);
            0 sin(phi) cos(phi)];
end

ident=[0 0 0 1];

R_01=eye(3);
iTj_0(:, :, 1)=[R_01 [0 0 0.105]'; ident];

R_12=X(-90)*Z(-90);
iTj_0(:, :, 2)=[R_12 [0 0 0.110]'; ident];

R_23=Y(90);
iTj_0(:, :, 3)=[R_23 [0.100 0 0]'; ident];

R_34=Y(-90);
iTj_0(:, :, 4)=[R_34 [0 0 0.325]'; ident];

R_45=Y(90);
iTj_0(:, :, 5)=[R_45 [0.095 0 0]'; ident];

iTj_0(:, :, 6)=[eye(3) [0 0 0.095]'; ident];

iTj_0(:, :, 7)=[eye(3) [0 0 0.355]'; ident];

end


% function [iTj_0] = BuildTree()
%     % This function should build the tree of frames for the chosen manipulator.
%     % Inputs: 'None'
%     % Outputs: The tree of frames.
% 
%     % iTj_0 corresponds to the trasformation from the frame <i> to <i'> which
%     % for q = 0 is equal to the trasformation from <i> to <i+1>
%     % (see notes)
% 
%     % iTj_0 is a 3-dimensional matlab matrix, suitable for defining tree of
%     % frames. iTj_0 should represent the transformation matrix between the i-th and j-th
%     % frames. iTj_0(row,col,joint_idx)
% 
%     i = [1 0 0]';
%     j = [0 1 0]';
%     k = [0 0 1]';
% 
%     iTj_0(:, :, 1) = tFactory([i j k], k.*0.105);   % <1>
% 
%     iTj_0(:, :, 2) = tFactory([i -k j], k.*0.110);  % <2>
% 
%     iTj_0(:, :, 3) = tFactory([i k -j], -j.*0.100); % <3>
% 
%     iTj_0(:, :, 4) = tFactory([i -k j], k.*0.325);  % <4>
% 
%     iTj_0(:, :, 5) = tFactory([i k -j], -j.*0.095); % <5>
% 
%     iTj_0(:, :, 6) = tFactory([i j k], k.*0.095);   % <6>
% 
%     iTj_0(:, :, 7) = tFactory([i j k], k.*0.355);   % <7> = <EE>
% 
% end





function [iTj_0] = BuildTree_stef()
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

