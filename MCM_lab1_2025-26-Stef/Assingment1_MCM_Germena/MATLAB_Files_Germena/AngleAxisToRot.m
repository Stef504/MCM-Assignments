function R = AngleAxisToRot(h,theta)
% The fuction implement the Rodrigues Formula

% Input: 
% h is the axis of rotation
% theta is the angle of rotation (rad)
%convert from deg to rad when typing input
I= eye(3);

x=h(1);
y=h(2);
z=h(3);

%the skew symmetric matrix
matrix_A =[0 -z y; z 0 -x; -y x 0];
square_matrix_A= h.*transpose(h) - I;

%formula
rodrigues_formula = I + (sin(theta).*matrix_A) + ((1-cos(theta)).*square_matrix_A);


% Output:
% R rotation matrix
disp('Rotation Matrix=');
disp(rodrigues_formula);


end
