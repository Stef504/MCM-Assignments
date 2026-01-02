%frame tree

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

T_12=X(-90)*Z(-90)
T23=Y(90)
T34=Y(-90)
T45=Y(90)

%%
ident=[0 0 0 1];

transformation_01= [1 0 0 0;0 1 0 0;0 0 1 0.175;0 0 0  1];
disp('Transformation from base to frame 1');
disp(transformation_01);

pose_12=[0;0;0.38];
rotation_12=X(90)*Y(-180);
transformation_12= [rotation_12 pose_12];
transformation_12=[transformation_12; ident];
disp('Transformation from frame 1 to frame 2');
disp(transformation_12);

pose_23=[1.05;0;0];
transformation_23=[Y(90) pose_23 ];
transformation_23=[transformation_23; ident];
disp('Transformation from frame 2 to frame 3');
disp(transformation_23);

pose_34=[0;1.455;3.265];
transformation_34=[Y(90)*X(180) pose_34];
transformation_34=[transformation_34; ident];
disp('Transformation from frame 3 to frame 4');
disp(transformation_34);

pose_45=[0.35;0;0];
transformation_45= [Z(-90)*X(-90) pose_45];
transformation_45=[transformation_45; ident];
disp('Transformation from frame 4 to frame 5');
disp(transformation_45);

pose_56=[0;0;3.85];
transformation_56=[Z(-90)*Y(-90) pose_56];
transformation_56=[transformation_56; ident];
disp('Transformation from frame 5 to frame 6');
disp(transformation_56);

pose_67=[1.53;0;0];
transformation_67=[Y(90)*Z(90) pose_67];
transformation_67=[transformation_67; ident];
disp('Transformation from frame 6 to frame 6');
disp(eye(4));
disp('Transformation from frame 6 to frame 7');
disp(transformation_67);

disp('Transformation from frame 7 to frame 6');
rotation_67=Y(90)*Z(90);
transformation_76=[rotation_67' ((-rotation_67')*pose_67);ident];
disp(transformation_76);

total=transformation_01 * transformation_12 * transformation_23 * transformation_34 * transformation_45 * transformation_56 *transformation_67;

disp('Transformation matrix from base to end effector= ')
disp(total);