function [h,theta] =EigenToRot(R)

if IsRotationMatrix(R)
lamba=1;
B=lamba*eye(3,3)-R;

A=rref(B); %row echoloen form
h_sol=null(A,'rational');%solving for h values


if isempty(h_sol) || size(h_sol, 2) > 1
        disp('Singularity: Rotation angle is 0, axis is arbitary.');
        h = [1;0;0];
        theta = 0;
    else
        h=h_sol/norm(h_sol); %normailze vector unit vector needed for axial vector
        
        %RotToAngleAxis(R.*h);
        theta = acos((trace(R)-1)/2);
        
        disp('Theta is=');
        disp(theta);
        disp('Axial vector is an eigen vector, which is=');
        disp(h);
end
else
    disp('Not a rotation matrix');
end
end

function a = vex(S_a)
% input: skew matrix S_a (3x3)
% output: the original a vector (3x1)
y=S_a(1,3);
z=S_a(2,1);
x=S_a(3,2);

a=[x y z];
end

%end