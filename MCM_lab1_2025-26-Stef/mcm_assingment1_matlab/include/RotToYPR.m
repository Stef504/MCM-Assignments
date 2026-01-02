function [psi,theta,phi] = RotToYPR(R)
% Given a rotation matrix the function outputs the relative euler angles
% usign the convention YPR
% Check that R is a valid rotation matrix using IsRotationMatrix().


if IsRotationMatrix(R)
theta = atan2(-R(3,1), sqrt((R(1,1)^2) + (R(2,1)^2)));

if (abs(theta)- pi/2) < 1e-3
    psi=atan2(R(2,1),R(1,1));
    phi=atan2(R(3,2),R(3,3));

    disp('psi= '); 
    disp(rad2deg(psi));
    disp('theta= ')
    disp(rad2deg(theta));
     disp('phi= ');
     disp(rad2deg(phi));
   
else 
    error('Singularity');
    if theta==pi/2
    phi=0;
    psi=atan2(R(2,3),R(1,3));
    
    else if theta==-pi/2
    phi=0;
    psi=atan2(-R(1,3),-R(2,3));
    end
    end


end

else 
    disp('Not a rotation matrix');

end
end

