function [isRotationMatrix] = IsRotationMatrix(R)
% The function checks that the input R is a valid rotation matrix, that is 
% a valid element of SO(3).
% Return true if R is a valid rotation matrix, false otherwise. In the
% latter case, print a warning pointing out the failed check.
I= eye(3);

first_check=transpose(R)*R;
tol=1e-3;


if (max(abs(first_check-I),[],'all')<tol) && (abs(det(R)-1)<tol)
    isRotationMatrix=true;
else
    isRotationMatrix=false;
   

end
end