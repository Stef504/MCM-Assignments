function [h,theta] = RotToAngleAxis(R)
% Given a rotation matrix this function
% should output the equivalent angle-axis representation values,
% respectively 'theta' (angle), 'h' (axis)
% Check that R is a valid rotation matrix using IsRotationMatrix()

if IsRotationMatrix(R)

theta = acos((trace(R)-1)/2);
I= [1 0 0; 0 1 0; 0 0 1];

if theta == 0
    R=I;
    h=[1 0 0]';
    % disp('Theta = 0');
    % disp('Axial Vector=');
    % disp(h);
end

if theta == pi
    
    if R(1,1)>0 
        h_1= abs(2*R(1,1)^2-1);

        h_2= sign(h_1)*sign(R(1,2))*sqrt((R(2,2)+1)/2);
        h_3= sign(h_1)*sign(R(1,3))*sqrt((R(3,3)+1)/2);

        h=[h_1 h_2 h_3]';

    else if R(2,2) >0 
        h_2= abs(2*R(2,2)^2-1);

        h_1= sign(h_2)*sign(R(2,1))*sqrt((R(1,1)+1)/2);
        h_3= sign(h_2)*sign(R(2,3))*sqrt((R(3,3)+1)/2);

        h=[h_1 h_2 h_3]';
    else
        h_3= abs(2*R(3,3)^2-1);

        h_1= sign(h_3)*sign(R(3,1))*sqrt((R(1,1)+1)/2);
        h_2= sign(h_3)*sign(R(3,2))*sqrt((R(2,2)+1)/2);

        h=[h_1 h_2 h_3]';
    end
    
      

    % disp('Axial Vector=');
    % disp(h);
    % disp('Theta = pi');
  end
else  

    h=(vex((R-transpose(R))/2)/sin(theta));
    %disp(['Axial Vector=', num2str(h)]);
    %disp(['Theta = ', num2str(theta)]);

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