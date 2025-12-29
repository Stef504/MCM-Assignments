function a = vex(S_a)
% input: skew matrix S_a (3x3)
% output: the original a vector (3x1)
y=S_a(1,3);
z=S_a(2,1);
x=S_a(3,2);

a=[x y z];
end