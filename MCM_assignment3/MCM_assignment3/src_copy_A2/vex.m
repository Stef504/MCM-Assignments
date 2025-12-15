function S = vex(a)
            % input: skew matrix S_a (3x3)
            % output: the original a vector (3x1)
            x=a(3,2);
            y=a(1,3);
            z=a(2,1);
            
            S=[x,y,z];
end