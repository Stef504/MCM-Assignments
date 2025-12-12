function S = skew(a)
            % input: skew matrix S_a (3x3)
            % output: the original a vector (3x1)
            x=a(1);
            y=a(2);
            z=a(3);
            
            S=[0 -z y;z 0 -x; -y x 0];
    end