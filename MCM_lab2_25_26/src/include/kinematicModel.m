%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end


        function bJi = getJacobianOfLinkWrtBase(self, i)
            %%% getJacobianOfJointWrtBase
            % This method computes the Jacobian matrix bJi of joint i wrt base.
            % Inputs:
            % i : joint indnex ;

            % The function returns:
            % bJi
            
            %TO DO
            T_n_base= self.gm.getTransformWrtBase(7);
            r_n_base= T_n_base(1:3,4);
            for k= 1:i

                if self.gm.jointType(k) == 0 
                    J_A = self.gm.iTj(1:3,3,k);
                    
                    %linear of revolute
                    bTk = self.gm.getTransformWrtBase(k);
                    r_n_k = bTk(1:3,4);
                    axis_rotation = self.gm.iTj(1:3,3,k);
                    J_L= cross(axis_rotation, (r_n_base - r_n_k));
                end

                if self.gm.jointType(k) == 1
                    J_A=[0 0 0];
                    J_L= self.gm.iTj(1:3,3,k);
                end

                bJi(1:3,k,:)= J_A(:,:);
                bJi(4:6,k,:)= J_L(:,:);
            end
            

            
        end

        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix

        function S = skew(a)
            % input: skew matrix S_a (3x3)
            % output: the original a vector (3x1)
            x=a(1);
            y=a(2);
            z=a(3);
            
            S=[0 -z y;z 0 -x; -y x 0];
         end
            % TO DO
            self.J=zeros(6,self.gm.jointNumber)

            %task dependant
            T7ee=[1 0 0 0;
                  0 1 0 0;
                  0 0 1 0.060;
                  0 0 0 1];

            r_en=[0 0 0.060];
            skew_matrix= skew(r_en);
            
            J_en = [eye(3,3) zeros(3,3); skew_matrix' eye(3,3) ];
            bJi= self.getJacobianOfLinkWrtBase(self.gm.jointNumber); %has a return value
            
            self.J= J_en * bJi ; 
            
        end
    end
end

