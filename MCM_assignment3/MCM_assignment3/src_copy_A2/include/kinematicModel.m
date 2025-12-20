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
            bJi=zeros(6,self.gm.jointNumber);
            
            T_n_base= self.gm.getTransformWrtBase(i);
            r_n_base= T_n_base(1:3,4);

            for k= 1:i
                T_base=self.gm.getTransformWrtBase(k);

                if self.gm.jointType(k) == 0 
                    J_A = T_base(1:3,3);
                    
                    %linear of revolute
                
                    r_n_k = T_base(1:3,4);
                    axis_rotation = T_base(1:3,3);
                    J_L= cross(axis_rotation, (r_n_base - r_n_k));
                end

                if self.gm.jointType(k) == 1
                    J_A=[0 0 0];
                    J_L= T_base(1:3,3);
                end

                bJi(1:3,k,:)= J_A(:,:);
                bJi(4:6,k,:)= J_L(:,:);
            end
            

            
        end

        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix

         % TO DO
            self.J=zeros(6,self.gm.jointNumber);
                     
            bJi= self.getJacobianOfLinkWrtBase(self.gm.jointNumber); %has a return value

            self.J= bJi ; 
      
            
        end
    end
end

