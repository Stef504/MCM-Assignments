%% Kinematic Model Class - GRAAL Lab
classdef cartesianControl < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        k_a
        k_l
        eTt
    end

    methods
        % Constructor to initialize the geomModel property
        function self = cartesianControl(gm,angular_gain,linear_gain,eTt)
            if nargin > 2
                self.gm = gm;
                self.k_a = angular_gain;
                self.k_l = linear_gain;
                self.eTt= eTt;
            else
                error('Not enough input arguments (cartesianControl)')
            end
        end
        function [x_dot]=getCartesianReference(self,bTg)
            %% getCartesianReference function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control
            
            b_T_t= self.gm.getToolTransformWrtBase();
            b_T_e= self.gm.getTransformWrtBase(self.gm.jointNumber);

            b_r_t= b_T_t(1:3,4);
            b_r_g= bTg(1:3,4);

            %linear error:
            %in base frame
            t_r_g= b_r_g-b_r_t;
            
            %rotation error:
            b_R_t= b_T_t(1:3,1:3);
            b_R_g= bTg(1:3,1:3);

            %in tool frame
            t_R_g= b_R_t'*b_R_g;
            
            [h,theta]=RotToAngleAxis(t_R_g);
            
            %our correction factor
            p= theta*h;
            % disp("from rotToAngleAxis:")
            % disp(p);
            
                      
            %angular and linear velocoties of the tool wrt base
            
            %b_angular_t_ee= self.k_a *p';
            b_angular_t_ee= p';
            % disp("angular from rotToangleAxis")
            % disp(b_angular_t);
            b_linear_t= self.k_l*t_r_g;
            %b_linear_t=t_r_g;
            
            %convert angualar velocity to base frame
            b_angular_t= b_R_t*b_angular_t_ee;         
            
            %desired velocity of tool
            x_dot=[b_angular_t;b_linear_t];

            
        end
    end
end

