%Noah Ossanna 10/25/20
%Class to create robot instances and plot trajectories as a function of:
    %wheel velocity (angular), wheel radii, wheel orientation, and geometry

classdef basicRobot
    %basicRobot creates robots with unique parameters for path simulation
    properties
        robotID        %robot name
        phi            %robot wheel velocities (shared)
        alpha1         %angle wheelbase 1
        alpha2         %angle wheelbase 2
        beta1          %angle wheel 1
        beta2          %angle wheel 2
        r1             %wheel 1 radius
        r2             %wheel 2 radius
        b              %wheel base of robot
        theta0         %initial heading of the robot
    end
    methods
        function globalVeloc = getVelocity(obj)
            %getVelocity returns the global velocity of a robot based on
            %geometry and heading
            J1 = [sin(obj.alpha1 + obj.beta1) -cos(obj.alpha1 + obj.beta1) -obj.b/2*cos(obj.beta1) ; sin(obj.alpha2 + obj.beta2) -cos(obj.alpha2 + obj.beta2) -obj.b/2*cos(obj.beta2) ; cos(obj.alpha1 + obj.beta1) sin(obj.alpha1 + obj.beta1) obj.b/2*sin(obj.beta1)];
            J2 = eye(3).*[obj.r1; obj.r2 ; 0];
            Rz = [cos(obj.theta0) -sin(obj.theta0) 0; sin(obj.theta0) cos(obj.theta0) 0; 0 0 1];
            %invJ1 = inv(J1)
            %invRz = inv(Rz)
            globalVeloc = inv(Rz)*inv(J1)*obj.phi.*J2; % get xdot, ydot, omega
        end
        
        function globalPose = getPose(dt, xdot, ydot, omega)
            %getPose returns global position based on velocity and timestep
            globalPose = [xdot*dt ydot*dt omega*dt]; %get x, y, theta
        end
    end
end
            
            