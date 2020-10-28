%Noah Ossanna 10/25/20
%Class to create robot instances and plot trajectories as a function of:
    %wheel velocity (angular), wheel radii, wheel orientation, and geometry

classdef basicRobot
    %basicRobot creates robots with unique parameters for path simulation
    properties
        robotID        %robot name
        phi1           
        phi2
        alpha1         %angle wheelbase 1
        alpha2         %angle wheelbase 2
        beta1          %angle wheel 1
        beta2          %angle wheel 2
        r1             %wheel 1 radius
        r2             %wheel 2 radius
        b              %wheel base of robot
        theta0         %initial heading of the robot
    end
    methods (Static)
       function obj = basicRobot(robotID, phi1, phi2, alpha1, alpha2, beta1, beta2, r1, r2, b, theta0)
            %constructor function: define properties of created objects
            obj.robotID = robotID;
            obj.phi1       = phi1;           %robot wheel velocities (shared)
            obj.phi2       = phi2;           %robot wheel velocities (shared)
            obj.alpha1    = alpha1;       %angle wheelbase 1
            obj.alpha2    = alpha2;        %angle wheelbase 2
            obj.beta1     = beta1;          %angle wheel 1
            obj.beta2     = beta2;           %angle wheel 2
            obj.r1        = r1;        %wheel 1 radius
            obj.r2        = r2;         %wheel 2 radius
            obj.b         = b;          %wheel base of robot
            obj.theta0    = theta0;        %initial heading of the robot
      end
        
        function globalVeloc = getVelocity(obj)
            %getVelocity returns the global velocity of a robot based on
            %geometry and heading
            ab1 = obj.alpha1 + obj.beta1;
            ab2 = obj.alpha2 + obj.beta2;
            b1 = obj.beta1;
            b2 = obj.beta2;
            J1 = [sin(ab1) -cos(ab1) -obj.b/2*cos(b1) ; sin(ab2) -cos(ab2) -obj.b/2*cos(b2) ; cos(ab1) sin(ab1) obj.b/2*sin(b1)];
            J2 = eye(3).*[obj.r1; obj.r2 ; 0]*[obj.phi1; obj.phi2; 0];
            Rz = [cos(obj.theta0) -sin(obj.theta0) 0; sin(obj.theta0) cos(obj.theta0) 0; 0 0 1];
            %invJ1 = inv(J1)
            %invRz = inv(Rz)
            globalVeloc = inv(Rz)*inv(J1)*J2 % get xdot, ydot, omega (this is defined gross but lazy
        end
        
        function globalPose = getPose(dt, xdot, ydot, omega)
            %getPose returns global position based on velocity and timestep
            globalPose = [xdot*dt ydot*dt omega*dt]; %get x, y, theta
        end
    end
end
            
            