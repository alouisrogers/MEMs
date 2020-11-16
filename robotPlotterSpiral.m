clc; clear;
%Noah Ossanna 10/25/20
%Sub-script using robotPlotter architecture to plot 2D spirals
%Goal: cover all 2D space with outward spiral trajectories
    %Uses nested for loop to iterate through increasing wheel speeds

%Outcome: failed: only way to change the diameter of the trajectories is to alter wheel diameters
%Only negligable outward progression of traj with unrealistic speed 
%params currently set to show an unstable solution 

%define time of simulation
dt = 0.1;
t = 0:dt:100;

vi= 0.1;
vf = 10;   %NOT REALISTIC

dv = (vf - vi)/(length(t)-1);
vel = vi:dv:vf;

%define properties of first robot class (CCW = + convention)
    % robotID = robot name
    % phi1    = left wheel velocity
    % phi2    = right wheel velocity
    % alpha1  = left wheel angle with respect to the positive horizontal plane
    % alpha2  = right wheel angle with respect to the positive horizontal plane
    % beta1   = left wheel angle with respect to the wheelface normal vector
    % beta2   = right wheel angle with respect to the wheelface normal vector
    % r1      = left wheel radius
    % r2      = right wheel radius
    % b       = wheelbase length
    % theta0  = initial heading
    
                  %(robotID, phi1, phi2, alpha1, alpha2, beta1, beta2, r1, r2, b, theta0)
robot1 = basicRobot('robot1', vel(1), vel(1), pi/2, -pi/2, 0, pi, 0.1, 0.2, 0.05, pi/2);  
    
%Pre-allocate pose storage vectors for plotting
poseX = zeros(1,length(t));
poseY = zeros(1,length(t));
poseTheta = zeros(1,length(t));
poseTheta(1) = robot1.theta0;

%Begin time and velocity stepping
for i = 2:length(t)+1      
        
    robot1 = basicRobot('robot1', vel(i-1), vel(i-1), pi/2, -pi/2, 0, pi, 0.1, 0.2, 0.05, poseTheta(i-1));   

    %access velocity from robot class static methods
    globalVelocity = robot1.getVelocity(robot1);
    xdot = globalVelocity(1);
    ydot = globalVelocity(2);
    omega = globalVelocity(3);

    %access pose from robot class static methods
    poseChange = robot1.getPose(dt, xdot, ydot, omega);     %returns dx, dy, dtheta (change in each var given over the given step)
    poseX(i) = poseX(i-1) + poseChange(1);
    poseY(i) = poseY(i-1) + poseChange(2);
    poseThetaLocal = poseTheta(i-1) + poseChange(3);        %accumulate robot heading = previous angle plus current change in angle (same with above positions)

    poseTheta(i) = wrapTo2Pi(poseThetaLocal);               %store and export incremental robot heading

    robot1.theta0 = poseThetaLocal;                         %update robot heading based on most recent accumulation
end

%Calc Trajectory Diameter
trajDia = abs(max(poseX) - min(poseX));

%Get robot heading in degrees
poseTheta = wrapTo360(rad2deg(poseTheta));

%Plotting
figure(1111)
%title('Robot Position Over ', num2str(t(end)),' Seconds')
plot(t,poseY(1:end-1), 'r.')
hold on
plot(t,poseX(1:end-1), 'b.')
xlabel('time (s)')
ylabel('X and Y Position (m)')
title('Robot Position vs. Time Over 20 Seconds')
legend('t vs. y', 't vs. x')

figure(1112)
plot(poseX,poseY, 'b.')
title('X vs. Y Position')
legend(sprintf('Trajectory Diameter = %0.3f m',trajDia),'Location','best')
xlabel('X Position (m)')
ylabel('Y Position (m)')

% figure(1113)
% plot(t,poseTheta(1:end-1))
% title('Theta vs. Time')
% xlabel('Time (s)')
% ylabel('Mapped Heading (0-360 Degrees)')
% 
% figure(1114)
% plot(t,poseX(1:end-1))


