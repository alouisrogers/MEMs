clc; clear;
%Noah Ossanna 10/25/20
%Script to plot robot trajectories for different geometries using:
%basicRobot Class

%define time of simulation
dt = 0.1;
t = 0:dt:15;

%define properties of first robot class
              %(robotID, phi1, phi2, alpha1, alpha2, beta1, beta2, r1, r2, b, theta0)
robot1 = basicRobot('robot1', 0.7, -0.7, pi/2, -pi/2, 0, pi, 0.1, 0.3, 0.5, pi/2);


poseX = zeros(1,length(t));
poseY = zeros(1,length(t));
poseTheta = zeros(1,length(t));
poseTheta(1) = robot1.theta0;

for i = 2:length(t)+1
    globalVelocity = robot1.getVelocity(robot1);
    xdot = globalVelocity(1);
    ydot = globalVelocity(2);
    omega = globalVelocity(3);
    
    poseChange = robot1.getPose(dt, xdot, ydot, omega);     %returns dx, dy, dtheta (change in each var given over the given step)
    poseX(i) = poseX(i-1) + poseChange(1);
    poseY(i) = poseX(i-1) + poseChange(2);
    poseThetaLocal = poseTheta(i-1) + poseChange(3);
    
    poseTheta(i) = wrapTo2Pi(poseThetaLocal);
    
    robot1.theta0 = poseThetaLocal; %update robot heading based on most recent timestep calc
    
end

poseTheta = wrapTo360(rad2deg(poseTheta));
figure(1111)
%title('Robot Position Over ', num2str(t(end)),' Seconds')
title('Robot Position vs. Time')
plot(t,poseY(1:end-1), 'r.')
hold on
plot(t,poseX(1:end-1), 'm.')
xlabel('time (s)')
ylabel('X and Y Position (m)')
legend('t vs. x', 't vs. y')

figure(1112)
plot(poseX,poseY, 'b.')
title('X Versus Y Position')
xlabel('X Position (m)')
ylabel('Y Position (m)')

figure(1113)
plot(t,poseTheta(1:end-1))
title('Theta Versus Time')
xlabel('time (s)')
ylabel('Mapped Heading (0-360 Degrees)')




