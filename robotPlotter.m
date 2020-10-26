%Noah Ossanna 10/25/20
%Script to plot robot trajectories for different geometries using:
%basicRobot Class

%define time of simulation
dt = 0.1;
t = 0:dt:5;

%define properties of first robot class
robot1 = basicRobot;
%robot1.ID = 'robot1';
robot1.phi       = 4;           %robot wheel velocities (shared)
robot1.alpha1    = -pi/2;       %angle wheelbase 1
robot1.alpha2    = pi/2;        %angle wheelbase 2
robot1.beta1     = pi;          %angle wheel 1
robot1.beta2     = 0;           %angle wheel 2
robot1.r1        = 0.25;        %wheel 1 radius
robot1.r2        = 0.1;         %wheel 2 radius
robot1.b         =0.4;          %wheel base of robot
robot1.theta0    = pi/2;        %initial heading of the robot

xPose = [];
yPose = [];
thetaPose = [];

for i = 1:length(t)
    globalVelocity = robot1.getVelocity(robot1);
    xdot = globalVelocity(1);
    ydot = globalVelocity(2);
    omega = globalVelocity(3);
    
    %globalPose = robot1.getPose(dt, xdot, ydot, omega);
    xPose(i) = xdot*dt; %globalPose(1);
    yPose(i) = ydot*dt;%globalPose(2);
    thetaPose(i) = omega*dt;%globalPose(3);
    
    robot1.theta0 = thetaPose(i); %update robot heading based on most recent timestep calc
end

figure(1111)
plot(xPose,yPose, 'r.')
%title('Robot Position Over ', num2str(t(end)),' Seconds')
xlabel('X Position (m)')
ylabel('Y Position (m)')



