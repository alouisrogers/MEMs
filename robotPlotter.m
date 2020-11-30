clc; clear;
%Noah Ossanna 10/25/20
%Script to plot robot trajectories for different geometries using:
%basicRobot Class

%In progress:
%11/15/20: modify loop to reverse wheel velicities
    %test zig zag trajectory patterns for max robot coverage
    
    %Overall: test ways to cover 2D plane using only changes in wheel
    %directions/velocities with time
    %END result: failure, can only change trajectories by changing wheel
    %dia

%Implement trajectory mapping to intestinal profile in 3D (started
    %11/14/20)
%Implement inverse kinematics controller to navigate to a desired point


%define time of simulation
dt = 0.1;
t = 0:dt:20;

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
robot1 = basicRobot('robot1', 3, 3, pi/2, -pi/2, 0, pi, 0.005, 0.005, 0.01, pi/6);
%11/27/20: Current Params Set to

%Pre-allocate pose storage vectors for plotting
poseX = zeros(1,length(t));
poseY = zeros(1,length(t));
poseTheta = zeros(1,length(t));
poseTheta(1) = robot1.theta0;

%Begin time stepping
for i = 2:length(t)+1      %index of 2 makes sure robot pose vectors start at origin
    %access velocity from robot class static methods
    globalVelocity = robot1.getVelocity(robot1);
    xdot = globalVelocity(1);
    ydot = globalVelocity(2);
    omega = globalVelocity(3);
    
    %access pose from robot class static methods
    poseChange = robot1.getPose(dt, xdot, ydot, omega);     %returns dx, dy, dtheta (change in each var given over the given step)
    poseX(i) = poseX(i-1) + poseChange(1);
    poseY(i) = poseY(i-1) + poseChange(2);
    poseThetaLocal = poseTheta(i-1) + poseChange(3);        %accumulate robot heading
    
    poseTheta(i) = wrapTo2Pi(poseThetaLocal);               %store and export incremental robot heading
    
    robot1.theta0 = poseThetaLocal;                         %update robot heading based on most recent accumulation
    
end

%Calc Trajectory Diameter
trajDia = abs(max(poseX) - min(poseX));

%Get robot heading in degrees
poseTheta = wrapTo360(rad2deg(poseTheta));

%Plotting
figure(1111)

plot(t,poseY(1:end-1), 'r.')
hold on
plot(t,poseX(1:end-1), 'm.')
xlabel('time (s)')
ylabel('X and Y Position (m)')
title(['Robot Position Over ',num2str(t(end)),' Seconds'])
legend('t vs. y', 't vs. x')

figure(1112)
plot(poseX,poseY, 'b.')
title('X vs. Y Position')
legend(sprintf('Maximum Displacement = %0.3f m',trajDia),'Location','best')
xlabel('X Position (m)')
ylabel('Y Position (m)')

figure(1113)
plot(t,poseTheta(1:end-1))
title('Theta vs. Time')
xlabel('Time (s)')
ylabel('Mapped Heading (0-360 Degrees)')

figure(1114)
dia = 0.0762;
n = 6;
t = 0:n*pi/(length(t)):n*pi;
r = dia - cos(t)*0.07*dia;
%r = wrap2Number(r,1);

% order = 4;
% framelen = 19;
% sgf = sgolayfilt(r,order,framelen);
%plot (t,r)
%hold on
% plot(sgf,'.-')

zmin = -0.9;
zmax = 0.9;
[X,Y,Z] = cylinder(dia);
Z = zmin + (zmax-zmin)*Z;
surf(X,Z,Y,'FaceColor',[.7 .7 .7],'FaceAlpha',0.5,'EdgeColor', 'none') ;
hold on

%To do: 11/27/20: Change Plotting method to just project on a cylinder,
%makes things simpler: https://stackoverflow.com/questions/26408863/bending-a-plane-into-a-closed-surface-cylinder

phi = ((poseX)/(max(poseX)-min(poseX))*2*pi) + pi/4;
xp = dia*cos(phi);
yp = dia*sin(phi);
zp = poseY ;

% for i = 1:length(xp)
%     for j = 1:length(X(:,1))
%         for k = 1:length(X(1,:))
%             if (xp(i) - X(j,k)) > 0.001
%                 xp(i) = X(j,k);
%             end
%         end
%     end
% end
% for i = 1:length(yp)
%     for j = 1:length(Y(:,1))
%         for k = 1:length(Y(1,:))
%             if (yp(i) - Y(j,k)) > 0.001
%                 yp(i) = Y(j,k);
%             end
%         end
%     end
% end

hp = plot3(xp,zp,yp,'-o','color', rand(1,3) );
title('Simulated 3D Trajectories on Colon Wall')
xlabel('X Position (m)')
ylabel('Y Position (m)')
zlabel('Z Position (m)')







