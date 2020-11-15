clc; clear;
%Noah Ossanna 10/25/20
%Script to plot robot trajectories for different geometries using:
%basicRobot Class

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
robot1 = basicRobot('robot1', 4, 4, pi/2, -pi/2, 0, pi, 0.0015, 0.0025, 0.009, pi/2);

%Pre-allocate pose storage vectors for plotting
poseX = zeros(1,length(t));
poseY = zeros(1,length(t));
poseTheta = zeros(1,length(t));
poseTheta(1) = robot1.theta0;

%Begin time stepping
for i = 2:length(t)+1
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
%title('Robot Position Over ', num2str(t(end)),' Seconds')
plot(t,poseY(1:end-1), 'r.')
hold on
plot(t,poseX(1:end-1), 'm.')
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

zmin = -0.05;
zmax = 0.05;

[X,Y,Z] = cylinder(r);
Z = zmin + (zmax-zmin)*Z;
hs = surf(X,Z,Y);
set(hs,'FaceColor',[.7 .7 .7],'FaceAlpha',0.5,'EdgeColor', 'none') ;
hold on

%To do: 11/14/20: research how to map similar xp, yp, to x, y of cylinder,
%basically just want traj to change in axial direction, might need new
%projection method
phi = abs((poseX)/(dia)*2*pi);
xp = r.*cos(phi);
yp = r.*sin(phi);
zp = poseY ;

hp = plot3(xp,zp,yp,'-ok') ;


% function lon = wrap2Number(lon,number)
% 
% positiveInput = (lon > 0);
% lon = mod(lon, number);
% lon((lon == 0) & positiveInput) = number;
% end

% 
% z2 = y1;
% phi = (x1-xmin)/(xmax-xmin)*2*pi;
% x2 = Rc*cos(phi);
% y2 = Rc*sin(phi);
% 
% %// Plot cylinder
% [xc yc zc] = cylinder(Rc*ones(1,100),100);
% zc = zminc + (zmaxc-zminc)*zc;
% surf(xc,yc,zc)
% shading flat
% hold on
% 
% %// Plot bent spiral
% plot3(x2,y2,z2, 'k.-');





