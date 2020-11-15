%// Data
xmin = -3;
xmax = 3; %// this piece will get folded into a cylinder
Rc = 5; %// cylinder radius
zmaxc = 5; %// cylinder max z
zminc = -5; %// cylinder min z

%// Spiral
t = linspace(0,1,1000);
r = 1+2*t;
theta = 2*pi*3*t;
x1 = r.*cos(theta);
y1 = r.*sin(theta); %// example spiral. Defined by x1, y1

%// Do the bending
z2 = y1;
phi = (x1-xmin)/(xmax-xmin)*2*pi;
x2 = Rc*cos(phi);
y2 = Rc*sin(phi);

%// Plot cylinder
[xc yc zc] = cylinder(Rc*ones(1,100),100);
zc = zminc + (zmaxc-zminc)*zc;
surf(xc,yc,zc)
shading flat
hold on

%// Plot bent spiral
plot3(x2,y2,z2, 'k.-');