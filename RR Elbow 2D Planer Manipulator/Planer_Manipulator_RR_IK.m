clc;close;clear;

% end-effector rotation matrix w.r.t. base
RE= [0.25819   -0.9659   0;...
     0.9659     0.25819   0;...
     0   0    1];

DE=[2.249;2.931;0];
%DE=[1;1;0];

a1=2;
a2=2;

phi_rad = atan2(RE(2,1), RE(1,1));

phi_deg = phi_rad*(180/pi);

wx=DE(1,1)-a2*cos(phi_rad);
wy=DE(2,1)-a2*sin(phi_rad);

theta1_rad=atan2(wy,wx);
theta1_deg = theta1_rad*(180/pi)

theta2_deg=phi_deg-theta1_deg
