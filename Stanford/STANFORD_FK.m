clc;close;clear;

theta1=30*pi/180;
theta2=45*pi/180;
d3=0.2;
theta4=60*pi/180;
theta5=20*pi/180;
theta6=90*pi/180;

d1=0.25;
d2=0.15; 
d6=0.1;

DH_table=[theta1 d1 0 -pi/2;...
          theta2-pi/2 d2 0 pi/2;...
          -pi/2 d3 0 0;...
          theta4 0 0 -pi/2;...
          theta5 0 0 pi/2;...
          theta6 d6 0 0];

T=DH_A(DH_table(1,1),DH_table(1,2),DH_table(1,3),DH_table(1,4));

for ii=2:size(DH_table,1)
    A=DH_A(DH_table(ii,1),DH_table(ii,2),DH_table(ii,3),DH_table(ii,4));
    T=T*A;
end

disp('end-effector position from base origin in inches: ');
disp(T(1:3,4))
disp('end-effector frame rotation matrix w.r.t base frame: ');
disp(T(1:3,1:3))