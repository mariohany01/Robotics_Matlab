%this inverse kinematics is based on the yt video thats a clone for his
%code
clc;close;clear;

% end-effector rotation matrix w.r.t. base
R60= [-0.1268   -0.9427   -0.3086;...
    0.9268   -0.0017   -0.3756;...
    0.3536   -0.3336    0.8739];

% end-effector position w.r.t. base
d60=[-0.2283;0.0216;0.4788];

% robot geometry
d1=0.25;
d2=0.15; 
d6=0.1;

% wrist position
OC=d60-d6*R60(:,3);
xc=OC(1);
yc=OC(2);
zc=OC(3);

% inverse position problem
d3=sqrt(xc^2+yc^2+(zc-d1)^2-d2^2)
D=(zc-d1)/d3;

theta2=atan2d(D,sqrt(1-D^2))

M=[-d2 -d3*cosd(theta2);-d3*cosd(theta2) d2];
RH=[xc;yc];
sol=inv(M)*RH;
theta1=atan2d(sol(1),sol(2))

% Function to compute DH transformation matrix
DH_A = @(theta, d, a, alpha) [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                              sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                              0,           sin(alpha),              cos(alpha),            d;
                              0,           0,                       0,                     1];

% inverse orientation problem
DH_table=[theta1*pi/180 d1 0 -pi/2;...
          (theta2)*pi/180-pi/2 d2 0 pi/2;...
          -pi/2 d3 0 0];
      T=DH_A(DH_table(1,1),DH_table(1,2),DH_table(1,3),DH_table(1,4));

for ii=2:size(DH_table,1)
    A=DH_A(DH_table(ii,1),DH_table(ii,2),DH_table(ii,3),DH_table(ii,4));
    T=T*A;
end

R30=T(1:3,1:3);

R63=R30'*R60;

theta4=atan2d(R63(2,3),R63(1,3))
theta5=atan2d(sqrt(R63(2,3)^2+R63(1,3)^2),R63(3,3))
theta6=atan2d(R63(3,2),-R63(3,1))