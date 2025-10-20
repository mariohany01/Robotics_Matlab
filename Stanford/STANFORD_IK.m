%this is inverse kinematics Based on my papers and baesd on what i learned 

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


%Talla3 kol wahda lwahdaha

% Spherical position
%equation el split OC=OP-CE=DE-(d6*Z60)

OC=d60-d6*R60(:,3);
xc=OC(1);
yc=OC(2);
zc=OC(3);



% Hena momken ngeib equations el Xc Yc Zc men el F bta3 el robot le awel 3
% links men el DH parameters
%check File STANFORD FK BASE WRISTT
%XC=     - d2*sin(theta1) - d3*cos(theta1)*cos(theta2)
%YC=     d2*cos(theta1) - d3*cos(theta2)*sin(theta1)
%ZC=     d1 + d3*sin(theta2)

%finding Equations le kol Var lwahdo Theta1 Theta2 D3
%inverse position problem
%Solving d3 by pythogras

d3=sqrt(xc^2+yc^2+(zc-d1)^2-d2^2)


%solving for theta2 
%From ZC Zabatha ZC=d1+d3*sin(theta2)
%sin(theta2)=(Zc-d1)/d3 = X
%Sin^2(theta2)+cos^2(theta2)=1 we found Y 
X=(zc-d1)/d3;
Y=sqrt(1-X^2);

theta2=atan2d(X,Y) %used positive Y not negative because ik that Theta 2 is postive as iam assuming but the right way is to assume 2 angles for theta

%By some Maths for 2 equations Xc And Yc
% |  -d2   -d3c2|*|S1|=|Xc|
% | -d3C2    d2 |*|C1|=|Yc|
%       M         need  RHS

M=[-d2 -d3*cosd(theta2);-d3*cosd(theta2) d2];
RH=[xc;yc];
trig_theta1=inv(M)*RH; %=Vector |S1;C1|
theta1=atan2d(trig_theta1(1),trig_theta1(2))


% Function to compute DH transformation matrix
DH_A = @(theta, d, a, alpha) [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                              sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                              0,           sin(alpha),              cos(alpha),            d;
                              0,           0,                       0,                     1];

% inverse orientation problem
DH_table=[theta1*pi/180 d1 0 -pi/2;...
          theta2*pi/180-pi/2 d2 0 pi/2;...
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%

L(1) = Link([0 d1 0 -pi/2]); % Joint 1: Revolute
L(2) = Link([0 d2 0 pi/2]);  % Joint 2: Revolute (removed -pi/2 offset) but add it in the angle
L(3) = Link([-pi/2 0 0 0 1]); % Joint 3: Prismatic (simplified)
L(4) = Link([0 0 0 -pi/2]);  % Joint 4: Revolute
L(5) = Link([0 0 0 pi/2]);   % Joint 5: Revolute
L(6) = Link([0 d6 0 0]);     % Joint 6: Revolute
L(3).qlim = [0 0.5];   % extension range

Rob = SerialLink (L);
    
Rob.name ='RRPRRR';


q_end = [theta1, theta2, d3, theta4, theta5, theta6];
Rob.plot(q_end, 'workspace', [-0.7 0.7 -0.7 0.7 -0.1 0.7]);

% Calculate forward kinematics for the end pose
fk_matrix = Rob.fkine(q_end); %Change q_end - q_start

disp('Forward kinematics for the end configuration:');
disp(fk_matrix);

%Add plot
Rob.plot ([q_end]) %Change q_end - q_start

